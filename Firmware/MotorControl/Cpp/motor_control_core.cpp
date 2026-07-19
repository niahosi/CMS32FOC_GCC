/**
 * @file motor_control_core.cpp
 * @author setsuna
 * @brief MotorControl C ABI 实现、内部状态、慢环状态机、快环分发
 * @version 0.1
 * @date 2026-07-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "BoardConfig.h"
#include "BoardConfig.h"
#include "MotorControl.h"
#include "TuneConfig.h"
#include "foc_math.h" //已经处理过C/C++不兼容
#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_pwm.h"
#include "motor_control_internal.h"
#include "motor_control_vf.h"
#include "enum_utils.hpp"
#include "irq_guard.hpp"
#include "motor_command_sanitizer.hpp"
#include "motor_control_types.hpp"

#include <stdint.h>

// 初始化电机命令
volatile MotorControlCommand_t g_motor_cmd = {
    0U,
    0U,
    0,
    0,
    0,
    0,
    CTRL_SPD_IQ_LIMIT,
    CTRL_CUR_KP,
    CTRL_CUR_KI,
    CTRL_SPD_KP,
    CTRL_SPD_KI,
    CTRL_CUR_V_LIMIT,
    OL_SPEED_REF,
    OL_VF_VOLTAGE,
    OL_IF_ID_REF,
    OL_IF_IQ_REF,
    OL_TIMEOUT_MS,
    0,
    0,
};

volatile MotorControlWatch_t g_motor_watch;

// helper
namespace
{
using cms32::motor::ControlFault;
using cms32::motor::ControlMode;
using cms32::motor::ControlState;
using cms32::motor::is_closed_loop_mode;
using cms32::motor::is_supported_run_mode;
using cms32::motor::to_control_mode;
using cms32::support::to_underlying;

MotorControlCState s_mc;

constexpr bool command_enabled(const MotorControlCommand_t& command) noexcept
{
    return command.enable != 0U;
}

constexpr ControlMode active_mode_for(const MotorControlCommand_t& command) noexcept
{
    return command_enabled(command) ? to_control_mode(command.control_mode) : ControlMode::Off;
}

ControlMode current_mode() noexcept
{
    return to_control_mode(s_mc.mode);
}

ControlState current_state() noexcept
{
    return static_cast<ControlState>(s_mc.state);
}

void set_state(ControlState state) noexcept
{
    s_mc.state = to_underlying(state);
}

void set_fault(ControlFault fault) noexcept
{
    s_mc.fault = to_underlying(fault);
}

bool has_valid_encoder_for_closed_loop() noexcept
{
    return (s_mc.encoder_ok != 0U) || (s_mc.encoder_initialized == 0U);
}

bool ready_for_mode(ControlMode mode) noexcept
{
    if (mode == ControlMode::VfOpenLoop)
    {
        return s_mc.check.current_ok != 0U;
    }

    if (is_closed_loop_mode(mode))
    {
        return (s_mc.check.current_ok != 0U) && has_valid_encoder_for_closed_loop();
    }

    return false;
}

ControlFault fault_for_not_ready(ControlMode mode) noexcept
{
    if (s_mc.check.current_ok == 0U)
    {
        return ControlFault::Current;
    }

    if (is_closed_loop_mode(mode) && (s_mc.check.ma600_ok == 0U))
    {
        return ControlFault::Encoder;
    }

    return ControlFault::Current;
}

bool safe_state_required() noexcept
{
    return (current_state() != ControlState::Fault) || (s_mc.pwm_output != 0U);
}

void reset_for_mode_change(ControlMode next_mode) noexcept
{
    MotorControl_SpeedReset(&s_mc);
    MotorControl_CurrentReset(&s_mc);

    if (next_mode == ControlMode::VfOpenLoop)
    {
        MotorControlVf_ResetForMode(to_underlying(next_mode));
    }
}

void apply_vf_voltage_mirror(const MotorControlCommand_t& command) noexcept
{
    if ((s_mc.enabled != 0U) && (current_mode() == ControlMode::VfOpenLoop))
    {
        curr_set_vf_voltage(command.vf_voltage);
    }
    else
    {
        curr_set_vf_voltage(0);
    }
}

void enter_idle_if_disabled() noexcept
{
    if (s_mc.enabled != 0U)
    {
        return;
    }

    if ((s_mc.pwm_output != 0U) || (current_state() != ControlState::Idle))
    {
        MotorControl_InternalEnterSafeState(&s_mc);
    }

    set_state(ControlState::Idle);
    set_fault(ControlFault::None);
}

void refresh_slow_checks(ControlMode mode) noexcept
{
    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    s_mc.check.current_ok = MotorControl_InternalCurrentOk(&s_mc);
    s_mc.check.pwm_off_safe = (pwm_is_off_safe() != 0U) ? 1U : 0U;
    s_mc.check.ma600_ok = is_closed_loop_mode(mode)
                              ? static_cast<uint8_t>(has_valid_encoder_for_closed_loop())
                              : static_cast<uint8_t>(mode == ControlMode::VfOpenLoop);
    s_mc.check.ready_closed_loop = static_cast<uint8_t>(ready_for_mode(mode));
}

void enter_ready_state(ControlMode mode) noexcept
{
    if (current_state() != ControlState::ClosedLoop)
    {
        if (mode != ControlMode::VfOpenLoop)
        {
            MotorControl_SpeedReset(&s_mc);
        }
        MotorControl_CurrentReset(&s_mc);
    }

    set_state(ControlState::ClosedLoop);
    set_fault(ControlFault::None);
}

void enter_fault_state(ControlFault fault) noexcept
{
    const bool should_enter_safe = safe_state_required();

    set_state(ControlState::Fault);
    set_fault(fault);

    if (should_enter_safe)
    {
        MotorControl_InternalEnterSafeState(&s_mc);
    }
}

void apply_runtime_commands(const cms32::motor::CommandSanitizer& sanitizer,
                            const MotorControlCommand_t& command) noexcept
{
    s_mc.current_command = sanitizer.current_command(command);
    s_mc.speed_command = sanitizer.speed_command(command);
    s_mc.vf_command = sanitizer.vf_command(command);
}

} // namespace

extern "C" void MotorControl_Init(void)
{
    set_state(ControlState::Idle);
    set_fault(ControlFault::None);
    s_mc.enabled = 0U;
    s_mc.mode = to_underlying(ControlMode::Off);
    s_mc.pwm_output = 0U;
    s_mc.command_apply_count = 0U;
    s_mc.slow_loop_count = 0U;
    s_mc.fast_loop_count = 0U;
    s_mc.speed_reset_count = 0U;
    s_mc.safe_state_count = 0U;
    s_mc.speed_loop_count = 0U;
    s_mc.speed_deadband_count = 0U;
    s_mc.current_loop_div = 0U;
    s_mc.speed_sample_div = 0U;
    s_mc.current_command = MCCurrentCommand_t{
        0, 0, CTRL_SPD_IQ_LIMIT, CTRL_CUR_KP, CTRL_CUR_KI, CTRL_CUR_V_LIMIT, 0, 0};
    s_mc.speed_command = MCSpeedCommand_t{0, CTRL_SPD_KP, CTRL_SPD_KI, CTRL_SPD_IQ_LIMIT};
    s_mc.vf_command = MCVfCommand_t{OL_SPEED_REF, OL_VF_VOLTAGE, OL_TIMEOUT_MS};

    MotorControl_EncoderReset(&s_mc);

    foc_pi_init(&s_mc.speed_pi,
                CTRL_SPD_KP,
                CTRL_SPD_KI,
                -CTRL_SPD_IQ_LIMIT,
                CTRL_SPD_IQ_LIMIT,
                CTRL_SPD_ERR_SHIFT);
    foc_pi_init(&s_mc.current_pi_d,
                CTRL_CUR_KP,
                CTRL_CUR_KI,
                -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT,
                CTRL_CUR_PI_SHIFT);
    foc_pi_init(&s_mc.current_pi_q,
                CTRL_CUR_KP,
                CTRL_CUR_KI,
                -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT,
                CTRL_CUR_PI_SHIFT);

    s_mc.current = FocPhaseCurrent_t{0, 0, 0};
    s_mc.current_dq = FocDq_t{0, 0};
    s_mc.id_ref_active = 0;
    s_mc.iq_ref_active = 0;
    s_mc.voltage_ab = FocAlphaBeta_t{0, 0};
    s_mc.voltage_dq = FocDq_t{0, 0};
    s_mc.voltage_theta = 0U;
    s_mc.duty = FocDuty_t{PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    s_mc.voltage_limited = 0U;
    s_mc.check = MotorControlCheck_t{0U, 1U, 1U, 0U};

    MotorControlVf_Init();
    pwm_off();
}

extern "C" void MotorControl_ApplyCommand(const volatile MotorControlCommand_t *command)
{
    if (command == nullptr)
    {
        return;
    }

    s_mc.command_apply_count++;

    const cms32::motor::CommandSanitizer sanitizer;
    const MotorControlCommand_t next_command = sanitizer.sanitize(sanitizer.snapshot(*command));
    const ControlMode next_mode = active_mode_for(next_command);
    // 小作用域，构造时：屏蔽ADC_IRQn 析构时：重新打开ADC_IRQn
    {
        const cms32::support::AdcIrqGuard guard;
        (void)guard;

        if (next_mode != current_mode())
        {
            reset_for_mode_change(next_mode);
        }

        s_mc.enabled = static_cast<uint8_t>(command_enabled(next_command));
        s_mc.mode = to_underlying(next_mode);
        apply_runtime_commands(sanitizer, next_command);
    }

    apply_vf_voltage_mirror(next_command);
    enter_idle_if_disabled();
}

extern "C" void MotorControl_RunSlowLoop(void)
{
    const ControlMode mode = current_mode();
    refresh_slow_checks(mode);

    if (s_mc.enabled == 0U)
    {
        s_mc.slow_loop_count++;
        return;
    }

    // 有冻结的模式，尝试兼容被冻结不编译的程序
    if (is_supported_run_mode(mode))
    {
        if (s_mc.check.ready_closed_loop != 0U)
        {
            enter_ready_state(mode);
        }
        else
        {
            enter_fault_state(fault_for_not_ready(mode));
        }
    }
    else
    {
        enter_fault_state(ControlFault::UnsupportedMode);
    }

    s_mc.slow_loop_count++;
}

extern "C" uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    const uint8_t sample_ready = bsp_adc_irq();
    if (sample_ready == 0U)
    {
        return 0U;
    }

    // 采样成功，但控制层当前不允许输出快环。
    if ((s_mc.enabled == 0U) || (current_state() != ControlState::ClosedLoop))
    {
        return 1U;
    }
    switch (current_mode())
    {
    case ControlMode::VfOpenLoop:
        MotorControlVf_RunFastLoop(&s_mc);
        break;
    case ControlMode::Speed:
        MotorControl_CurrentRunFastLoop(&s_mc, 1U);
        break;
    case ControlMode::Current:
        MotorControl_CurrentRunFastLoop(&s_mc, 0U);
        break;
    default:
        break;
    }

    return 1U;
}

extern "C" void MotorControl_GetWatch(MotorControlWatch_t *out)
{
    if (out == nullptr)
    {
        return;
    }

    MotorControl_WatchFill(&s_mc, out);
}

extern "C" void MotorControl_UpdateWatch(volatile MotorControlWatch_t *out)
{
    if (out == nullptr)
    {
        return;
    }

    MotorControlWatch_t snapshot;
    MotorControl_WatchFill(&s_mc, &snapshot);
    MotorControl_WatchCopyToVolatile(out, &snapshot);
}
