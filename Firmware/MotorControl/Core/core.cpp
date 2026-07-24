/**
 * @file core.cpp
 * @author setsuna
 * @brief MotorControl C ABI shim and MotorController slow-loop core
 * @version 0.1
 * @date 2026-07-23
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "command_sanitizer.hpp"
#include "config.hpp"
#include "motor_control_state.h"
#include "motor_controller.hpp"

#include "MotorControl.h"
#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_math.h"
#include "foc_pwm.h"
#include "motor_control_vf.h"
#include "enum_utils.hpp"
#include "irq_guard.hpp"
#include "types.hpp"

#include <stdint.h>

volatile MotorControlCommand_t g_mc_cmd = {
    0U,
    0U,
    0,
    0,
    0,
    0,
    cms32::motor::SpeedLoopConfig::iq_limit,
    cms32::motor::CurrentLoopConfig::kp,
    cms32::motor::CurrentLoopConfig::ki,
    cms32::motor::SpeedLoopConfig::kp,
    cms32::motor::SpeedLoopConfig::ki,
    0,
    cms32::motor::PositionLoopConfig::kp,
    cms32::motor::PositionLoopConfig::err_shift,
    cms32::motor::PositionLoopConfig::speed_limit_rpm,
    cms32::motor::PositionLoopConfig::deadband_counts,
    cms32::motor::CurrentLoopConfig::voltage_limit,
    cms32::motor::OpenLoopConfig::speed_ref,
    cms32::motor::OpenLoopConfig::vf_voltage,
    cms32::motor::OpenLoopConfig::if_id_ref,
    cms32::motor::OpenLoopConfig::if_iq_ref,
    cms32::motor::OpenLoopConfig::timeout_ms,
    0,
    0,
};

namespace
{

constexpr bool command_enabled(const MotorControlCommand_t& command) noexcept
{
    return command.enable != 0U;
}

constexpr cms32::motor::ControlMode
active_mode_for(const MotorControlCommand_t& command) noexcept
{
    return command_enabled(command)
               ? cms32::motor::to_control_mode(command.control_mode)
               : cms32::motor::ControlMode::Off;
}

} // namespace

namespace cms32::motor
{

MotorController g_motor;

ControlMode MotorController::currentMode() const noexcept
{
    return to_control_mode(runtime.mode);
}

ControlState MotorController::currentState() const noexcept
{
    return static_cast<ControlState>(runtime.state);
}

void MotorController::setState(ControlState state) noexcept
{
    runtime.state = static_cast<MotorControlStateRaw_t>(
        cms32::support::to_underlying(state));
}

void MotorController::setFault(ControlFault fault) noexcept
{
    runtime.fault = static_cast<MotorControlFaultRaw_t>(
        cms32::support::to_underlying(fault));
}

void MotorController::setMode(ControlMode mode) noexcept
{
    runtime.mode = static_cast<MotorControlModeRaw_t>(
        cms32::support::to_underlying(mode));
}

bool MotorController::readyForMode(ControlMode mode) const noexcept
{
    if (mode == ControlMode::VfOpenLoop)
    {
        return check.current_ok != 0U;
    }

    if (is_closed_loop_mode(mode))
    {
        return check.current_ok != 0U;
    }

    return false;
}

ControlFault MotorController::faultForNotReady(ControlMode mode) const noexcept
{
    (void)mode;

    if (check.current_ok == 0U)
    {
        return ControlFault::Current;
    }

    return ControlFault::Current;
}

bool MotorController::safeStateRequired() const noexcept
{
    return (currentState() != ControlState::Fault) || (runtime.pwm_output != 0U);
}

void MotorController::resetForModeChange(ControlMode next_mode) noexcept
{
    if ((next_mode == ControlMode::Position) || (next_mode == ControlMode::Off))
    {
        diag.speed_reset_count++;
        speedControllerReset();
    }
    else
    {
        speedReset();
    }
    positionReset();
    currentReset();

    if (next_mode == ControlMode::VfOpenLoop)
    {
        vfResetForMode(cms32::support::to_underlying(next_mode));
    }
}

void MotorController::applyVfVoltageMirror(
    const MotorControlCommand_t& next_command) noexcept
{
    if ((runtime.enabled != 0U) && (currentMode() == ControlMode::VfOpenLoop))
    {
        curr_set_vf_voltage(next_command.vf_voltage);
    }
    else
    {
        curr_set_vf_voltage(0);
    }
}

void MotorController::enterIdleIfDisabled() noexcept
{
    if (runtime.enabled != 0U)
    {
        return;
    }

    if ((runtime.pwm_output != 0U) || (currentState() != ControlState::Idle))
    {
        enterSafeState();
    }

    setState(ControlState::Idle);
    setFault(ControlFault::None);
    publishDebugState();
}

void MotorController::refreshSlowChecks(ControlMode mode) noexcept
{
    current.phase.u = curr_u();
    current.phase.v = curr_v();
    current.phase.w = curr_w();
    check.current_ok = currentOk();
    check.pwm_off_safe = (pwm_is_off_safe() != 0U) ? 1U : 0U;
    check.ma600_ok =
        is_closed_loop_mode(mode)
            ? static_cast<uint8_t>((encoder.ok != 0U) || (encoder.initialized == 0U))
            : static_cast<uint8_t>(mode == ControlMode::VfOpenLoop);
    check.ready_closed_loop = static_cast<uint8_t>(readyForMode(mode));
}

void MotorController::enterReadyState(ControlMode mode) noexcept
{
    if (currentState() != ControlState::ClosedLoop)
    {
        if (mode == ControlMode::Position)
        {
            speedControllerReset();
        }
        else if (mode != ControlMode::VfOpenLoop)
        {
            speedReset();
        }
        currentReset();
    }

    setState(ControlState::ClosedLoop);
    setFault(ControlFault::None);
    publishDebugState();
}

void MotorController::enterFaultState(ControlFault fault) noexcept
{
    const bool should_enter_safe = safeStateRequired();

    setState(ControlState::Fault);
    setFault(fault);

    if (should_enter_safe)
    {
        enterSafeState();
    }
    publishDebugState();
}

void MotorController::applyRuntimeCommands(const CommandSanitizer& sanitizer,
                                           const MotorControlCommand_t& next_command)
    noexcept
{
    command.current = sanitizer.current_command(next_command);
    command.speed = sanitizer.speed_command(next_command);
    command.position = sanitizer.position_command(next_command);
    command.vf = sanitizer.vf_command(next_command);
}

void MotorController::publishDebugState() noexcept
{
    debug.state = currentState();
    debug.mode = currentMode();
    debug.fault = static_cast<ControlFault>(runtime.fault);
    debug.enabled = runtime.enabled != 0U;
    debug.pwm_output = runtime.pwm_output != 0U;
    debug.speed_ref_cmd_rpm = speedCountsToRpm(command.speed.speed_ref);
    debug.speed_ref_active_rpm = speedCountsToRpm(speed.ref_active.value);
    debug.speed_fb_rpm = speedCountsToRpm(speed.feedback);
    debug.speed_err_rpm = speed.err_rpm;
    debug.position_ref = position.target;
    debug.position_error = position.error;
    debug.position_speed_ref_rpm = position.speed_ref_rpm;
    debug.position_at_target = position.at_target != 0U;
}

void publish_debug_state() noexcept
{
    g_motor.publishDebugState();
}

void MotorController::init() noexcept
{
    setState(ControlState::Idle);
    setFault(ControlFault::None);
    runtime.enabled = 0U;
    setMode(ControlMode::Off);
    runtime.pwm_output = 0U;
    diag = MotorControlDiag_t{};
    current.loop_div = 0U;
    speed.sample_div = 0U;
    command.current = MCCurrentCommand_t{0,
                                         0,
                                         SpeedLoopConfig::iq_limit,
                                         CurrentLoopConfig::kp,
                                         CurrentLoopConfig::ki,
                                         CurrentLoopConfig::voltage_limit,
                                         0,
                                         0};
    command.speed = MCSpeedCommand_t{0,
                                     SpeedLoopConfig::kp,
                                     SpeedLoopConfig::ki,
                                     SpeedLoopConfig::iq_limit};
    command.position = MCPositionCommand_t{0,
                                           PositionLoopConfig::kp,
                                           PositionLoopConfig::err_shift,
                                           PositionLoopConfig::speed_limit_rpm,
                                           PositionLoopConfig::deadband_counts};
    command.vf = MCVfCommand_t{OpenLoopConfig::speed_ref,
                               OpenLoopConfig::vf_voltage,
                               OpenLoopConfig::timeout_ms};

    encoderReset();
    positionReset();

    foc_pi_init(&speed.pi,
                SpeedLoopConfig::kp,
                SpeedLoopConfig::ki,
                -SpeedLoopConfig::iq_limit,
                SpeedLoopConfig::iq_limit,
                SpeedLoopConfig::err_shift);
    foc_pi_init(&current.pi_d,
                CurrentLoopConfig::kp,
                CurrentLoopConfig::ki,
                -CurrentLoopConfig::voltage_limit,
                CurrentLoopConfig::voltage_limit,
                CurrentLoopConfig::pi_shift);
    foc_pi_init(&current.pi_q,
                CurrentLoopConfig::kp,
                CurrentLoopConfig::ki,
                -CurrentLoopConfig::voltage_limit,
                CurrentLoopConfig::voltage_limit,
                CurrentLoopConfig::pi_shift);

    current.phase = FocPhaseCurrent_t{0, 0, 0};
    current.dq = FocDq_t{0, 0};
    current.id_ref_active.reset();
    current.iq_ref_active.reset();
    output.voltage_ab = FocAlphaBeta_t{0, 0};
    output.voltage_dq = FocDq_t{0, 0};
    output.voltage_theta = 0U;
    output.duty = FocDuty_t{PwmConfig::duty_center,
                            PwmConfig::duty_center,
                            PwmConfig::duty_center};
    output.voltage_limited = 0U;
    check = MotorControlCheck_t{0U, 1U, 1U, 0U};

    vfInit();
    pwm_off();
    publishDebugState();
}

void MotorController::applyCommand(const volatile MotorControlCommand_t& input) noexcept
{
    diag.command_apply_count++;

    const CommandSanitizer sanitizer;
    const MotorControlCommand_t next_command =
        sanitizer.sanitize(sanitizer.snapshot(input));
    const ControlMode next_mode = active_mode_for(next_command);

    {
        const cms32::support::AdcIrqGuard guard;
        (void)guard;

        if (next_mode != currentMode())
        {
            resetForModeChange(next_mode);
        }

        runtime.enabled = static_cast<uint8_t>(command_enabled(next_command));
        setMode(next_mode);
        applyRuntimeCommands(sanitizer, next_command);
    }

    applyVfVoltageMirror(next_command);
    enterIdleIfDisabled();
    publishDebugState();
}

void MotorController::runSlowLoop() noexcept
{
    const ControlMode mode = currentMode();
    refreshSlowChecks(mode);

    if (runtime.enabled == 0U)
    {
        diag.slow_loop_count++;
        publishDebugState();
        return;
    }

    if (is_supported_run_mode(mode))
    {
        if (check.ready_closed_loop != 0U)
        {
            enterReadyState(mode);
        }
        else
        {
            enterFaultState(faultForNotReady(mode));
        }
    }
    else
    {
        enterFaultState(ControlFault::UnsupportedMode);
    }

    diag.slow_loop_count++;
    publishDebugState();
}

uint8_t MotorController::fastLoopFromAdcIrq() noexcept
{
    const uint8_t sample_ready = bsp_adc_irq();
    if (sample_ready == 0U)
    {
        return 0U;
    }

    if ((runtime.enabled == 0U) || (currentState() != ControlState::ClosedLoop))
    {
        return 1U;
    }

    switch (currentMode())
    {
    case ControlMode::VfOpenLoop:
        vfRunFastLoop();
        break;
    case ControlMode::Speed:
        runCurrentFastLoop(ControlMode::Speed);
        break;
    case ControlMode::Position:
        runCurrentFastLoop(ControlMode::Position);
        break;
    case ControlMode::Current:
        runCurrentFastLoop(ControlMode::Current);
        break;
    default:
        break;
    }

    return 1U;
}

} // namespace cms32::motor

extern "C" void MotorControl_Init(void)
{
    cms32::motor::g_motor.init();
}

extern "C" void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command)
{
    if (command == nullptr)
    {
        return;
    }

    cms32::motor::g_motor.applyCommand(*command);
}

extern "C" void MotorControl_RunSlowLoop(void)
{
    cms32::motor::g_motor.runSlowLoop();
}

extern "C" uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    return cms32::motor::g_motor.fastLoopFromAdcIrq();
}
