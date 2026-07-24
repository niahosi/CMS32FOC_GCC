/**
 * @file vf.cpp
 * @author setsuna
 * @brief MotorControl VF emergency open-loop path
 * @version 0.1
 * @date 2026-07-23
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "motor_controller.hpp"

#include "MotorControl.h"
#include "foc_curr.h"
#include "motor_control_vf.h"
#include "enum_utils.hpp"

#include <stdint.h>

namespace
{

using cms32::motor::ControlFault;
using cms32::motor::ControlMode;
using cms32::motor::ControlState;
using cms32::motor::EncoderConfig;
using cms32::motor::OpenLoopConfig;
using cms32::motor::SpeedLoopConfig;
using cms32::support::to_underlying;

constexpr uint32_t timeout_ticks_from_ms(uint16_t timeout_ms) noexcept
{
    return static_cast<uint32_t>(timeout_ms) * 2U;
}

constexpr int32_t round_shift_speed_to_theta_step(int32_t speed_ref) noexcept
{
    return (speed_ref * OpenLoopConfig::speed_to_theta_step +
            (1L << (OpenLoopConfig::speed_to_theta_shift - 1U))) >>
           OpenLoopConfig::speed_to_theta_shift;
}

} // namespace

namespace cms32::motor
{

static void reset_open_loop_theta(MotorController& motor) noexcept
{
    motor.vf.open_loop_ticks = 0U;
    motor.vf.open_loop_theta = 0U;
    motor.vf.open_loop_theta_acc = 0;
    motor.vf.open_loop_reset_count++;
}

static void refresh_timeout(MotorController& motor) noexcept
{
    motor.vf.open_loop_timeout_ticks =
        timeout_ticks_from_ms(motor.command.vf.open_loop_timeout_ms);
}

static bool timed_out(const MotorController& motor) noexcept
{
    return (motor.vf.open_loop_timeout_ticks > 0U) &&
           (motor.vf.open_loop_ticks >= motor.vf.open_loop_timeout_ticks);
}

static uint16_t advance_open_loop_theta(MotorController& motor) noexcept
{
    const int32_t step = round_shift_speed_to_theta_step(
        motor.command.vf.open_loop_speed_ref);

    motor.vf.open_loop_theta_acc += step;
    motor.vf.open_loop_theta = static_cast<uint16_t>(
        static_cast<uint32_t>(motor.vf.open_loop_theta_acc) & 0xFFFFUL);
    motor.vf.open_loop_ticks++;

    return static_cast<uint16_t>(
        static_cast<int32_t>(motor.vf.open_loop_theta) *
        static_cast<int32_t>(EncoderConfig::direction));
}

static void sample_phase_currents(MotorController& motor) noexcept
{
    motor.current.phase.u = curr_u();
    motor.current.phase.v = curr_v();
    motor.current.phase.w = curr_w();
}

static void enter_fault(MotorController& motor, ControlFault fault) noexcept
{
    motor.runtime.state =
        static_cast<MotorControlStateRaw_t>(to_underlying(ControlState::Fault));
    motor.runtime.fault =
        static_cast<MotorControlFaultRaw_t>(to_underlying(fault));
    motor.enterSafeState();
    motor.publishDebugState();
}

static void update_encoder_watch_path(MotorController& motor) noexcept
{
    if (++motor.speed.sample_div < SpeedLoopConfig::sample_div)
    {
        return;
    }

    motor.speed.sample_div = 0U;
    if (motor.updateEncoderAngle() != 0U)
    {
        (void)motor.updateEncoderSpeed();
    }
}

/** @brief 初始化 VF 开环状态。 */
void MotorController::vfInit() noexcept
{
    vf.open_loop_timeout_ticks = timeout_ticks_from_ms(OpenLoopConfig::timeout_ms);
    vf.open_loop_ticks = 0U;
    vf.open_loop_theta = 0U;
    vf.open_loop_theta_acc = 0;
    vf.open_loop_reset_count = 0U;
}

/** @brief 仅在明确切入 VF 模式时重置开环角。 */
void MotorController::vfResetForMode(uint8_t mode) noexcept
{
    if (static_cast<ControlMode>(mode) == ControlMode::VfOpenLoop)
    {
        reset_open_loop_theta(*this);
    }
}

/** @brief 运行 VF 应急开环快环，输出 q 轴开环电压并保留基础电流保护。 */
void MotorController::vfRunFastLoop() noexcept
{
    sample_phase_currents(*this);
    if (currentOk() == 0U)
    {
        enter_fault(*this, ControlFault::Current);
        return;
    }

    refresh_timeout(*this);
    if (timed_out(*this))
    {
        enter_fault(*this, ControlFault::OpenLoopTimeout);
        return;
    }

    const uint16_t theta = advance_open_loop_theta(*this);
    applyVoltageVector(0, command.vf.vf_voltage, theta);
    update_encoder_watch_path(*this);
    diag.fast_loop_count++;
}

uint16_t MotorController::vfOpenLoopTheta() const noexcept
{
    return vf.open_loop_theta;
}

uint32_t MotorController::vfOpenLoopTicks() const noexcept
{
    return vf.open_loop_ticks;
}

uint32_t MotorController::vfOpenLoopResetCount() const noexcept
{
    return vf.open_loop_reset_count;
}

} // namespace cms32::motor

extern "C" void MotorControlVf_Init(void)
{
    cms32::motor::g_motor.vfInit();
}

extern "C" void MotorControlVf_ResetForMode(uint8_t mode)
{
    cms32::motor::g_motor.vfResetForMode(mode);
}

extern "C" void MotorControlVf_RunFastLoop(void)
{
    cms32::motor::g_motor.vfRunFastLoop();
}

extern "C" uint16_t MotorControlVf_OpenLoopTheta(void)
{
    return cms32::motor::g_motor.vfOpenLoopTheta();
}

extern "C" uint32_t MotorControlVf_OpenLoopTicks(void)
{
    return cms32::motor::g_motor.vfOpenLoopTicks();
}

extern "C" uint32_t MotorControlVf_OpenLoopResetCount(void)
{
    return cms32::motor::g_motor.vfOpenLoopResetCount();
}
