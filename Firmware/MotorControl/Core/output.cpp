/**
 * @file output.cpp
 * @author setsuna
 * @brief MotorControl output, safe state, and voltage vector path
 * @version 0.1
 * @date 2026-07-23
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "motor_controller.hpp"

#include "fixed_pi.hpp"
#include "foc_math.h"

#include "board_profile.h"
#include "foc_pwm.h"

#include <stdint.h>

namespace
{

using cms32::motor::CurrentCheckConfig;
using cms32::motor::CurrentLoopConfig;
using cms32::motor::PwmConfig;

constexpr bool count_in_limit(int16_t value, int16_t limit) noexcept
{
    return (value >= -limit) && (value <= limit);
}

uint8_t phase_currents_ok(const FocPhaseCurrent_t& current) noexcept
{
    if constexpr (CurrentCheckConfig::phase_limit < 32767)
    {
        return static_cast<uint8_t>(
            count_in_limit(current.u, CurrentCheckConfig::phase_limit) &&
            count_in_limit(current.v, CurrentCheckConfig::phase_limit) &&
            count_in_limit(current.w, CurrentCheckConfig::phase_limit));
    }

    return 1U;
}

uint8_t current_sum_ok(const FocPhaseCurrent_t& current) noexcept
{
    if constexpr (CurrentCheckConfig::sum_limit < 32767)
    {
        const int16_t sum = static_cast<int16_t>(current.u + current.v + current.w);
        return static_cast<uint8_t>(count_in_limit(sum, CurrentCheckConfig::sum_limit));
    }

    return 1U;
}

} // namespace

namespace cms32::motor
{

/** @brief 检查当前状态中的三相电流和 KCL 和是否在安全范围。 */
uint8_t MotorController::currentOk() const noexcept
{
    return static_cast<uint8_t>(phase_currents_ok(current.phase) &&
                                current_sum_ok(current.phase));
}

/** @brief 获取当前电流环/诊断电压限幅，命令未设置时回退默认值。 */
int16_t MotorController::voltageLimit() const noexcept
{
    const int16_t command_limit = command.current.current_v_limit;
    return (command_limit > 0) ? command_limit : CurrentLoopConfig::voltage_limit;
}

static void clear_output_state(MotorController& motor) noexcept
{
    motor.runtime.pwm_output = 0U;
    motor.output.voltage_dq = FocDq_t{0, 0};
    motor.output.voltage_ab = FocAlphaBeta_t{0, 0};
    motor.output.voltage_theta = 0U;
    motor.output.duty = FocDuty_t{PwmConfig::duty_center,
                                  PwmConfig::duty_center,
                                  PwmConfig::duty_center};
}

static void clear_speed_output_state(MotorController& motor) noexcept
{
    motor.speed.iq_ref.reset();
    motor.speed.err_rpm = 0;
    foc_pi_reset(&motor.speed.pi);
}

/** @brief 关闭 PWM 并清空输出/速度 PI 状态。 */
void MotorController::enterSafeState() noexcept
{
    diag.safe_state_count++;
    pwm_off();
    clear_output_state(*this);
    clear_speed_output_state(*this);
}

/** @brief 统一输出 dq 电压矢量，含限幅、反 Park、SVPWM 和 PWM 使能。 */
void MotorController::applyVoltageVector(int16_t vd,
                                         int16_t vq,
                                         uint16_t theta) noexcept
{
    const int16_t v_limit = voltageLimit();

    output.voltage_theta = theta;
    output.voltage_dq.d = vd;
    output.voltage_dq.q = vq;
    output.voltage_limited = foc_limit_dq(&output.voltage_dq, v_limit);
    output.voltage_ab = foc_inv_park(output.voltage_dq, theta);
    output.duty = foc_svpwm(output.voltage_ab,
                            PwmConfig::period,
                            PwmConfig::duty_min,
                            PwmConfig::duty_max);
    BoardProfile_FocMathEnd();

    BoardProfile_PwmUpdateBegin();
    pwm_set_duty(output.duty.u, output.duty.v, output.duty.w);
    (void)pwm_enable(1U);
    BoardProfile_PwmUpdateEnd();
    runtime.pwm_output = 1U;
}

} // namespace cms32::motor

extern "C" uint8_t MotorControl_InternalCurrentOk(void)
{
    return cms32::motor::g_motor.currentOk();
}

extern "C" int16_t MotorControl_InternalVoltageLimit(void)
{
    return cms32::motor::g_motor.voltageLimit();
}

extern "C" void MotorControl_InternalEnterSafeState(void)
{
    cms32::motor::g_motor.enterSafeState();
}

extern "C" void MotorControl_InternalApplyVoltageVector(int16_t vd,
                                                        int16_t vq,
                                                        uint16_t theta)
{
    cms32::motor::g_motor.applyVoltageVector(vd, vq, theta);
}
