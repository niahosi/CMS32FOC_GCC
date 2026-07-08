#include "Motor.hpp"
#include "Config.h"

extern "C" {
#include "foc_pwm.h"
}

namespace cms32::control {

void Motor::init()
{
    foc_pi_init(&pi_id_, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);
    foc_pi_init(&pi_iq_, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);
    enterSafeState();
    fault_ = MotorFault::None;
}

void Motor::enterSafeState()
{
    pwm_off();
    pwm_safe_ = (pwm_is_off_safe() != 0U);
    pwm_running_ = (pwm_is_running() != 0U);
    snapshot_.active = 0U;
}

void Motor::resetControl()
{
    current_loop_count_ = 0;
    align_ticks_ = 0;
    snapshot_ = {};
    fault_ = MotorFault::None;
    foc_pi_reset(&pi_id_);
    foc_pi_reset(&pi_iq_);
}

void Motor::configureCurrentPi(const Controller& controller)
{
    const int16_t limit = controller.currentVoltageLimit();
    foc_pi_set_gains(&pi_id_, controller.currentKp(), controller.currentKi(),
                     static_cast<int16_t>(-limit), limit, CTRL_CUR_PI_SHIFT);
    foc_pi_set_gains(&pi_iq_, controller.currentKp(), controller.currentKi(),
                     static_cast<int16_t>(-limit), limit, CTRL_CUR_PI_SHIFT);
}

bool Motor::runAlign(const CurrentSense& current_sense)
{
    PhaseCurrentCounts phase = current_sense.phaseCurrent();
    FocDq_t voltage = {.d = MOT_ALIGN_VD, .q = 0};

    snapshot_.current = {.u = phase.iu, .v = phase.iv, .w = phase.iw};
    if (!isCurrentSafe(snapshot_.current))
    {
        fault_ = MotorFault::RunCurrent;
        enterSafeState();
        return true;
    }

    align_ticks_++;
    if (align_ticks_ >= MOT_ALIGN_TICKS)
    {
        enterSafeState();
        return true;
    }

    (void)foc_limit_dq(&voltage, CTRL_CUR_V_LIMIT);
    outputVoltage(voltage, MOT_ALIGN_THETA, true);
    return false;
}

MotorFault Motor::runControl(ControlMode mode, Controller& controller, const Encoder& encoder,
                             const CurrentSense& current_sense, bool output_ready)
{
    PhaseCurrentCounts phase = current_sense.phaseCurrent();
    uint16_t theta = encoder.electrical();
    FocDq_t voltage;

    snapshot_.current = {.u = phase.iu, .v = phase.iv, .w = phase.iw};
    snapshot_.current_ab = foc_clarke_3phase(snapshot_.current);

    if ((mode != ControlMode::VfOpenLoop) && (mode != ControlMode::IfOpenLoop) && !encoder.safe())
    {
        fault_ = MotorFault::AngleStale;
        enterSafeState();
        return fault_;
    }

    if (!isCurrentSafe(snapshot_.current))
    {
        fault_ = MotorFault::RunCurrent;
        enterSafeState();
        return fault_;
    }

    snapshot_.active = 1U;
    current_loop_count_++;

    if (mode == ControlMode::VfOpenLoop)
    {
        if ((controller.openLoopTimeoutTicks() > 0U) &&
            (controller.openLoopTicks() >= controller.openLoopTimeoutTicks()))
        {
            fault_ = MotorFault::OpenLoopTimeout;
            enterSafeState();
            return fault_;
        }
        theta = controller.updateOpenLoopTheta();
        voltage = {.d = controller.vfVoltage(), .q = 0};
        snapshot_.current_ref = {.d = 0, .q = 0};
        outputVoltage(voltage, theta, output_ready);
        fault_ = MotorFault::None;
        return fault_;
    }

    snapshot_.current_dq = foc_park(snapshot_.current_ab, theta);
    if (mode == ControlMode::IfOpenLoop)
    {
        if ((controller.openLoopTimeoutTicks() > 0U) &&
            (controller.openLoopTicks() >= controller.openLoopTimeoutTicks()))
        {
            fault_ = MotorFault::OpenLoopTimeout;
            enterSafeState();
            return fault_;
        }
        theta = controller.updateOpenLoopTheta();
        snapshot_.current_ref = {.d = controller.ifIdRef(), .q = controller.ifIqRef()};
    }
    else if (mode == ControlMode::Speed)
    {
        snapshot_.current_ref = {.d = 0, .q = controller.speedIqRef()};
    }
    else if (mode == ControlMode::Current)
    {
        snapshot_.current_ref = {.d = controller.idRef(), .q = controller.iqRef()};
    }
    else
    {
        snapshot_.current_ref = {.d = 0, .q = 0};
        outputVoltage(snapshot_.current_ref, theta, false);
        fault_ = MotorFault::None;
        return fault_;
    }

    voltage.d = foc_pi_update(&pi_id_, snapshot_.current_ref.d, snapshot_.current_dq.d);
    voltage.q = foc_pi_update(&pi_iq_, snapshot_.current_ref.q, snapshot_.current_dq.q);
    snapshot_.voltage_limited = foc_limit_dq(&voltage, controller.currentVoltageLimit());
    outputVoltage(voltage, theta, output_ready);
    fault_ = MotorFault::None;
    return fault_;
}

uint32_t Motor::currentLoopCount() const
{
    return current_loop_count_;
}

bool Motor::pwmSafe() const
{
    return pwm_safe_;
}

bool Motor::pwmRunning() const
{
    return pwm_running_;
}

MotorFault Motor::fault() const
{
    return fault_;
}

const MotorSnapshot& Motor::snapshot() const
{
    return snapshot_;
}

bool Motor::isAbsOver(int16_t value, int16_t limit)
{
    return (value > limit) || (value < -limit);
}

bool Motor::isCurrentSafe(FocPhaseCurrent_t current)
{
    if (!isAbsOver(current.u, CTRL_CUR_SAFE_LIMIT) &&
        !isAbsOver(current.v, CTRL_CUR_SAFE_LIMIT) &&
        !isAbsOver(current.w, CTRL_CUR_SAFE_LIMIT))
    {
        snapshot_.current_over_count = 0U;
        return true;
    }

    if (snapshot_.current_over_count < 255U)
    {
        snapshot_.current_over_count++;
    }
    return snapshot_.current_over_count < CTRL_CUR_OVER_LIMIT;
}

void Motor::outputVoltage(FocDq_t voltage, uint16_t theta, bool output_ready)
{
    snapshot_.voltage_dq = voltage;
    snapshot_.voltage_ab = foc_inv_park(voltage, theta);
    snapshot_.duty =
        foc_svpwm(snapshot_.voltage_ab, PWM_PERIOD, PWM_DUTY_MIN, PWM_DUTY_MAX);

    if (output_ready)
    {
        pwm_set_duty(snapshot_.duty.u, snapshot_.duty.v, snapshot_.duty.w);
        (void)pwm_enable(1U);
    }
    else
    {
        pwm_off();
    }

    pwm_safe_ = (pwm_is_off_safe() != 0U);
    pwm_running_ = (pwm_is_running() != 0U);
}

} // namespace cms32::control
