#include "Controller.hpp"

extern "C" {
#include "Config.h"
#include "foc_math.h"
}

namespace cms32::control {

void Controller::reset()
{
    loop_count_ = 0;
    enabled_ = false;
    mode_ = ControlMode::Off;
    id_ref_ = 0;
    iq_ref_ = 0;
    speed_ref_ = 0;
    speed_iq_ref_ = 0;
    iq_limit_ = MOTOR_SPEED_IQ_LIMIT_DEFAULT;
    current_kp_ = MOTOR_CURRENT_KP;
    current_ki_ = MOTOR_CURRENT_KI;
    current_v_limit_ = MOTOR_CURRENT_V_LIMIT;
    open_loop_speed_ref_ = MOTOR_OL_SPEED_REF_DEFAULT;
    vf_voltage_ = MOTOR_VF_VOLTAGE_DEFAULT;
    if_id_ref_ = MOTOR_IF_ID_REF_DEFAULT;
    if_iq_ref_ = MOTOR_IF_IQ_REF_DEFAULT;
    open_loop_timeout_ticks_ = static_cast<uint32_t>(MOTOR_OL_TIMEOUT_MS_DEFAULT) * 2U;
    resetOpenLoop();
    speed_fb_raw_ = 0;
    speed_fb_filt_ = 0;
    previous_position_ = 0;
    speed_integral_ = 0;
    speed_div_ = 0;
}

void Controller::applyCommand(const volatile MotorControlCommand_t& command)
{
    enabled_ = (command.enable != 0U);
    mode_ = (command.control_mode <= static_cast<uint8_t>(ControlMode::IfOpenLoop))
                ? static_cast<ControlMode>(command.control_mode)
                : ControlMode::Off;
    if (!enabled_)
    {
        mode_ = ControlMode::Off;
    }

    id_ref_ = clampRef(command.id_ref, MOTOR_CURRENT_REF_LIMIT);
    iq_ref_ = clampRef(command.iq_ref, MOTOR_CURRENT_REF_LIMIT);
    speed_ref_ = foc_clamp_s32(command.speed_ref, -MOTOR_SPEED_REF_LIMIT, MOTOR_SPEED_REF_LIMIT);
    iq_limit_ = absLimit(command.iq_limit, MOTOR_CURRENT_REF_LIMIT);
    current_kp_ = foc_clamp_s16(command.current_kp, 0, 32767);
    current_ki_ = foc_clamp_s16(command.current_ki, 0, 32767);
    current_v_limit_ = absLimit(command.current_v_limit, PWM_PERIOD / 2);
    open_loop_speed_ref_ =
        foc_clamp_s32(command.open_loop_speed_ref, -MOTOR_SPEED_REF_LIMIT, MOTOR_SPEED_REF_LIMIT);
    vf_voltage_ = clampRef(command.vf_voltage, MOTOR_CURRENT_V_LIMIT);
    if_id_ref_ = clampRef(command.if_id_ref, MOTOR_CURRENT_REF_LIMIT);
    if_iq_ref_ = clampRef(command.if_iq_ref, MOTOR_CURRENT_REF_LIMIT);
    open_loop_timeout_ticks_ = static_cast<uint32_t>(command.open_loop_timeout_ms) * 2U;
}

void Controller::runSlowLoop()
{
    loop_count_++;
}

void Controller::updateSpeedEstimate(int32_t position)
{
    int32_t pos_delta = position - previous_position_;
    int32_t control_delta;
    int32_t raw;
    int32_t diff;

    previous_position_ = position;
    control_delta = pos_delta * static_cast<int32_t>(MOTOR_SENSOR_DIR);
    if ((control_delta <= MOTOR_SPEED_POS_DEADBAND) && (control_delta >= -MOTOR_SPEED_POS_DEADBAND))
    {
        raw = 0;
    }
    else
    {
        raw = control_delta * MOTOR_SPEED_EST_HZ;
    }

    speed_fb_raw_ = raw;
    diff = speed_fb_raw_ - speed_fb_filt_;
    speed_fb_filt_ += scaleDownS32(diff, MOTOR_SPEED_FILTER_SHIFT);
    if ((speed_fb_filt_ <= MOTOR_SPEED_ZERO_SNAP) && (speed_fb_filt_ >= -MOTOR_SPEED_ZERO_SNAP))
    {
        speed_fb_filt_ = 0;
    }
}

int16_t Controller::runSpeedLoopIfDue(int32_t position)
{
    int32_t error;
    int32_t error_scaled;
    int32_t integral_new;
    int32_t output_unclamped;
    int32_t output;

    speed_div_++;
    if (speed_div_ < MOTOR_SPEED_LOOP_DIV)
    {
        return speed_iq_ref_;
    }
    speed_div_ = 0;

    updateSpeedEstimate(position);
    if (mode_ != ControlMode::Speed)
    {
        speed_iq_ref_ = 0;
        speed_integral_ = 0;
        return speed_iq_ref_;
    }

    error = foc_clamp_s32(speed_ref_ - speed_fb_filt_, -MOTOR_SPEED_REF_LIMIT,
                          MOTOR_SPEED_REF_LIMIT);
    error_scaled = scaleDownS32(error, MOTOR_SPEED_ERR_SHIFT);
    error_scaled = foc_clamp_s32(error_scaled, -32767, 32767);
    integral_new = foc_clamp_s32(speed_integral_ + error_scaled, -32767, 32767);
    output_unclamped = static_cast<int32_t>(MOTOR_SPEED_KP) * error_scaled +
                       static_cast<int32_t>(MOTOR_SPEED_KI) * integral_new;
    output = foc_clamp_s32(output_unclamped, -iq_limit_, iq_limit_);

    if ((output == output_unclamped) || (MOTOR_SPEED_KI == 0) ||
        ((output_unclamped > iq_limit_) && (error_scaled < 0)) ||
        ((output_unclamped < -iq_limit_) && (error_scaled > 0)))
    {
        speed_integral_ = integral_new;
    }

    output_unclamped = static_cast<int32_t>(MOTOR_SPEED_KP) * error_scaled +
                       static_cast<int32_t>(MOTOR_SPEED_KI) * speed_integral_;
    speed_iq_ref_ = static_cast<int16_t>(foc_clamp_s32(output_unclamped, -iq_limit_, iq_limit_));
    return speed_iq_ref_;
}

uint16_t Controller::updateOpenLoopTheta()
{
    int32_t step = (open_loop_speed_ref_ * MOTOR_OL_SPEED_TO_THETA_STEP +
                    (1L << (MOTOR_OL_SPEED_TO_THETA_SHIFT - 1U))) >>
                   MOTOR_OL_SPEED_TO_THETA_SHIFT;
    open_loop_theta_acc_ += step;
    open_loop_theta_ = static_cast<uint16_t>(static_cast<uint32_t>(open_loop_theta_acc_) & 0xFFFFU);
    open_loop_ticks_++;
    return static_cast<uint16_t>(static_cast<int32_t>(open_loop_theta_) *
                                 static_cast<int32_t>(MOTOR_SENSOR_DIR));
}

uint32_t Controller::loopCount() const
{
    return loop_count_;
}

ControlMode Controller::mode() const
{
    return mode_;
}

bool Controller::enabled() const
{
    return enabled_;
}

int16_t Controller::idRef() const
{
    return id_ref_;
}

int16_t Controller::iqRef() const
{
    return iq_ref_;
}

int16_t Controller::speedIqRef() const
{
    return speed_iq_ref_;
}

int32_t Controller::speedRef() const
{
    return speed_ref_;
}

int32_t Controller::speedFeedback() const
{
    return speed_fb_filt_;
}

int16_t Controller::iqLimit() const
{
    return iq_limit_;
}

int16_t Controller::currentKp() const
{
    return current_kp_;
}

int16_t Controller::currentKi() const
{
    return current_ki_;
}

int16_t Controller::currentVoltageLimit() const
{
    return current_v_limit_;
}

int16_t Controller::vfVoltage() const
{
    return vf_voltage_;
}

int16_t Controller::ifIdRef() const
{
    return if_id_ref_;
}

int16_t Controller::ifIqRef() const
{
    return if_iq_ref_;
}

uint32_t Controller::openLoopTimeoutTicks() const
{
    return open_loop_timeout_ticks_;
}

uint32_t Controller::openLoopTicks() const
{
    return open_loop_ticks_;
}

void Controller::resetOpenLoop()
{
    open_loop_ticks_ = 0;
    open_loop_theta_ = 0;
    open_loop_theta_acc_ = 0;
}

int16_t Controller::clampRef(int16_t value, int16_t limit)
{
    return foc_clamp_s16(value, static_cast<int16_t>(-limit), limit);
}

int16_t Controller::absLimit(int16_t value, int16_t limit)
{
    if (value < 0)
    {
        value = static_cast<int16_t>(-value);
    }
    return foc_clamp_s16(value, 0, limit);
}

int32_t Controller::scaleDownS32(int32_t value, uint8_t shift)
{
    if (value >= 0)
    {
        return value >> shift;
    }
    return -((-value) >> shift);
}

} // namespace cms32::control
