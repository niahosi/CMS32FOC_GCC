#include "Axis.hpp"
#include "Config.h"

extern "C" {
#include "foc_pwm.h"
}

namespace cms32::control {

void Axis::init()
{
    controller_.reset();
    encoder_.reset();
    motor_.init();
    current_sense_.sampleFromBoard();
    check_ = {};
    ma600_good_samples_ = 0U;
    fault_ = MotorFault::None;
    state_ = AxisState::Idle;
    motor_.enterSafeState();
}

void Axis::applyCommand(const volatile MotorControlCommand_t& command)
{
    command_apply_count_++;
    command_enable_ = command.enable;
    command_control_mode_ = command.control_mode;
    command_vf_voltage_ = command.vf_voltage;
    command_open_loop_speed_ref_ = command.open_loop_speed_ref;

    controller_.applyCommand(command);
    encoder_.setZeroTrim(command.elec_zero_trim);
    motor_.configureCurrentPi(controller_);
    if (!controller_.enabled())
    {
        forceIdle();
    }
}

void Axis::runSlowLoop()
{
    const bool open_loop_mode = (controller_.mode() == ControlMode::VfOpenLoop) ||
                                (controller_.mode() == ControlMode::IfOpenLoop);

    if (controller_.enabled())
    {
        if (!open_loop_mode)
        {
            encoder_.updateFromBoard();
        }
        current_sense_.sampleFromBoard();
    }
    updateChecks();
    controller_.runSlowLoop();

    if (!controller_.enabled())
    {
        forceIdle();
        slow_loop_count_++;
        return;
    }

    switch (state_)
    {
        case AxisState::Idle:
            motor_.enterSafeState();
            motor_.resetControl();
            encoder_.reset();
            ma600_good_samples_ = 0U;
            state_ = AxisState::SensorCheck;
            break;

        case AxisState::SensorCheck:
            if (controller_.mode() == ControlMode::Off)
            {
                motor_.enterSafeState();
                motor_.resetControl();
            }
            else if (open_loop_mode && (check_.current_ok != 0U) &&
                     (check_.pwm_off_safe != 0U))
            {
                motor_.resetControl();
                controller_.resetOpenLoop();
                state_ = AxisState::ClosedLoop;
            }
            else if (check_.ready_closed_loop != 0U)
            {
                motor_.resetControl();
                controller_.resetOpenLoop();
                state_ = AxisState::Align;
            }
            break;

        case AxisState::Align:
            if (check_.ma600_ok == 0U)
            {
                enterFault(MotorFault::Ma600Check);
            }
            else if (check_.current_ok == 0U)
            {
                enterFault(MotorFault::CurrentCheck);
            }
            break;

        case AxisState::ClosedLoop:
            if (!open_loop_mode && (check_.ma600_ok == 0U))
            {
                enterFault(MotorFault::Ma600Check);
            }
            else if (check_.current_ok == 0U)
            {
                enterFault(MotorFault::CurrentCheck);
            }
            break;

        case AxisState::Fault:
        default:
            motor_.enterSafeState();
            break;
    }
    slow_loop_count_++;
}

void Axis::runFastLoop()
{
    current_sense_.sampleFromBoard();
    if (!controller_.enabled())
    {
        return;
    }

    if (state_ == AxisState::Align)
    {
        if (motor_.runAlign(current_sense_))
        {
            if (motor_.fault() != MotorFault::None)
            {
                enterFault(motor_.fault());
            }
            else
            {
                state_ = AxisState::ClosedLoop;
            }
        }
    }
    else if (state_ == AxisState::ClosedLoop)
    {
        MotorFault fault;

        (void)controller_.runSpeedLoopIfDue(encoder_.position());
        fault = motor_.runControl(controller_.mode(), controller_, encoder_, current_sense_,
                                  controller_.mode() != ControlMode::Off);
        if (fault != MotorFault::None)
        {
            if (fault == MotorFault::OpenLoopTimeout)
            {
                motor_.enterSafeState();
            }
            else
            {
                enterFault(fault);
            }
        }
    }
    fast_loop_count_++;
}

void Axis::updateEncoderFast()
{
    const bool open_loop_mode = (controller_.mode() == ControlMode::VfOpenLoop) ||
                                (controller_.mode() == ControlMode::IfOpenLoop);

    if (open_loop_mode)
    {
        return;
    }

    encoder_.updateFastFromBoard();
}

void Axis::fillWatch(MotorControlWatch_t& out) const
{
    const auto phase_current = current_sense_.phaseCurrent();
    const auto& snap = motor_.snapshot();

    out.state = static_cast<uint8_t>(state_);
    out.control_mode = static_cast<uint8_t>(controller_.mode());
    out.fault_reason = static_cast<uint8_t>(fault_);
    out.enable = controller_.enabled() ? 1U : 0U;
    out.slow_loop_count = slow_loop_count_;
    out.fast_loop_count = fast_loop_count_;
    out.adc_sample_count = current_sense_.sampleCount();
    out.encoder_raw = encoder_.raw();
    out.encoder_elec = encoder_.electrical();
    out.encoder_delta = encoder_.delta();
    out.encoder_pos = encoder_.position();
    out.encoder_age = encoder_.age();
    out.encoder_ok = encoder_.ok() ? 1U : 0U;
    out.iu_cnt = phase_current.iu;
    out.iv_cnt = phase_current.iv;
    out.iw_cnt = phase_current.iw;
    out.i_sum = phase_current.sum;
    out.id_ref = snap.current_ref.d;
    out.iq_ref = snap.current_ref.q;
    out.speed_ref = controller_.speedRef();
    out.speed_fb = controller_.speedFeedback();
    out.id = snap.current_dq.d;
    out.iq = snap.current_dq.q;
    out.vd = snap.voltage_dq.d;
    out.vq = snap.voltage_dq.q;
    out.v_limited = snap.voltage_limited;
    out.duty_u = snap.duty.u;
    out.duty_v = snap.duty.v;
    out.duty_w = snap.duty.w;
    out.pwm_safe = motor_.pwmSafe() ? 1U : 0U;
    out.pwm_running = motor_.pwmRunning() ? 1U : 0U;
    out.check = check_;
    out.command_apply_count = command_apply_count_;
    out.command_enable = command_enable_;
    out.command_control_mode = command_control_mode_;
    out.command_vf_voltage = command_vf_voltage_;
    out.command_open_loop_speed_ref = command_open_loop_speed_ref_;
}

AxisState Axis::state() const
{
    return state_;
}

MotorFault Axis::fault() const
{
    return fault_;
}

uint32_t Axis::slowLoopCount() const
{
    return slow_loop_count_;
}

uint32_t Axis::fastLoopCount() const
{
    return fast_loop_count_;
}

Encoder& Axis::encoder()
{
    return encoder_;
}

CurrentSense& Axis::currentSense()
{
    return current_sense_;
}

Motor& Axis::motor()
{
    return motor_;
}

Controller& Axis::controller()
{
    return controller_;
}

void Axis::enterFault(MotorFault fault)
{
    if (fault_ == MotorFault::None)
    {
        fault_ = fault;
    }
    state_ = AxisState::Fault;
    motor_.enterSafeState();
}

void Axis::forceIdle()
{
    state_ = AxisState::Idle;
    fault_ = MotorFault::None;
    ma600_good_samples_ = 0U;
    motor_.enterSafeState();
    motor_.resetControl();
}

bool Axis::checkCurrentSafe() const
{
    bool u_ok = true;
    bool v_ok = true;
    bool w_ok = true;
    bool sum_ok = true;

#if ((MOT_CHECK_CURR_CNT_LIMIT < 32767) || (MOT_CHECK_SUM_CNT_LIMIT < 32767))
    const auto current = current_sense_.phaseCurrent();
#endif

#if (MOT_CHECK_CURR_CNT_LIMIT < 32767)
    u_ok = (current.iu >= -MOT_CHECK_CURR_CNT_LIMIT) &&
           (current.iu <= MOT_CHECK_CURR_CNT_LIMIT);
    v_ok = (current.iv >= -MOT_CHECK_CURR_CNT_LIMIT) &&
           (current.iv <= MOT_CHECK_CURR_CNT_LIMIT);
    w_ok = (current.iw >= -MOT_CHECK_CURR_CNT_LIMIT) &&
           (current.iw <= MOT_CHECK_CURR_CNT_LIMIT);
#endif

#if (MOT_CHECK_SUM_CNT_LIMIT < 32767)
    sum_ok = (current.sum >= -MOT_CHECK_SUM_CNT_LIMIT) &&
             (current.sum <= MOT_CHECK_SUM_CNT_LIMIT);
#endif

    return u_ok && v_ok && w_ok && sum_ok;
}

void Axis::updateChecks()
{
    if (encoder_.safe())
    {
        if (ma600_good_samples_ < MOT_CHECK_MA600_SAMPLES)
        {
            ma600_good_samples_++;
        }
    }
    else
    {
        ma600_good_samples_ = 0U;
    }

    check_.ma600_ok = (ma600_good_samples_ >= MOT_CHECK_MA600_SAMPLES) ? 1U : 0U;
    check_.current_ok = checkCurrentSafe() ? 1U : 0U;
    check_.pwm_off_safe = (pwm_is_off_safe() != 0U) ? 1U : 0U;
    check_.ready_closed_loop =
        (check_.ma600_ok != 0U && check_.current_ok != 0U && check_.pwm_off_safe != 0U) ? 1U
                                                                                         : 0U;
}

} // namespace cms32::control
