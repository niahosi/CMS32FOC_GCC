#pragma once

#include <stdint.h>

#include "MotorControl.h"

namespace cms32::control {

enum class ControlMode : uint8_t
{
    Off = 0,
    Current = 1,
    Speed = 2,
    VfOpenLoop = 3,
    IfOpenLoop = 4,
};

class Controller
{
public:
    void reset();
    void applyCommand(const volatile MotorControlCommand_t& command);
    void runSlowLoop();
    void updateSpeedEstimate(int32_t position);
    int16_t runSpeedLoopIfDue(int32_t position);
    uint16_t updateOpenLoopTheta();

    uint32_t loopCount() const;
    ControlMode mode() const;
    bool enabled() const;
    int16_t idRef() const;
    int16_t iqRef() const;
    int16_t speedIqRef() const;
    int32_t speedRef() const;
    int32_t speedFeedback() const;
    int16_t iqLimit() const;
    int16_t currentKp() const;
    int16_t currentKi() const;
    int16_t currentVoltageLimit() const;
    int16_t vfVoltage() const;
    int16_t ifIdRef() const;
    int16_t ifIqRef() const;
    uint32_t openLoopTimeoutTicks() const;
    uint32_t openLoopTicks() const;
    void resetOpenLoop();

private:
    static int16_t clampRef(int16_t value, int16_t limit);
    static int16_t absLimit(int16_t value, int16_t limit);
    static int32_t scaleDownS32(int32_t value, uint8_t shift);

    uint32_t loop_count_ = 0;
    bool enabled_ = false;
    ControlMode mode_ = ControlMode::Off;
    int16_t id_ref_ = 0;
    int16_t iq_ref_ = 0;
    int32_t speed_ref_ = 0;
    int16_t speed_iq_ref_ = 0;
    int16_t iq_limit_ = 0;
    int16_t current_kp_ = 0;
    int16_t current_ki_ = 0;
    int16_t current_v_limit_ = 0;
    int32_t open_loop_speed_ref_ = 0;
    int16_t vf_voltage_ = 0;
    int16_t if_id_ref_ = 0;
    int16_t if_iq_ref_ = 0;
    uint32_t open_loop_timeout_ticks_ = 0;
    uint32_t open_loop_ticks_ = 0;
    uint16_t open_loop_theta_ = 0;
    int32_t open_loop_theta_acc_ = 0;

    int32_t speed_fb_raw_ = 0;
    int32_t speed_fb_filt_ = 0;
    int32_t previous_position_ = 0;
    int32_t speed_integral_ = 0;
    uint16_t speed_div_ = 0;
};

} // namespace cms32::control
