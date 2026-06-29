#pragma once

#include <stdint.h>

#include "Controller.hpp"
#include "CurrentSense.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"

namespace cms32::control {

enum class AxisState : uint8_t
{
    Idle = 0,
    SensorCheck = 1,
    Align = 2,
    ClosedLoop = 3,
    Fault = 4,
};

class Axis
{
public:
    void init();
    void applyCommand(const volatile MotorControlCommand_t& command);
    void runSlowLoop();
    void runFastLoop();
    void updateEncoderFast();
    void fillWatch(MotorControlWatch_t& out) const;

    AxisState state() const;
    MotorFault fault() const;
    uint32_t slowLoopCount() const;
    uint32_t fastLoopCount() const;

    Encoder& encoder();
    CurrentSense& currentSense();
    Motor& motor();
    Controller& controller();

private:
    void enterFault(MotorFault fault);
    void forceIdle();
    bool checkCurrentSafe() const;
    void updateChecks();

    AxisState state_ = AxisState::Idle;
    uint32_t slow_loop_count_ = 0;
    uint32_t fast_loop_count_ = 0;
    uint8_t ma600_good_samples_ = 0;
    MotorControlCheck_t check_{};
    MotorFault fault_ = MotorFault::None;
    Encoder encoder_;
    CurrentSense current_sense_;
    Motor motor_;
    Controller controller_;
};

} // namespace cms32::control
