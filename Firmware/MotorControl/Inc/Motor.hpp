#pragma once

#include <stdint.h>

#include "Controller.hpp"
#include "CurrentSense.hpp"
#include "Encoder.hpp"

extern "C" {
#include "foc_math.h"
}

namespace cms32::control {

enum class MotorFault : uint8_t
{
    None = 0,
    Ma600Check = 1,
    CurrentCheck = 2,
    AngleStale = 3,
    RunCurrent = 4,
    OpenLoopTimeout = 5,
    State = 6,
};

typedef struct
{
    uint8_t active;
    FocPhaseCurrent_t current;
    FocAlphaBeta_t current_ab;
    FocDq_t current_dq;
    FocDq_t current_ref;
    FocDq_t voltage_dq;
    FocAlphaBeta_t voltage_ab;
    FocDuty_t duty;
    uint8_t voltage_limited;
    uint8_t current_over_count;
} MotorSnapshot;

class Motor
{
public:
    void init();
    void enterSafeState();
    void resetControl();
    void configureCurrentPi(const Controller& controller);
    bool runAlign(const CurrentSense& current_sense);
    MotorFault runControl(ControlMode mode, Controller& controller, const Encoder& encoder,
                          const CurrentSense& current_sense, bool output_ready);

    uint32_t currentLoopCount() const;
    bool pwmSafe() const;
    bool pwmRunning() const;
    MotorFault fault() const;
    const MotorSnapshot& snapshot() const;

private:
    static bool isAbsOver(int16_t value, int16_t limit);
    bool isCurrentSafe(FocPhaseCurrent_t current);
    void outputVoltage(FocDq_t voltage, uint16_t theta, bool output_ready);

    uint32_t current_loop_count_ = 0;
    uint16_t align_ticks_ = 0;
    bool pwm_safe_ = true;
    bool pwm_running_ = false;
    MotorFault fault_ = MotorFault::None;
    FocPi_t pi_id_{};
    FocPi_t pi_iq_{};
    MotorSnapshot snapshot_{};
};

} // namespace cms32::control
