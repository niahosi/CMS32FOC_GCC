/**
 * @file motor_control_types.hpp
 * @author setsuna
 * @brief enum class / mode / state / fault helper
 * @version 0.1
 * @date 2026-07-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>

#include "motor_control_internal.h"
namespace cms32::motor
{
enum class ControlState : uint8_t
{
    Idle = MC_STATE_IDLE,
    ClosedLoop = MC_STATE_CLOSED_LOOP,
    Fault = MC_STATE_FAULT,
};

enum class ControlMode : uint8_t
{
    Off = MC_MODE_OFF,
    Current = MC_MODE_CURRENT,
    Speed = MC_MODE_SPEED,
    VfOpenLoop = MC_MODE_VF_OPEN_LOOP,
    AlignLock = MC_MODE_ALIGN_LOCK,
    EncoderVoltage = MC_MODE_ENCODER_VOLTAGE,
};

enum class ControlFault : uint8_t
{
    None = MC_FAULT_NONE,
    UnsupportedMode = MC_FAULT_UNSUPPORTED_MODE,
    Current = MC_FAULT_CURRENT,
    OpenLoopTimeout = MC_FAULT_OPEN_LOOP_TIMEOUT,
    Encoder = MC_FAULT_ENCODER,
};

constexpr ControlMode to_control_mode(uint8_t value) noexcept
{
    return static_cast<ControlMode>(value);
}

constexpr bool is_closed_loop_mode(ControlMode mode) noexcept
{
    return (mode == ControlMode::Current) || (mode == ControlMode::Speed);
}

constexpr bool is_supported_run_mode(ControlMode mode) noexcept
{
    return (mode == ControlMode::VfOpenLoop) || (is_closed_loop_mode(mode));
}
} // namespace cms32::motor