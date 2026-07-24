#pragma once

#include <stdint.h>

#include "clamp.hpp"
#include "config.hpp"
#include "units.hpp"

namespace cms32::motor
{

struct SpeedMathConfig
{
    static constexpr int32_t counts_per_rev = EncoderConfig::counts_per_rev;
    static constexpr int32_t estimate_hz = SpeedLoopConfig::estimate_hz;
    static constexpr int32_t ramp_rpm_per_s = SpeedLoopConfig::ref_ramp_rpm_per_s;
    static constexpr int32_t ref_limit_rpm = SpeedLoopConfig::ref_limit_rpm;
    static constexpr int16_t command_deadband_rpm =
        SpeedLoopConfig::command_deadband_rpm;
};

static_assert(SpeedMathConfig::counts_per_rev > 0, "invalid speed scale");
static_assert(SpeedMathConfig::estimate_hz > 0, "invalid speed estimate frequency");
static_assert(SpeedMathConfig::ramp_rpm_per_s > 0, "invalid speed ramp");
static_assert(SpeedMathConfig::ref_limit_rpm > 0, "invalid speed limit");
static_assert(SpeedMathConfig::ref_limit_rpm <= 32767, "speed limit does not fit Rpm");

struct SpeedMath
{
    static constexpr int32_t ref_limit_counts() noexcept
    {
        return cms32::support::to_speed<SpeedMathConfig::counts_per_rev>(
                   cms32::support::Rpm{
                       static_cast<int16_t>(SpeedMathConfig::ref_limit_rpm)})
            .value;
    }

    static constexpr cms32::support::Rpm
    to_rpm(cms32::support::SpeedCounts speed) noexcept
    {
        const int32_t rpm =
            (speed.value * 60L) / static_cast<int32_t>(SpeedMathConfig::counts_per_rev);
        return cms32::support::Rpm{
            static_cast<int16_t>(cms32::support::clamp<int32_t>(rpm, -32768, 32767))};
    }

    static constexpr cms32::support::SpeedCounts
    to_speed(cms32::support::Rpm rpm) noexcept
    {
        const int32_t speed =
            (static_cast<int32_t>(rpm.value) * SpeedMathConfig::counts_per_rev) / 60L;
        return cms32::support::SpeedCounts{
            cms32::support::clamp<int32_t>(speed,
                                           -ref_limit_counts(),
                                           ref_limit_counts())};
    }

    static constexpr int32_t ramp_step() noexcept
    {
        const int64_t step =
            (static_cast<int64_t>(SpeedMathConfig::ramp_rpm_per_s) *
             static_cast<int64_t>(SpeedMathConfig::counts_per_rev)) /
            (60LL * static_cast<int64_t>(SpeedMathConfig::estimate_hz));
        return (step <= 0) ? 1 : step;
    }

    static constexpr bool in_deadband(cms32::support::Rpm rpm) noexcept
    {
        return (rpm.value > -SpeedMathConfig::command_deadband_rpm) &&
               (rpm.value < SpeedMathConfig::command_deadband_rpm);
    }

    static constexpr int16_t error_rpm(cms32::support::Rpm ref,
                                       cms32::support::Rpm feedback) noexcept
    {
        return static_cast<int16_t>(cms32::support::clamp<int32_t>(
            static_cast<int32_t>(ref.value) - static_cast<int32_t>(feedback.value),
            -32768,
            32767));
    }
};

} // namespace cms32::motor
