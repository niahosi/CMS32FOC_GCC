/**
 * @file encoder_math.hpp
 * @author setsuna
 * @brief 纯公式
 * @version 0.1
 * @date 2026-07-20
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <stdint.h>

#include "config.hpp"
#include "units.hpp"

namespace cms32::motor
{
struct EncoderMathConfig
{
    static constexpr int32_t sensor_cpr = EncoderConfig::sensor_cpr;
    static constexpr int32_t counts_per_rev = EncoderConfig::counts_per_rev;
    static constexpr uint32_t current_hz = FastLoopConfig::current_hz;
    static constexpr int32_t speed_estimate_hz = SpeedLoopConfig::estimate_hz;
    static constexpr int32_t direction = EncoderConfig::direction;
    static constexpr uint16_t max_step_raw = EncoderConfig::max_step_raw;
    static constexpr int32_t spike_rpm = SpeedLoopConfig::diff_spike_rpm;
    static constexpr int32_t pos_deadband = SpeedLoopConfig::pos_deadband;
    static constexpr int32_t zero_snap = SpeedLoopConfig::zero_snap;
};

static_assert(EncoderMathConfig::sensor_cpr > 0, "invalid sensor cpr");
static_assert(EncoderMathConfig::counts_per_rev > 0, "invalid encoder scale");
static_assert(EncoderMathConfig::current_hz > 0, "invalid current loop frequency");
static_assert(EncoderMathConfig::speed_estimate_hz > 0, "invalid estimate frequency");
static_assert((EncoderMathConfig::direction == 1) ||
                  (EncoderMathConfig::direction == -1),
              "invalid sensor direction");
static_assert((EncoderMathConfig::pos_deadband >= 0) &&
                  (EncoderMathConfig::pos_deadband <= 32767),
              "invalid speed position deadband");

constexpr int16_t raw_delta(cms32::support::EncoderRaw previous,
                            cms32::support::EncoderRaw current) noexcept
{
    return static_cast<int16_t>(current.value - previous.value);
}

constexpr uint16_t speed_diff_max_delta_raw() noexcept
{
    int32_t limit = (EncoderMathConfig::spike_rpm * EncoderMathConfig::counts_per_rev) /
                    (60L * EncoderMathConfig::speed_estimate_hz);

    if (limit < 1)
    {
        limit = 1;
    }
    if (limit > 32767L)
    {
        limit = 32767L;
    }
    return static_cast<uint16_t>(limit);
}

constexpr uint16_t angle_step_max_delta_raw_per_sample() noexcept
{
    constexpr int64_t denom =
        60LL * static_cast<int64_t>(EncoderMathConfig::current_hz);
    int64_t limit = (static_cast<int64_t>(EncoderMathConfig::spike_rpm) *
                         static_cast<int64_t>(EncoderMathConfig::counts_per_rev) +
                     (denom - 1LL)) /
                    denom;

    if (limit < 1LL)
    {
        limit = 1LL;
    }
    if (limit < static_cast<int64_t>(EncoderMathConfig::max_step_raw))
    {
        limit = EncoderMathConfig::max_step_raw;
    }
    if (limit > 32767LL)
    {
        limit = 32767LL;
    }
    return static_cast<uint16_t>(limit);
}

constexpr uint16_t angle_step_max_delta_raw(uint8_t held_fast_samples) noexcept
{
    const uint32_t samples = static_cast<uint32_t>(held_fast_samples) + 1UL;
    uint32_t limit =
        static_cast<uint32_t>(angle_step_max_delta_raw_per_sample()) * samples;

    if (limit > 32767UL)
    {
        limit = 32767UL;
    }
    return static_cast<uint16_t>(limit);
}

constexpr bool raw_delta_plausible(int16_t delta, uint16_t max_delta) noexcept
{
    return (delta <= static_cast<int16_t>(max_delta)) &&
           (delta >= -static_cast<int16_t>(max_delta));
}

constexpr int16_t deadband_delta(int16_t delta) noexcept
{
    if ((delta > -EncoderMathConfig::pos_deadband) &&
        (delta < EncoderMathConfig::pos_deadband))
    {
        return 0;
    }
    return delta;
}

constexpr cms32::support::SpeedCounts raw_delta_to_speed_counts(int16_t delta) noexcept
{
    const int16_t speed_delta = deadband_delta(delta);
    return cms32::support::SpeedCounts{static_cast<int32_t>(speed_delta) *
                                       EncoderMathConfig::speed_estimate_hz *
                                       EncoderMathConfig::direction};
}

constexpr cms32::support::SpeedCounts
zero_snap_speed(cms32::support::SpeedCounts speed) noexcept
{
    if ((speed.value > -EncoderMathConfig::zero_snap) &&
        (speed.value < EncoderMathConfig::zero_snap))
    {
        return cms32::support::SpeedCounts{0};
    }
    return speed;
}

} // namespace cms32::motor
