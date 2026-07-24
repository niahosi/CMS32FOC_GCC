/**
 * @file pwm_values.hpp
 * @author setsuna
 * @brief 只表达duty数值规则
 * @version 0.1
 * @date 2026-07-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>
#include "config.hpp"
#include "clamp.hpp"
namespace cms32::board
{
struct PwmDuty
{
    uint16_t value;
};

struct ThreePhaseDuty
{
    PwmDuty u;
    PwmDuty v;
    PwmDuty w;
};

constexpr PwmDuty clamp_pwm_duty(uint16_t duty) noexcept
{
    return PwmDuty{static_cast<uint16_t>(
        cms32::support::clamp<uint16_t>(duty,
                                        cms32::motor::PwmConfig::duty_min,
                                        cms32::motor::PwmConfig::duty_max))};
}

constexpr ThreePhaseDuty safe_center_duty() noexcept
{
    return ThreePhaseDuty{
        PwmDuty{cms32::motor::PwmConfig::duty_center},
        PwmDuty{cms32::motor::PwmConfig::duty_center},
        PwmDuty{cms32::motor::PwmConfig::duty_center},
    };
}

} // namespace cms32::board
