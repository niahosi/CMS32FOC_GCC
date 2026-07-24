#pragma once

#include <stdint.h>

#include "config.hpp"

namespace cms32::board
{
struct AdcTriggerTick
{
    uint16_t value;
};

constexpr bool adc_trigger_inside_pwm_period(AdcTriggerTick tick) noexcept
{
    return (tick.value > cms32::motor::PwmConfig::deadtime_ticks) &&
           (tick.value < cms32::motor::PwmConfig::period);
}

// 默认触发点
constexpr AdcTriggerTick default_adc_trigger_tick() noexcept
{
    return AdcTriggerTick{cms32::motor::PwmConfig::adc_trigger_tick};
}

static_assert(adc_trigger_inside_pwm_period(default_adc_trigger_tick()),
              "default ADC trigger is outside PWM period");

} // namespace cms32::board
