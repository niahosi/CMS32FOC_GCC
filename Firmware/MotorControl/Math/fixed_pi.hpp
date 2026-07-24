#pragma once

#include <stdint.h>
#include "foc_math.h"

namespace cms32::motor
{
struct FixedPiConfig
{
    int16_t kp;
    int16_t ki;
    int16_t output_min;
    int16_t output_max;
    uint8_t shift;
};

class FixedPiRef
{
public:
    explicit FixedPiRef(FocPi_t& pi) noexcept
        : pi_(pi)
    {
    }

    void init(FixedPiConfig config) noexcept
    {
        foc_pi_init(&pi_,
                    config.kp,
                    config.ki,
                    config.output_min,
                    config.output_max,
                    config.shift);
    }

    void reset() noexcept
    {
        foc_pi_reset(&pi_);
    }

    void set_gains(FixedPiConfig config) noexcept
    {
        foc_pi_set_gains(&pi_,
                         config.kp,
                         config.ki,
                         config.output_min,
                         config.output_max,
                         config.shift);
    }

    int16_t update(int16_t ref, int16_t feedback) noexcept
    {
        return foc_pi_update(&pi_, ref, feedback);
    }

    FocPi_t& raw() noexcept
    {
        return pi_;
    }

    const FocPi_t& raw() const noexcept
    {
        return pi_;
    }

private:
    FocPi_t& pi_;
};

class FixedPi
{
public:
    void init(FixedPiConfig config) noexcept
    {
        foc_pi_init(&pi_,
                    config.kp,
                    config.ki,
                    config.output_min,
                    config.output_max,
                    config.shift);
    }

    void reset() noexcept
    {
        foc_pi_reset(&pi_);
    }

    void set_gains(FixedPiConfig config) noexcept
    {
        foc_pi_set_gains(&pi_,
                         config.kp,
                         config.ki,
                         config.output_min,
                         config.output_max,
                         config.shift);
    }

    int16_t update(int16_t ref, int16_t feedback) noexcept
    {
        return foc_pi_update(&pi_, ref, feedback);
    }

    FocPi_t& raw() noexcept
    {
        return pi_;
    }

    const FocPi_t& raw() const noexcept
    {
        return pi_;
    }

private:
    FocPi_t pi_{};
};
} // namespace cms32::motor
