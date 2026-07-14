#pragma once

#include <stdint.h>

namespace cms32::support
{

template <uint8_t Shift> class LowPassI32
{
public:
    static_assert(Shift < 31U, "Shift is too large");

    constexpr int32_t update(int32_t sample) noexcept
    {
        if constexpr (Shift == 0U)
        {
            value_ = sample;
        }
        else
        {
            value_ += (sample - value_) >> Shift;
        }
        return value_;
    }

    constexpr void reset(int32_t value = 0) noexcept
    {
        value_ = value;
    }

    constexpr int32_t value() const noexcept
    {
        return value_;
    }

private:
    int32_t value_{0};
};

} // namespace cms32::support
