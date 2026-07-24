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
            value = sample;
        }
        else
        {
            value += (sample - value) >> Shift;
        }
        return value;
    }

    constexpr void reset(int32_t next_value = 0) noexcept
    {
        value = next_value;
    }

    constexpr int32_t get() const noexcept
    {
        return value;
    }

    int32_t value{0};
};

} // namespace cms32::support
