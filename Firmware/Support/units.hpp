#pragma once

#include <stdint.h>

namespace cms32::support
{

struct Rpm
{
    int16_t value;
};

struct Milliseconds
{
    uint16_t value;
};

struct CurrentCount
{
    int16_t value;
};

struct VoltageCount
{
    int16_t value;
};

struct EncoderRaw
{
    uint16_t value;
};

struct EncoderPosition
{
    int32_t value;
};

struct SpeedCounts
{
    int32_t value;
};

struct Angle16
{
    uint16_t value;
};

template <int32_t CountsPerRev> constexpr SpeedCounts to_speed(Rpm rpm) noexcept
{
    static_assert(CountsPerRev > 0, "CountsPerRev must be positive");
    return SpeedCounts{
        static_cast<int32_t>((static_cast<int32_t>(rpm.value) * CountsPerRev) / 60L)};
}

template <int32_t CountsPerRev> constexpr Rpm to_rpm(SpeedCounts speed) noexcept
{
    static_assert(CountsPerRev > 0, "CountsPerRev must be positive");
    return Rpm{static_cast<int16_t>((speed.value * 60L) / CountsPerRev)};
}

constexpr Angle16 add_angle(Angle16 angle, int16_t delta) noexcept
{
    return Angle16{static_cast<uint16_t>(angle.value + delta)};
}

} // namespace cms32::support
