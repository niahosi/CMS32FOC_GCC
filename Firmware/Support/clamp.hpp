#pragma once

#include <type_traits>

namespace cms32::support
{

template <typename T> constexpr T clamp(T value, T min_value, T max_value) noexcept
{
    return (value < min_value) ? min_value : ((value > max_value) ? max_value : value);
}

template <typename T, T MinValue, T MaxValue> constexpr T clamp_static(T value) noexcept
{
    static_assert(MinValue <= MaxValue, "invalid clamp range");
    return clamp<T>(value, MinValue, MaxValue);
}

template <typename T> constexpr T abs_limit(T value, T limit) noexcept
{
    static_assert(std::is_signed<T>::value, "abs_limit requires signed type");
    return clamp<T>(value, static_cast<T>(-limit), limit);
}

template <typename T, T Limit> constexpr T clamp_symmetric(T value) noexcept
{
    static_assert(std::is_signed<T>::value, "clamp_symmetric requires signed type");
    static_assert(Limit >= static_cast<T>(0), "Limit must be non-negative");
    return clamp<T>(value, static_cast<T>(-Limit), Limit);
}

} // namespace cms32::support
