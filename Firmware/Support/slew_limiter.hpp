#pragma once

#include <type_traits>

namespace cms32::support
{

template <typename T> constexpr T slew_step(T current, T target, T step) noexcept
{
    static_assert(std::is_signed<T>::value, "slew_step requires signed type");

    if (step <= static_cast<T>(0))
    {
        return target;
    }

    const T delta = static_cast<T>(target - current);
    if (delta > step)
    {
        return static_cast<T>(current + step);
    }
    if (delta < static_cast<T>(-step))
    {
        return static_cast<T>(current - step);
    }
    return target;
}

template <typename T, T Step> class SlewLimiter
{
public:
    static_assert(std::is_signed<T>::value, "SlewLimiter requires signed type");
    static_assert(Step > static_cast<T>(0), "Step must be positive");

    constexpr T update(T target) noexcept
    {
        value = slew_step<T>(value, target, Step);
        return value;
    }

    constexpr T reset(T next_value = static_cast<T>(0)) noexcept
    {
        value = next_value;
        return value;
    }

    constexpr T get() const noexcept
    {
        return value;
    }

    T value{0};
};

} // namespace cms32::support
