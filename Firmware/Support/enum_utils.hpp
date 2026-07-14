#pragma once

#include <type_traits>

namespace cms32::support
{

template <typename Enum>
constexpr auto to_underlying(Enum value) noexcept -> typename std::underlying_type<Enum>::type
{
    static_assert(std::is_enum<Enum>::value, "to_underlying requires enum");
    return static_cast<typename std::underlying_type<Enum>::type>(value);
}

} // namespace cms32::support
