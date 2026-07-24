#pragma once

#include <stdint.h>
#include "foc_math.h"
#include "units.hpp"

namespace cms32::motor
{
struct Q15
{
    int16_t value;
};

struct PhaseCurrent
{
    FocPhaseCurrent_t value;
};

struct AlphaBeta
{
    FocAlphaBeta_t value;
};

struct Dq
{
    FocDq_t value;
};

struct Duty
{
    FocDuty_t value;
};

constexpr cms32::support::Angle16 angle16(uint16_t value) noexcept
{
    return cms32::support::Angle16{value};
}

inline Q15 sin_q15(cms32::support::Angle16 angle) noexcept
{
    return Q15{foc_sin_q15(angle.value)};
}

inline Q15 cos_q15(cms32::support::Angle16 angle) noexcept
{
    return Q15{foc_cos_q15(angle.value)};
}

inline AlphaBeta clarke_2phase(int16_t iu, int16_t iv) noexcept
{
    return AlphaBeta{foc_clarke_2phase(iu, iv)};
}

inline AlphaBeta clarke_3phase(PhaseCurrent current) noexcept
{
    return AlphaBeta{foc_clarke_3phase(current.value)};
}

inline Dq park(AlphaBeta input, cms32::support::Angle16 theta) noexcept
{
    return Dq{foc_park(input.value, theta.value)};
}

inline AlphaBeta inv_park(Dq input, cms32::support::Angle16 theta) noexcept
{
    return AlphaBeta{foc_inv_park(input.value, theta.value)};
}

inline bool limit_dq(Dq& voltage, int16_t limit) noexcept
{
    return foc_limit_dq(&voltage.value, limit) != 0U;
}

inline Duty
svpwm(AlphaBeta voltage, uint16_t vdc, uint16_t duty_min, uint16_t duty_max) noexcept
{
    return Duty{foc_svpwm(voltage.value, vdc, duty_min, duty_max)};
}

} // namespace cms32::motor