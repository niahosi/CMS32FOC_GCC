#include "CurrentSense.hpp"

extern "C" {
#include "foc_curr.h"
}

namespace cms32::control {

void CurrentSense::sampleFromBoard()
{
    phase_current_.iu = curr_u();
    phase_current_.iv = curr_v();
    phase_current_.iw = curr_w();
    phase_current_.sum = curr_sum();
}

PhaseCurrentCounts CurrentSense::phaseCurrent() const
{
    return phase_current_;
}

uint32_t CurrentSense::sampleCount() const
{
    return curr_sync_count();
}

} // namespace cms32::control
