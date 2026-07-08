#pragma once

#include <stdint.h>

namespace cms32::control {

struct PhaseCurrentCounts
{
    int16_t iu;
    int16_t iv;
    int16_t iw;
    int16_t sum;
};

class CurrentSense
{
public:
    void sampleFromBoard();
    PhaseCurrentCounts phaseCurrent() const;
    uint32_t sampleCount() const;

private:
    PhaseCurrentCounts phase_current_{};
};

} // namespace cms32::control
