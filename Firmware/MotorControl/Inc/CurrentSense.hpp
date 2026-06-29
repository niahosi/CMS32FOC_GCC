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
    uint8_t samplePair() const;
    uint8_t threeShuntActive() const;
    uint8_t sampleHold() const;
    uint16_t sampleHoldCount() const;
    uint16_t samplePairHoldLeft() const;
    uint16_t commonWindow() const;
    uint32_t sampleSwitchCount() const;
    uint32_t sampleFallbackCount() const;
    uint32_t ivSpikeCount() const;
    uint32_t iwSpikeCount() const;
    uint16_t ivMaxStep() const;
    uint16_t iwMaxStep() const;

private:
    PhaseCurrentCounts phase_current_{};
    uint32_t sample_count_ = 0;
};

} // namespace cms32::control
