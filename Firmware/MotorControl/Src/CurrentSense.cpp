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
    sample_count_++;
}

PhaseCurrentCounts CurrentSense::phaseCurrent() const
{
    return phase_current_;
}

uint32_t CurrentSense::sampleCount() const
{
    return sample_count_;
}

uint8_t CurrentSense::samplePair() const
{
    return curr_pair();
}

uint8_t CurrentSense::threeShuntActive() const
{
    return curr_three_shunt_active();
}

uint8_t CurrentSense::sampleHold() const
{
    return curr_is_hold();
}

uint16_t CurrentSense::sampleHoldCount() const
{
    return curr_hold_count();
}

uint16_t CurrentSense::samplePairHoldLeft() const
{
    return curr_sample_pair_hold_left();
}

uint16_t CurrentSense::commonWindow() const
{
    return curr_window_common();
}

uint32_t CurrentSense::sampleSwitchCount() const
{
    return curr_sample_switch_count();
}

uint32_t CurrentSense::sampleFallbackCount() const
{
    return curr_sample_fallback_count();
}

uint32_t CurrentSense::ivSpikeCount() const
{
    return curr_iv_spike_count();
}

uint32_t CurrentSense::iwSpikeCount() const
{
    return curr_iw_spike_count();
}

uint16_t CurrentSense::ivMaxStep() const
{
    return curr_iv_max_step();
}

uint16_t CurrentSense::iwMaxStep() const
{
    return curr_iw_max_step();
}

} // namespace cms32::control
