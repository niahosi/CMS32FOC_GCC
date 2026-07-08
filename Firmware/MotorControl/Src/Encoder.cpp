#include "Encoder.hpp"
#include "Config.h"

extern "C" {
#include "foc_bsp.h"
}

namespace cms32::control {

void Encoder::reset()
{
    raw_ = 0;
    electrical_ = 0;
    previous_raw_ = 0;
    delta_ = 0;
    position_ = 0;
    age_ = 255U;
    initialized_ = false;
    ok_ = false;
}

void Encoder::updateFromBoard()
{
    consumeBoardCache(bsp_update_angle(), bsp_angle_raw());
}

void Encoder::updateFastFromBoard()
{
    consumeBoardCache(bsp_update_angle_fast(), bsp_angle_raw());
}

void Encoder::setZeroTrim(int16_t trim)
{
    zero_trim_ = trim;
}

uint16_t Encoder::raw() const
{
    return raw_;
}

uint16_t Encoder::electrical() const
{
    return electrical_;
}

int16_t Encoder::delta() const
{
    return delta_;
}

int32_t Encoder::position() const
{
    return position_;
}

uint8_t Encoder::age() const
{
    return age_;
}

bool Encoder::ok() const
{
    return ok_;
}

bool Encoder::safe() const
{
    return ok_ && (age_ <= MOT_ANGLE_MAX_AGE);
}

void Encoder::consumeBoardCache(uint8_t ok, uint16_t raw)
{
    ok_ = (ok != 0U) && (bsp_angle_ok() != 0U);
    if (!ok_)
    {
        if (age_ < 255U)
        {
            age_++;
        }
        return;
    }

    raw_ = raw;
    electrical_ = electricalFromRaw(raw);
    age_ = 0U;

    if (!initialized_)
    {
        previous_raw_ = raw;
        delta_ = 0;
        initialized_ = true;
        return;
    }

    delta_ = static_cast<int16_t>(raw - previous_raw_);
    previous_raw_ = raw;
    position_ += delta_;
}

uint16_t Encoder::electricalFromRaw(uint16_t raw) const
{
    const int32_t zero = static_cast<int32_t>(MOT_ELEC_ZERO) +
                         static_cast<int32_t>(zero_trim_);
#if MOT_SENSOR_DIR > 0
    return static_cast<uint16_t>(zero + static_cast<int32_t>(raw) *
                                            static_cast<int32_t>(MOT_SENSOR_ELEC));
#else
    return static_cast<uint16_t>(zero - static_cast<int32_t>(raw) *
                                            static_cast<int32_t>(MOT_SENSOR_ELEC));
#endif
}

} // namespace cms32::control
