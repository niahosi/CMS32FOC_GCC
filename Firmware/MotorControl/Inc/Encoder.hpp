#pragma once

#include <stdint.h>

namespace cms32::control {

class Encoder
{
public:
    void reset();
    void updateFromBoard();
    void updateFastFromBoard();
    void setZeroTrim(int16_t trim);
    uint16_t raw() const;
    uint16_t electrical() const;
    int16_t delta() const;
    int32_t position() const;
    uint8_t age() const;
    bool ok() const;
    bool safe() const;

private:
    void consumeBoardCache(uint8_t ok, uint16_t raw);
    uint16_t electricalFromRaw(uint16_t raw) const;

    uint16_t raw_ = 0;
    uint16_t electrical_ = 0;
    uint16_t previous_raw_ = 0;
    int16_t delta_ = 0;
    int32_t position_ = 0;
    int16_t zero_trim_ = 0;
    uint8_t age_ = 255;
    bool initialized_ = false;
    bool ok_ = false;
};

} // namespace cms32::control
