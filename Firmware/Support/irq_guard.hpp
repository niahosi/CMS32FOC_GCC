#pragma once

#include "CMS32M6510.h"

namespace cms32::support
{

// 只用于很短的临界区；不要包住串口解析、watch 填充或 FOC 计算。
template <IRQn_Type Irq> class NvicIrqGuard
{
public:
    NvicIrqGuard() noexcept
    {
        NVIC_DisableIRQ(Irq);
    }

    ~NvicIrqGuard() noexcept
    {
        NVIC_EnableIRQ(Irq);
    }

    NvicIrqGuard(const NvicIrqGuard&) = delete;
    NvicIrqGuard& operator=(const NvicIrqGuard&) = delete;
};

using AdcIrqGuard = NvicIrqGuard<ADC_IRQn>;

} // namespace cms32::support
