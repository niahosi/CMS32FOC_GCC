#include "MotorControl.h"

#include "Axis.hpp"

extern "C" {
#include "foc_bsp.h"
}

namespace {

cms32::control::Axis g_axis;

} // namespace

extern "C" void MotorControl_Init(void)
{
    g_axis.init();
}

extern "C" void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command)
{
    if (command == nullptr)
    {
        return;
    }
    g_axis.applyCommand(*command);
}

extern "C" void MotorControl_RunSlowLoop(void)
{
    g_axis.runSlowLoop();
}

extern "C" uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    const uint8_t sample_ready = bsp_adc_irq();
    if (sample_ready != 0U)
    {
        g_axis.updateEncoderFast();
        g_axis.runFastLoop();
    }
    return sample_ready;
}

extern "C" void MotorControl_GetWatch(MotorControlWatch_t* out)
{
    if (out == nullptr)
    {
        return;
    }

    g_axis.fillWatch(*out);
}

extern "C" void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out)
{
    MotorControlWatch_t snapshot;

    if (out == nullptr)
    {
        return;
    }

    g_axis.fillWatch(snapshot);
    const uint8_t* src = reinterpret_cast<const uint8_t*>(&snapshot);
    volatile uint8_t* dst = reinterpret_cast<volatile uint8_t*>(out);
    for (uint32_t i = 0; i < sizeof(snapshot); i++)
    {
        dst[i] = src[i];
    }
}
