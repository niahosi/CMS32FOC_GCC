#include "MotorControl.h"
#include "cms32m6510_platform.h" // IWYU pragma: keep
#include "foc_bsp.h"
#include "screw_axis.h"

int main(void)
{
    bsp_init();
    MotorControl_Init();
    ScrewAxis_Init();
    MotorControl_UpdateWatch(&g_motor_watch);
    bsp_start_adc_sync();

    while (1)
    {
        ScrewAxis_Run();
        MotorControl_ApplyCommand(&g_motor_cmd);
        MotorControl_RunSlowLoop();
        MotorControl_UpdateWatch(&g_motor_watch);
    }
}

void ADC_IRQHandler(void)
{
    if (MotorControl_FastLoopFromAdcIrq() != 0U)
    {
        ScrewAxis_OnAdcSample();
    }
}
