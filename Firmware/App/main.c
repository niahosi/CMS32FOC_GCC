#include "MotorControl.h"
#include "board_uart.h"
#include "cms32m6510_platform.h" // IWYU pragma: keep
#include "foc_bsp.h"
#include "screw_axis.h"

/**
 * @brief 主固件入口。
 *
 * 主循环只做应用层状态机、命令复制、慢环安全检查和 watch 刷新；
 * 20 kHz ADC/PWM 同步快环在 ADC_IRQHandler() 中运行。
 */
int main(void)
{
    bsp_init();
    /* P06/P07 与 SWD 共用；BoardUart_Init() 内部先等待，再关闭 SWD 切 UART。 */
    BoardUart_Init();

    MotorControl_Init();
    ScrewAxis_Init();
    MotorControl_UpdateWatch(&g_motor_status);
    bsp_start_adc_sync();

    while (1)
    {
        ScrewAxis_Run();
        MotorControl_ApplyCommand(&g_motor_command);
        MotorControl_RunSlowLoop();
        MotorControl_UpdateWatch(&g_motor_status);
    }
}

/**
 * @brief ADC 同步采样中断入口。
 *
 * MotorControl_FastLoopFromAdcIrq() 先让 Board 层解析本次 ADC 采样；
 * 只有形成有效控制采样时，才推进 ScrewAxis 的毫秒计数。
 */
void ADC_IRQHandler(void)
{
    /* 防止之前 UART 访问状态影响 ADC/控制外设寄存器访问。 */
    BoardUart_EndAccess();

    if (MotorControl_FastLoopFromAdcIrq() != 0U)
    {
        ScrewAxis_OnAdcSample();
    }

    /* ADC 中断周期推进 UART TX，每次最多写 1 字节到 THR。 */
    BoardUart_TxTask();
    BoardUart_EndAccess();
}
