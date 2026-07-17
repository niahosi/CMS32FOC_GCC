#include <stdint.h>
#include <stdio.h>

#include "CMS32M6510.h"
#include "adc.h"
#include "cgc.h"
#include "cmsis_gcc.h"
#include "common.h"
#include "delay.h"
#include "demo_uart.h"
#include "system_CMS32M6510.h"

static volatile uint32_t g_adc_irq_count;

static void TestAdc_Init(void);

int main(void)
{
    uint32_t primask;

    SystemCoreClockUpdate();
    delay_init(SystemCoreClock);

    m0_delay_ms(10000U);

    primask = __get_PRIMASK();
    __disable_irq();

    TestAdc_Init();
    UART0_Config();
    UART_Lock_Inline(UART0);

    if (primask == 0U)
    {
        __enable_irq();
    }

    printf("\r\nUART test baud=%lu wait=%lu ms\r\n", (unsigned long)9600U, (unsigned long)10000U);
    printf("UART RX IRQ, ADC TX, '?' -> hex count\r\n");

    while (1)
    {
        __WFI();
    }
}

static void TestAdc_Init(void)
{
    g_adc_irq_count = 0U;

    CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCEN, ENABLE);

    ADC_Stop();
    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_SINGLE, ADC_CLK_DIV_128, 255U);
    ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);
    ADC_ConfigVREF(ADC_VREFP_VDD);
    ADC_DisableChargeAndDischarge();

    ADC_DisableScanChannel(0xFFFFFFFFUL);
    ADC_EnableScanChannel(ADC_CH_21_MSK);
    ADC_DisableChannelInt(0xFFFFFFFFUL);
    ADC_ClearChannelIntFlag(ADC_CH_21);
    ADC_EnableChannelInt(ADC_CH_21_MSK);
    ADC_Start();

    NVIC_ClearPendingIRQ(ADC_IRQn);
    NVIC_SetPriority(ADC_IRQn, 1U);
    NVIC_EnableIRQ(ADC_IRQn);

    ADC_Go();
}

void ADC_IRQHandler(void)
{
    /* Any enabled ISR can interrupt UART access, so release the UART bus gate first. */
    UART_Lock_Inline(UART0);

    if (ADC_GetChannelIntFlag(ADC_CH_21) != 0U)
    {
        ADC_ClearChannelIntFlag(ADC_CH_21);
        g_adc_irq_count++;
    }
    else
    {
        ADC_ClearChannelIntFlag(ADC_CH_21);
    }

    Uart0_AdcTxTask();
    UART_Lock_Inline(UART0);
    ADC_Go();
}
