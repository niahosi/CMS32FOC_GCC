/***********************************************************************************************************************
* Copyright (C) . All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : main.c
* Version      :  
* Device(s)    : CMS32M6510
* Tool-Chain   : MDK(armcc)
* Description  : This file is a template.
* Creation Date: 2022/8/24
***********************************************************************************************************************/

/***********************************************************************************************************************
Macro Definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include <stdio.h>
#include "demo_uart.h"
#include "demo_adc.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
volatile uint32_t g_ticks;
volatile  uint32_t ADCValue = 0;
double voltage = 3.3;

void delayMS(uint32_t n)
{
		g_ticks = n;
		while(g_ticks);
}

double ADCTempSensor_Judge(int16_t AdcVal)
{
	double RtValue=0;
	RtValue = (double)AdcVal * voltage / 4096;

	return(RtValue);
}
int main()
{
	uint32_t msCnt; 	// count value of 1ms
	uint32_t i,j;
  
  
//-----------------------------------------------------------------------
// Systick setting 
//-----------------------------------------------------------------------
	g_ticks = 1000; 	// 1000ms
	SystemCoreClockUpdate();
	msCnt = SystemCoreClock / 1000;
	SysTick_Config(msCnt); 

	UART0_Config();
	printf("Start ADC\n");
	ADC_TempSensor_Mode();

	while(1)
	{
		for(i=65535;i>0;i--)
			for(j=110;j>0;j--);
		if(!ADC_IS_BUSY())
		{	
			UART_Lock(UART0);
			ADC_Go();
			while(ADC_IS_BUSY());
			ADCValue = ADC_GetResult(ADC_CH_22);		

			printf("Reference Voltage = %3.2fV, ADC Get Value = %04d, Temperature = %3.2f\n", voltage, ADCValue, (double)adc_get_temperature(ADCTempSensor_Judge(ADCValue)));
		}

	}
}

/***********************************************************************************************************************
* Function Name: SysTick Handler
* Description  : Decreament the g_ticks value
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void SysTick_Handler(void)
{
	UART_Lock(UART0);			//Lock when system used UART 
	g_ticks--;
}


