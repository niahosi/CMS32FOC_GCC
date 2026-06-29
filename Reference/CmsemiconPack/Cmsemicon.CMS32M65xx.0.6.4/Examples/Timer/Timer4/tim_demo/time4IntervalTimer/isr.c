/*******************************************************************************
* Copyright (C) 2019 China Micro Semiconductor Limited Company. All Rights Reserved.
*
* This software is owned and published by:
* CMS LLC, No 2609-10, Taurus Plaza, TaoyuanRoad, NanshanDistrict, Shenzhen, China.
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with CMS
* components. This software is licensed by CMS to be adapted only
* for use in systems utilizing CMS components. CMS shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. CMS is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/

/****************************************************************************/
/** \file isr.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "common.h"
#include "timer4.h"
#include "uart.h"

/****************************************************************************/
/*	Local pre-processor symbols/macros('#define')
*****************************************************************************/
uint32_t g_Int_Taken;
uint32_t InterTim=0;
uint8_t interval_time_flag;
/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
*****************************************************************************/

/****************************************************************************/
/*	Local type definitions('typedef')
*****************************************************************************/

/****************************************************************************/
/*	Local variable  definitions('static')
*****************************************************************************/


/****************************************************************************/
/*	Local function prototypes('static')
*****************************************************************************/

/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
*****************************************************************************/
 
 /**
  * @brief  SysTick Handler Decreament the g_ticks value
  * @param  TM13 IRQ
  * @retval None
  */
void SysTick_Handler(void)
{
		UART_Lock(UART0);			//Lock when system used UART
		//user code 
}

/****************************************************************************
 ** \brief	TMR0_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void TM00_IRQHandler(void)
{
	  UART_Lock(UART0);			//Lock when system used UART

	  NVIC_ClearPendingIRQ(TM00_IRQn);    // clear INTTM00 interrupt flag
	
		g_Int_Taken++;											//g_Int_Taken increase 1
		if (1U == (TM40->TSR00 & _0001_TM4_OVERFLOW_OCCURS))
			{
					InterTim = (uint32_t)(TM40->TDR00 + 1UL) + 0x10000UL;
			}
			else
			{
					InterTim = (uint32_t)(TM40->TDR00 + 1UL);
			}		
}

void TM01_IRQHandler(void)
{
	 UART_Lock(UART0);			//Lock when system used UART

	 NVIC_ClearPendingIRQ(TM01_IRQn);    // clear INTTM00 interrupt flag
		
}	

void TM02_IRQHandler(void)
{
	 UART_Lock(UART0);			//Lock when system used UART

	 NVIC_ClearPendingIRQ(TM02_IRQn);    // clear INTTM00 interrupt flag
	 interval_time_flag = 1;
}	

