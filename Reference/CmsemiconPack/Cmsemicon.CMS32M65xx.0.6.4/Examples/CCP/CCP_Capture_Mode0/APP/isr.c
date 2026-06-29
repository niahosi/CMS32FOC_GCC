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
#include "ccp.h"
#include "uart.h"

/****************************************************************************/
/*	Local pre-processor symbols/macros('#define')
*****************************************************************************/

/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
*****************************************************************************/
extern volatile uint32_t  CaptureTime;

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
    
/****************************************************************************
 ** \brief	CCP_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void CCP_IRQHandler(void)
{
	UART_Lock(UART0);			//Lock when system used UART 
	
	if(CCP_GetOverflowIntFlag(CCP0))
	{
		CCP_ClearOverflowIntFlag(CCP0);
	}	
	if(CCP_GetCAPMode0IntFlag(CCP0))
	{																			/*捕获的时间（高电平时间）= （4800（计数初值）- 捕获值）/ CPP0的时钟频率 */
		CaptureTime = (4800 - CCP_GetCAPMode0Result(CCP0,CCPxA)) / 0.75 ;		/*测试的高电平时间小于CPP计数周期（6400us），所以不用计算CCP模块计数溢出时的情况*/
		CCP_ClearCAPMode0IntFlag(CCP0);

		CCP_DisableRun(CCP0);		/*实现重新捕获的功能： 先Stop再Run*/
		CCP_EnableRun(CCP0);	
	}	
}




