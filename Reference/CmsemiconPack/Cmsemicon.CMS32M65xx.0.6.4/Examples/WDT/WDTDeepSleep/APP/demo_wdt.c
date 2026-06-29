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
/** \file demo_wdt.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_wdt.h"

/****************************************************************************/
/*	Local pre-processor symbols/macros('#define')
*****************************************************************************/

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
/*****************************************************************************
 ** \brief	WDT_Config
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void WDT_Config(void)
{
	/*
	(1)设置WDT时钟
	*/
	WDT_ConfigClk(WDT_CLK_DIV_16);		/*Fwdt = LSI(40KHz)/16 = 2.5Khz ->Pwdt = 400us */
	/*
	(2)设置WDT是否复位MCU
	*/	
	WDT_DisableRestSystem();			/*禁止WDT溢出复位MCU*/	
	
	/*
	(3)设置仿真态时(断点功能)WDT是否停止计数
	*/	
	WDT_DISABLE_DEBUG();				/*仿真态时停止计数*/
										/*WDT复位使能 + 仿真态继续计数 -> 程序调试（断点）时，WDT继续计数并且溢出。若调试时间过长，WDT计数溢出且没清除WDT中断标志位，则WDT会复位芯片*/
	/*
	(4)设置WDT的溢出周期
	*/		
	WDT_ConfigPeriod(2500);				   /* 400us * 2500 = 1s */
	/*
	(6)设置WDT的溢出中断
	*/		
	WDT_EnableOverflowInt();
	NVIC_EnableIRQ(WDT_IRQn);
	/*
	(7)设置WDT中断优先级
	*/	
	NVIC_SetPriority(WDT_IRQn, 3);		
	
}




