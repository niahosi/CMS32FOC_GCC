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
/** \file demo_lsitimer.c
**
**  
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_lsitimer.h"

/****************************************************************************/
/*	Local pre-processor symbols('#define')
****************************************************************************/

/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
****************************************************************************/

/****************************************************************************/
/*	Local type definitions('typedef')
****************************************************************************/

/****************************************************************************/
/*	Local variable  definitions('static')
****************************************************************************/

/****************************************************************************/
/*	Local function prototypes('static')
****************************************************************************/
	
/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
****************************************************************************/
/******************************************************************************
 ** \brief	 LSITIMER_Config
 ** \param [in] 
 **            	
 ** \return  none
 ** \note  
 ******************************************************************************/
void LSITIMER_Config(void)
{
	//设置LSI timer时钟
	CGC_PER0PeriphClockCmd(CGC_PER0Periph_LSITIMER,ENABLE);
	CGC_LSI_Enable();				    //使能系统内部低速时钟
	CGC_LSITIMERCLK_Enable();		//使能LSI TIMER时钟
	
	//关闭LSI timer
	LSITIMER_Disable();		//关闭LSI TIMER
	
	//配置LSI timer比较时间
	LSITIMER_ConfigInput(10);		//设置比较时间为1.333MS 
	
	//使能LSI timer中断
	LSITIMER_EnableInt();
	NVIC_SetPriority(LSITIMER_IRQn,3);
	NVIC_EnableIRQ(LSITIMER_IRQn);
//	
	//开启LSI timer
	LSITIMER_Enable();	
}




