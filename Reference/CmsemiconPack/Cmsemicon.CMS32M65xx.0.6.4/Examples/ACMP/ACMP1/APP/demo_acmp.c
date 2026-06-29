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
/** \file demo_acmp.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_acmp.h"
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
 ** \brief	ACMP_ACMP1_Config
 **			实现ACMP1的比较功能
 **			（1）正端电压(IO)>负端电压 输出1(高电平)
 **			（2）正端电压(IO)<负端电压 输出0(低电平)
 **			（3）输出从0跳变到1 ，产生上升沿中断
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void ACMP_ACMP1_Config(void)
{
	/*
	(0)开启ACMP模块时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_ACMPEN,ENABLE);	
	
	/*
	(1)配置ACMP IO口
	*/
	GPIO_Init(PORT2,PIN4,ANALOG_INPUT);/*设置P24为ACMP C1P0正端输入*/
	GPIO_Init(PORT2,PIN7,ANALOG_INPUT);/*设置P27为ACMP C1N负端输入*/
	
	GPIO_Init(PORT2,PIN6,OUTPUT);/*设置P26为ACMP 输出C1O*/
	GPIO_PinAFInConfig(P26CFG,IO_OUTCFG_P26_C1_O);
	/*
	(2)设置ACMP 正端输入
	 */
	ACMP_ConfigPositive(ACMP1,ACMP_POSSEL_P0);
	/*
	(3)设置ACMP 负端输入 
	*/	
	ACMP_ConfigNegative(ACMP1,ACMP_NEGSEL_N);		/*负端选择C1N*/
	/*
	(4)使能ACMP输出 
	*/
		ACMPOut_Enable(ACMP1);
	 /*
	 (5)关闭事件输出
	 */
	 ACMP_DisableEventOut(ACMP1);
	
	 /*
	 (6)设置中断
	 */
	 ACMP_ConfigEventAndIntMode(ACMP1,ACMP_EVENT_INT_RISING);	/*上升沿触发中断与事件，中断触发方式与事件触发方式共用*/		 
	 ACMP_EnableInt(ACMP1);									
	 NVIC_EnableIRQ(ACMP_IRQn);
	/*
	(7)设置中断优先级
	*/	 	 
	 NVIC_SetPriority(ACMP_IRQn,0);
	/*
	(8)设置迟滞
	*/	 	 
	ACMP_DisableHYS(ACMP1);
	/*
	(9)开启ACMP
	*/
  	ACMP_Start(ACMP1);		
}

