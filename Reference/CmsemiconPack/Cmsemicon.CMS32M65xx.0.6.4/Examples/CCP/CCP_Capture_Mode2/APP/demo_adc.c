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
/** \file demo_adcb.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_adc.h"

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
 ** \brief	ADCB_Software_Trigger_Mode
 **			设置ADC软件触发转换
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void ADC_Software_Trigger_Mode(void)
{
	/*
	(1)设置ADC时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCEN,ENABLE);

	ADC_ConfigRunMode(ADC_MODE_HIGH,ADC_CONVERT_SINGLE,ADC_CLK_DIV_32,100);	/*单次模式+高速模式+32分频+100.5 ADCClk采样保持时间*/
	
	ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);	/*硬件自动切换*/
	
	ADC_DisableChargeAndDischarge();					/*关闭充电\放电功能*/
	
	ADC_ConfigVREF(ADC_VREFP_VDD,ADC_VREFN_VSS);		/*设置参考源*/

	/*
	(2)设置ADC通道使能
	*/	
	ADC_EnableScanChannel(ADC_CH_1_MSK);				/*选择AN1*/	
//	SYS_SET_IOCFG(IOP01CFG,SYS_IOCFG_P01_AN1);			/*关闭P01的数字功能*/
	GPIO_Init(PORT2,PIN0,INPUT);	
	/*
	(3)设置ADC中断
	*/		
	
	ADC_EnableChannelInt(ADC_CH_1_MSK);				/*开AN1转换中断*/	
	NVIC_EnableIRQ(ADC_IRQn);
	/*
	(4)设置优先级
	*/	
	NVIC_SetPriority(ADC_IRQn,2);					/*优先级0~3， 0最高、3最低*/	
	/*
	(5)开启ADCB
	*/		
	ADC_Start();
			
}


