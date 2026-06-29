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
/** \file demo_divsqrt.c
**
**  
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_divsqrt.h"

/****************************************************************************/
/*	Local pre-processor symbols('#define')
****************************************************************************/

/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
****************************************************************************/
extern uint32_t DIVRES0, DIVRES1;
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
 ** \brief	 DIVSQRT_Config
 ** \param [in] DatA：DIVSQRTALUA （dividend）
 ** \param [in] DatA：DIVSQRTALUB （divisor）
 **            	
 ** \return  none
 ** \note  
 ******************************************************************************/
void DIVSQRT_Config(Divsqrt_Param* DIVPar)
{
	uint32_t i=0xffff;
	//设置divsqrt时钟
	CGC_PER12PeriphClockCmd(CGC_PER12Periph_DIV,ENABLE);
	while(i--);

	DIVSQRT_EnableUnsingedMode();
	DIVSQRT_EnableDIVMode();
//	DIVSQRT_EnableSQRTMode();
	//计算商
	DIVPar->Divsqrt_Result = DIVSQRT_Div(DIVPar->Divsqrt_DivA,DIVPar->Divsqrt_DivB);
	
	//读取余数
	DIVPar->Divsqrt_remainder = DIVSQRT_GetMod();


}



