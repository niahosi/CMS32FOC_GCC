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
/** \file demo_extint.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_extint.h"

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
 ** \brief	EXTInt_Init
 **			初始化外部中断
 ** \param [in] Channel:
 **			               EXTINT_Rising: 上升沿触发中断
 **										 EXTINT_Falling: 下降沿触发中断	 
 **										 EXTINT_DoublEdge: 双沿触发中断 
 **						  INTCH:(0~3)
 **										 EXTINT_INT0: INT0
 **										 EXTINT_INT1: INT1
 **										 EXTINT_INT2: INT2
 **										 EXTINT_INT3: INT3 
 ** \return  none
 ** \note	
*****************************************************************************/
void EXTInt_Edge(uint8_t EXTINTEdge, uint8_t INTCH)
{
	if(EXTINTEdge == EXTINT_Rising)
	{	
		INTM->EGP0 |= (1 << INTCH);									//上升沿触发中断	
		INTM->EGN0 &= ~(1 << INTCH);
	}
	else if(EXTINTEdge == EXTINT_Falling)
	{		
		INTM->EGP0 &= ~(1 << INTCH);
		INTM->EGN0 |= (1 << INTCH);									//下降沿触发中断
	}
	else
	{
		INTM->EGP0 |= (1 << INTCH);									//双沿触发中断
		INTM->EGN0 |= (1 << INTCH);
	}
}


/*****************************************************************************
 ** \brief	EXTInt_Init
 **			初始化外部中断
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void EXTInt_Init(void)
{
	/*
	(1)设置端口
	*/
	GPIO_Init(PORT2,PIN2,PULLDOWN_INPUT);		   //P22作为外部中断的输入
	GPIO_PinAFInConfig(INT3CFG,INTxCFGP22);		 //外部中断3，选择P22
	
	/*
	(2)设置触发边沿
	*/
	EXTInt_Edge(EXTINT_Rising,EXTINT_INT3);    //INT3的上升沿触发中断
	
	/*
	(3)使能中断
	*/
	NVIC_EnableIRQ(INTP3_IRQn);							   //使能外部中断3
	
	/*
	(4)设置优先级
	*/
	NVIC_SetPriority(INTP3_IRQn,3);
	
			
}


