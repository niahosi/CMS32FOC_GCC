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
/** \file ADC.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "adcldo.h"
#include "cgc.h"

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
 ** \brief	ADCLDO_Enable
 **			使能ADCLDO模块
 ** \param [in] none
 ** \return  none
 ** \note  
*****************************************************************************/
void ADCLDO_Enable(void)
{	
	ADCLDO->LOCK = ADCLDO_LOCK_WRITE_KEY;	
	ADCLDO->CON0 |= ADCLDO_CON0_EN_Msk;
	ADCLDO->LOCK = 0x00;	
}

/*****************************************************************************
 ** \brief	ADCLDO_Disable
 **			使能ADCLDO模块
 ** \param [in] none
 ** \return  none
 ** \note  
*****************************************************************************/
void ADCLDO_Disable(void)
{
	ADCLDO->LOCK = ADCLDO_LOCK_WRITE_KEY;	
	ADCLDO->CON0 &= ~ADCLDO_CON0_EN_Msk;
	ADCLDO->LOCK = 0x00;	
}
/*****************************************************************************
 ** \brief	ADCLDO_OutVlotageSel
 **			选择ADCLDO的输出电压
 ** \param [in] OutVsel : 
 **				        (1)ADCLDO_OutV_4d2： 4.2V输出
 **						    (2)ADCLDO_OutV_3d6： 3.6V输出
 ** \return  none
 ** \note  
*****************************************************************************/
void ADCLDO_OutVlotageSel(uint8_t OutVsel)
{
	uint8_t TempV;
	
	ADCLDO->LOCK = ADCLDO_LOCK_WRITE_KEY;	
	
	TempV = ADCLDO->CON0;
	TempV &= ~ADCLDO_CON0_SEL_Msk;
	if(OutVsel == ADCLDO_OutV_4d2)
		TempV = ADCLDO_OutV_4d2;
	else
		TempV = ADCLDO_OutV_3d6;	
	ADCLDO->CON0 = TempV;
	
	ADCLDO->LOCK = 0x00;	
}

/*****************************************************************************
 ** \brief	ADCLDO_ADJFeedback
 **			读出ADCLDO的修调
 ** \param [in] none
 ** \return  ADCLDO修调
 ** \note  
*****************************************************************************/
uint8_t ADCLDO_ADJFeedback(void)
{
	uint8_t TempR;
	
	TempR = (ADCLDO->CON0) >> ADCLDO_CON0_ADJ_Pos;
	
	return(TempR);
}





