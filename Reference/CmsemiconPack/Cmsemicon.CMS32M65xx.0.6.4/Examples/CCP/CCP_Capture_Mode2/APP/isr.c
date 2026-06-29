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
#include "epwm.h"
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
uint32_t WavePeriod[6]={0};   //0--PERIOD, 1---CAP1-CAP2,3---CAP2-CAP3
uint8_t Temp=0;

void CCP_IRQHandler(void)
{
	if(CCP_GetOverflowIntFlag(CCP1))
	{
		CCP_ClearOverflowIntFlag(CCP1);
	}	
	if(CCP_GetCAPMode2IntFlag(CAP3))			/*CAP0DATA=周期值，CAPDUTY=CAP1-CAP2,CAP0DATA-CAPDUTY=CAP2-CAP3*/
	{																		
		CCP_ClearCAPMode2IntFlag(CAP3);
		WavePeriod[0+Temp] = CCP_GetCAPMode2Result(CAP1_SUB_CAP3);				//储存捕获周期
		WavePeriod[1+Temp] = CCP_GetCAPMode2Result(CAP1_SUB_CAP2);				//储存捕获的CAP1-CAP2脉宽
		WavePeriod[2+Temp] = WavePeriod[0+Temp] - WavePeriod[1+Temp];		  //储存捕获的CAP2-CAP3脉宽
		Temp += 3;

		CCP_DisableRun(CCP1);		/*实现重新捕获的功能： 先Stop再Run*/
		CCP_EnableRun(CCP1);	
	}	
}

/****************************************************************************
 ** \brief	EPWM_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void EPWM_IRQHandler(void)
{
	if(EPWM_GetZeroIntFlag(EPWM0))
	{
		EPWM_ClearZeroIntFlag(EPWM0);
	}	
}



