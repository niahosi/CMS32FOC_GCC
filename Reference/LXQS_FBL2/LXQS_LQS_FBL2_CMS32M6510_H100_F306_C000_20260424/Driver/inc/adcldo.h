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

/*****************************************************************************/
/** \file adcldo.h
**
** History:
** 
*****************************************************************************/
#ifndef __ADCLDO_H_
#define __ADCLDO_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include "common.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/

/*----------------------------------------------------------------------------
 **ADC 时钟分频
-----------------------------------------------------------------------------*/
#define 	ADCLDO_OutV_4d2		(0x55UL)			/*ADCLDO输出4.2V电压*/
#define 	ADCLDO_OutV_3d6		(0x00UL)			/*ADCLDO输出3.6V电压*/


/*----------------------------------------------------------------------------
 **ADCLDO   写保护
-----------------------------------------------------------------------------*/
#define		ADCLDO_LOCK_WRITE_KEY		(0x55UL)

/*****************************************************************************
 ** \brief	ADCLDO_Enable
 **			使能ADCLDO模块
 ** \param [in] none
 ** \return  none
 ** \note  
*****************************************************************************/
void ADCLDO_Enable(void);

/*****************************************************************************
 ** \brief	ADCLDO_Disable
 **			使能ADCLDO模块
 ** \param [in] none
 ** \return  none
 ** \note  
*****************************************************************************/
void ADCLDO_Disable(void);

/*****************************************************************************
 ** \brief	ADCLDO_OutVlotageSel
 **			选择ADCLDO的输出电压
 ** \param [in] OutVsel : 
 **				        (1)ADCLDO_OutV_4d2： 4.2V输出
 **						    (2)ADCLDO_OutV_3d6： 3.6V输出
 ** \return  none
 ** \note  
*****************************************************************************/
void ADCLDO_OutVlotageSel(uint8_t OutVsel);
	
/*****************************************************************************
 ** \brief	ADCLDO_ADJFeedback
 **			读出ADCLDO的修调
 ** \param [in] none
 ** \return  ADCLDO修调
 ** \note  
*****************************************************************************/
uint8_t ADCLDO_ADJFeedback(void);


#ifdef __cplusplus
}
#endif

#endif /* __ADCLDO_H_ */

