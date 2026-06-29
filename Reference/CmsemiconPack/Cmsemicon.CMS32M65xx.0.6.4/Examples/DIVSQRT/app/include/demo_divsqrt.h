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
/** \file demo_timer.h
**
**
**
** History:
** - 
*****************************************************************************/
#ifndef __DEMO_LSITIMER_H__
#define __DEMO_LSITIMER_H__
/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include "divsqrt.h"
#include "cgc.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/


/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/
typedef struct { 
int32_t	Divsqrt_DivA;			  //dividend
int32_t	Divsqrt_DivB;				//divisor
int32_t	Divsqrt_Result;			//quotient
int32_t	Divsqrt_remainder;	//remainder
}Divsqrt_Param;


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/

/******************************************************************************
 ** \brief	 DIVSQRT_Config
 ** \param [in] DatA：DIVSQRTALUA （dividend）
 ** \param [in] DatA：DIVSQRTALUB （divisor）
 **            	
 ** \return  none
 ** \note  
 ******************************************************************************/
void DIVSQRT_Config(Divsqrt_Param* DIVPar);


#endif /* __DEMO_LSITIMER_H__ */



