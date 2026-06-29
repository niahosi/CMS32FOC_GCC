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
/** \file pga.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "pga.h"
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
 ** \brief	 OPA_ConfigGain
 **			 设置PGA增益
 ** \param [in] pga :PGA0 、PGA1、PGA2
 **				Gain:  (1)	PGA_GAIN_1		
 **					     (2)  PGA_GAIN_2	
 **					     (3)  PGA_GAIN_2d5
 **					     (4)  PGA_GAIN_5	
 **					     (5)  PGA_GAIN_7d5
 **					     (6)  PGA_GAIN_10	
 **					     (6)  PGA_GAIN_15	
 ** \return  none
 ** \note    
 *****************************************************************************/
void PGA_ConfigGain(PGA_TypeDef PGAUnit, uint32_t Gain)
{
	uint8_t Temp=0;
	
	if(PGAUnit == PGA0x)
	{
		PGA0->PGA0LOCK = PGAUNLOCK;
		Temp = PGA0->PGA0CON0;
		Temp &= ~(PGA_CON_PS_Msk);
		Temp |= Gain;
		PGA0->PGA0CON0 = Temp;
		PGA0->PGA0LOCK = PGALOCK;
	}
	else if(PGAUnit == PGA1x)
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		Temp = PGA12->PGA1CON0;
		Temp &= ~(PGA_CON_PS_Msk);
		Temp |= Gain;
		PGA12->PGA1CON0 = Temp;
		PGA12->PGA12LOCK = PGALOCK;
	}
	else
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		Temp = PGA12->PGA2CON0;
		Temp &= ~(PGA_CON_PS_Msk);
		Temp |= Gain;
		PGA12->PGA2CON0 = Temp;
		PGA12->PGA12LOCK = PGALOCK;
	}
}

/*****************************************************************************
 ** \brief	 PGA_ConfigResistorPAD
 **			 设置PGA0输出串联电阻
 ** \param [in] pga :PGA0 
 **				GndMode:	(1) PGA_R_None：不串联电阻
 **							    (2) PGA_R_10K	：串联10K的电阻
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA0_ConfigResistorPAD(uint32_t GndMode)
{
	uint8_t Temp=0;
	
	CGC->RSTM = 1;	//close P02 reset
	PGA0->PGA0LOCK = PGAUNLOCK;
	Temp = PGA0->PGA0CON1;
	Temp &= ~(PGA_CON1_SELR_Msk);
	Temp |= GndMode;
	PGA0->PGA0CON1 = Temp;	
	PGA0->PGA0LOCK = PGALOCK;
}

/*****************************************************************************
 ** \brief	 PGA_EnableOutput
 **			 开启PGA输出
 ** \param [in] pga :PGA0x、PGA1x、PGA2x
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_EnableOutput(PGA_TypeDef PGAUnit)
{
	if(PGAUnit == PGA0x)
	{
		PGA0->PGA0LOCK = PGAUNLOCK;
		PGA0->PGA0CON1 |=( 0x1<< PGA_CON1_OTEN_Pos);
		PGA0->PGA0LOCK = PGALOCK;
	}
	else
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		PGA12->PGA12CON |=( 0x1<< PGA_PGA12_OEN_Pos);
		if(PGAUnit == PGA1x)
			PGA12->PGA12CON &= ~PGA_PGA12_OS_Msk;  //Enable PGA1 out
		else		
		  PGA12->PGA12CON |= PGA_PGA12_OS_Msk;   //Enable PGA2 out
		PGA12->PGA12LOCK = PGALOCK;
	}
}

/*****************************************************************************
 ** \brief	 PGA_DisableOutput
 **			 关闭PGA输出
 ** \param [in] pga :PGA0x、PGA1x、PGA2x
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_DisableOutput(PGA_TypeDef PGAUnit)
{
	if(PGAUnit == PGA0x)
	{
		PGA0->PGA0LOCK = PGAUNLOCK;
		PGA0->PGA0CON1 &= ~( 0x1<< PGA_CON1_OTEN_Pos);
		PGA0->PGA0LOCK = PGALOCK;
	}
	else
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		PGA12->PGA12CON &= ~( 0x1<< PGA_PGA12_OEN_Pos);
		PGA12->PGA12LOCK = PGALOCK;
	}
}

/*****************************************************************************
 ** \brief	 PGA_VrefCtrl
 **			 PGA0参考电压设置
 ** \param [in] pga :PGA0x 、PGA1x、PGA2x
 ** \param [in] VrefStat :
**                  VrefHalf:VREF/2
**                  PGA0BG:BG(0.8V)
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_VrefCtrl(PGA_TypeDef PGAUnit,PGAVref_TypeDef VrefStat)
{
	if(PGAUnit == PGA0x)
	{
		PGA0->PGA0LOCK = PGAUNLOCK;
		if(VrefStat == VrefHalf)
			PGA0->PGA0CON0 &= ~PGA_CON_VRef_Msk;
		else
			PGA0->PGA0CON0 |= PGA_CON_VRef_Msk;
		PGA0->PGA0LOCK = PGALOCK;
	}
	else
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		if(VrefStat == VrefHalf)
			PGA12->PGA12CON &= ~PGA_PGA12_VREF_Msk;
		else
			PGA12->PGA12CON |= PGA_PGA12_VREF_Msk;
		PGA12->PGA12LOCK = PGALOCK;
	}
}

/*****************************************************************************
 ** \brief	 PGA_Start
 **			 开启PGA
 ** \param [in] pga :PGA0x 、PGA1x、PGA2x
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_Start(PGA_TypeDef PGAUnit)
{
	if(PGAUnit == PGA0x)
	{
		PGA0->PGA0LOCK = PGAUNLOCK;
		PGA0->PGA0CON0 |= PGA_CON_EN_Msk;
		PGA0->PGA0LOCK = PGALOCK;
	}
	else if(PGAUnit == PGA1x)
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		PGA12->PGA1CON0 |= PGA_CON_EN_Msk;
		PGA12->PGA12LOCK = PGALOCK;
	}	
	else 
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		PGA12->PGA2CON0 |= PGA_CON_EN_Msk;
		PGA12->PGA12LOCK = PGALOCK;
	}		
}

/*****************************************************************************
 ** \brief	 PGA_Stop
 **			 关闭PGA
 ** \param [in] pga :PGA0 、PGA1
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_Stop(PGA_TypeDef PGAUnit)
{
	if(PGAUnit == PGA0x)
	{
		PGA0->PGA0LOCK = PGAUNLOCK;
		PGA0->PGA0CON0 &= ~PGA_CON_EN_Msk;
		PGA0->PGA0LOCK = PGALOCK;
	}
	else if(PGAUnit == PGA1x)
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		PGA12->PGA1CON0 &= ~PGA_CON_EN_Msk;
		PGA12->PGA12LOCK = PGALOCK;
	}
	else 
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		PGA12->PGA2CON0 &= ~PGA_CON_EN_Msk;
		PGA12->PGA12LOCK = PGALOCK;	
	}
}

/*****************************************************************************
 ** \brief	 PGA_ModeSet
 **			 PGA工作模式设置
 ** \param [in] pga :PGA0x 、PGA1x、PGA2x
 ** \param [in] PgaMode: PgaSingle、PgaDiffer
 ** \return  none	
 ** \note    
 *****************************************************************************/
void PGA_ModeSet(PGA_TypeDef PGAUnit,PGAMode_TypeDef PgaMode)
{
	uint8_t Temp=0;
	
	if(PGAUnit == PGA0x)
	{
		PGA0->PGA0LOCK = PGAUNLOCK;
		Temp = PGA0->PGA0CON0;
		Temp &= ~PGA_CON_MD_Msk;
		Temp |= PgaMode << PGA_CON_MD_Pos;
		PGA0->PGA0CON0 = Temp;
		PGA0->PGA0LOCK = PGALOCK;
	}
	else if(PGAUnit == PGA1x)
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		Temp = PGA12->PGA1CON0;
		Temp &= ~PGA_PGA12CON_MD_Msk;
		Temp |= PgaMode << PGA_PGA12CON_MD_Pos;
		PGA12->PGA1CON0 = Temp;	
		PGA12->PGA12LOCK = PGALOCK;
	}	
	else 
	{
		PGA12->PGA12LOCK = PGAUNLOCK;
		Temp = PGA12->PGA2CON0;
		Temp &= ~PGA_PGA12CON_MD_Msk;
		Temp |= PgaMode << PGA_PGA12CON_MD_Pos;
		PGA12->PGA2CON0 = Temp;	
		PGA12->PGA12LOCK = PGALOCK;
	}		
}

