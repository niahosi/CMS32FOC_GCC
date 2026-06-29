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
/** \file demo_i2c.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_i2c.h"

/****************************************************************************/
/*	Local pre-processor symbols/macros('#define')
*****************************************************************************/
/*----slave 地址+Mask-----------------------------------------------------*/
#define Slave_Addr	   0x50//0x4c//0x98 
#define I2C_S_ADDRMask 0xfe

/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
*****************************************************************************/

/****************************************************************************/
/*	Local type definitions('typedef')
*****************************************************************************/

/****************************************************************************/
/*	Local variable  definitions('static')
*****************************************************************************/
uint8_t *I2CS_RX_Address;        /* I2C slave receive buffer address */

/****************************************************************************/
/*	Local function prototypes('static')
*****************************************************************************/

/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
*****************************************************************************/


/********************************************************************************
 ** \brief	 I2C_Master_Mode
 **			
 ** \param [in] none
 **            	
 ** \return  none
 ** \note  
 ******************************************************************************/
 void I2C_Init(void)
 {
	/*
	 (1)设置I2C通讯时钟
	 */	 
	 CGC_PER12PeriphClockCmd(CGC_PER12Periph_IIC,ENABLE);
	 I2C_ConfigClk(0,11);						/*Fapb =48M,设置时钟SCL = 3.125K, 采样时钟为375Khz*/
	/*
	(3)设置IO复用
	*/
	 GPIO_PinAFOutConfig(P04CFG,IO_OUTCFG_P04_SCL);
	 GPIO_Init(PORT0,PIN4,OPENDRAIN_OUTPUT);
	 
	 GPIO_PinAFOutConfig(P03CFG,IO_OUTCFG_P03_SDA);	 
	 GPIO_Init(PORT0,PIN3,OPENDRAIN_OUTPUT);
	 
	 I2C_ConfigSlaveModeAddr(I2C_S_7BIT_ADDR0,Slave_Addr,I2C_S_BROADCAST_EN,I2C_S_ADDRMask);
	 
	 I2C_EnableInt();
	 I2C_SendACK();
	 I2C_EnableOutput();
	 
	 NVIC_SetPriority(I2C0_IRQn,0);
	 NVIC_EnableIRQ(I2C0_IRQn);
 }



