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
/** \file demo_uart.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_uart.h"

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
 ** \brief	UART0_Config
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void  UART0_Config(void)
{	
	
	
	CGC_PER12PeriphClockCmd(CGC_PER12Periph_UART,ENABLE);
	UART_ConfigRunMode(UART0, 9600, UART_WLS_8, UART_PARITY_NONE,UART_STOP_BIT_1);
	
	GPIO_PinAFOutConfig(P04CFG,IO_OUTCFG_P04_TXD);	
	GPIO_Init(PORT0,PIN4,OUTPUT);
}

/******************************************************************************
 ** \brief	 Uart0_putchar
 **			 
 ** \param [in] 
 ** \return  none
 ** \note   
*****************************************************************************/
void Uart0_putchar(char ch)
{
	while(!(UART0->LSR & UART_LSR_THRE_Msk));
	UART0->THR = ch;
}

/******************************************************************************
 ** \brief	 Uart0_getchar
 **			 
 ** \param [in] 
 ** \return  none
 ** \note   
*****************************************************************************/
char Uart0_getchar(void)
{
	while(!(UART0->LSR & UART_LSR_RDR_Msk));
	return  UART0->RBR;
}
