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
#include "cms32m6510.h"
#include "common.h"
#include "i2c.h"
#include "uart.h"
/****************************************************************************/
/*	Local pre-processor symbols/macros('#define')
*****************************************************************************/

/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
*****************************************************************************/
extern uint8_t *I2CS_RX_Address;  
/****************************************************************************/
/*	Local type definitions('typedef')
*****************************************************************************/

/****************************************************************************/
/*	Local variable  definitions('static')
*****************************************************************************/

/****************************************************************************/
/*	Local function prototypes('static')
*****************************************************************************/
volatile	uint32_t  I2C_Receive_Flag =0;			
volatile	uint32_t  I2C_Work_Flag =0;

volatile	uint32_t  I2C_Receive_Data =0;
 volatile	uint32_t  I2C_Send_Data =0;
volatile unsigned char buf[30];
volatile unsigned char *pbuf=buf;

volatile unsigned char sspcount =0;
/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
*****************************************************************************/
extern uint32_t g_ticks;     
/****************************************************************************
 ** \brief	NMI_Handler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void NMI_Handler(void)
{
	
}
/****************************************************************************
 ** \brief	HardFault_Handler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void HardFault_Handler(void)
{
	
}

/****************************************************************************
 ** \brief	SVC_Handler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void SVC_Handler(void)
{
	
}

/****************************************************************************
 ** \brief	PendSV_Handler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void PendSV_Handler(void)
{
	
}
/****************************************************************************
 ** \brief	SysTick_Handler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void SysTick_Handler(void)
{
	UART_Lock(UART0);			//Lock when system used UART
	g_ticks--;	
}
           
/****************************************************************************
 ** \brief	INTP0_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void INTP0_IRQHandler(void)
{
	UART_Lock(UART0);			//Lock when system used UART
	//user code 
}

/****************************************************************************
 ** \brief	INTP1_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void INTP1_IRQHandler(void)
{
	UART_Lock(UART0);			//Lock when system used UART
	//user code 
}

/****************************************************************************
 ** \brief	INTP2_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void INTP2_IRQHandler(void)
{
	UART_Lock(UART0);			//Lock when system used UART
	//user code 
}

/****************************************************************************
 ** \brief	INTP3_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void INTP3_IRQHandler(void)
{
	UART_Lock(UART0);			//Lock when system used UART
	//user code 
}

/****************************************************************************
 ** \brief	I2C_IRQHandler
 **
 ** \param [in]  none   
 ** \return none
 ** \note
****************************************************************************/
void I2C0_IRQHandler(void)
{
	if(I2C_GetIntFlag())
	{
		I2C_Receive_Flag = I2C_GetStatus();
		switch(I2C_Receive_Flag)
		{
/*----------------------------正常模式-----------------------------------------*/
			case  I2C_SS_RECEIVE_ADDR_W:			/*从机接收到主机发送的地址+写，回复ACK*/	
				
				I2C_Work_Flag = 0x01;				/*从机被选中并接收写操作*/	
				I2C_SendACK();						/*后面接收到数据 回复ACK*/				 
				break;
			case  I2C_SS_RECEIVE_DAT_ACK:							/*从机地址匹配后接收到数据，回复ACK*/	

				I2C_Work_Flag =0x02;
				I2C_Receive_Data =	I2C_GetData();		/*获取数据*/
							
				*pbuf++= I2C_Receive_Data;
				
					I2C_SendACK();						/*后面接收到数据 回复ACK*/
				break;	
			case  I2C_SS_RECEIVE_DAT_NO_ACK:							/*从机地址匹配后接收到数据，回复 no ACK*/	
				
				I2C_Work_Flag = 0x05;					/*从机接收到数据*/		
				I2C_NotSendACK();
				break;
			case I2C_SS_RECEIVE_STOP_OR_RESTART:		/*接收到重启或者停止信号*/
				
				pbuf = buf;
				I2C_Work_Flag = 0x04;	

			
				break;	
			
			case  I2C_SS_RECEIVE_ADDR_R_ACK:			/*从机接收到主机发送的地址+读，回复ACK**/	
				
				I2C_Work_Flag = 0x03;					/*从机接收读操作*/	
				/*-------------------*/
				//I2C_NotSendACK();			/*检测状态——>0XC8*/
				/*-------------------*/		
			
				I2C_Send_Data = buf[sspcount++];					
				I2C_SendData(I2C_Send_Data);
				
				break;			
			case  I2C_SS_SEND_DAT_NO_ACK:				/*从机模式下发送数据后，未接收到ACK*/				
				I2C_Work_Flag = 0x07;
				sspcount=0;	
				pbuf = buf;
				break;

			case  I2C_SS_SEND_DAT_ACK:					/*从机模式下发送数据后，接收到ACK*/				
				I2C_Work_Flag = 0x06;				    		

				I2C_Send_Data = buf[sspcount++];					
				I2C_SendData(I2C_Send_Data);					
				break;	


			
			case  I2C_SS_SEND_LAST_DAT_NO_ACK:				/*从机模式下发送最后一个数据后，未接收到ACK*/				
				I2C_Work_Flag = 0x08;				    		
				
				break;	
			
			case  I2C_SS_SEND_LAST_DAT_ACK:				/*从机模式下发送最后一个数据后，接收到ACK:*/				
				I2C_Work_Flag = 0x09;				    		
	
				I2C_Send_Data =5 ;					
				I2C_SendData(I2C_Send_Data);
			
				break;	

			case  I2C_SS_BORADCAST_RECEIVE_DAT_ACK:				/*从机接收广播呼叫地址后接收到数据，回复ACK*/	
				
				I2C_Receive_Data =	I2C_GetData();				/*获取数据*/			
				*pbuf++= I2C_Receive_Data;				  
					
				  I2C_Work_Flag = 0x0A;				    		
			
				break;	
			
			case  I2C_SS_BORADCAST_RECEIVE_DAT_NO_ACK:				/*从机接收广播呼叫地址后接收到数据，不回复ACK*/				
				I2C_Work_Flag = 0x0B;				    		
			
				break;	
			
		}
		if(sspcount > 30)
		{
			sspcount=0;	
		}
		if(pbuf > (buf + 30))
		{
			pbuf = buf;
		}	
		I2C_ClearIntFlag();
	}
}




