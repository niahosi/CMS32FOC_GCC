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
/** \file demo_spi.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_spi.h"
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
/*****************************************************************************
 ** \brief	SPI_M95256_Start		
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void SPI_M95256_Start(void)
{
	SSP_MasterClearCS();
}
/*****************************************************************************
 ** \brief	SPI_M95256_Stop
 ** \param [in] none
 ** \return  none
 ** \note	 
*****************************************************************************/
void SPI_M95256_Stop(void)
{
	SSP_MasterSetCS();
}
/*****************************************************************************
 ** \brief	SPI_Transmit
 **			
** \param [in] SendData: 发送的值
 ** \return  16bit 获取的值
 ** \note	
*****************************************************************************/
uint32_t  SPI_Transmit(uint32_t  Data)
{
	while(SSP_GetBusyFlag());			
	while(!SSP_GetTFEFlag());
	SSP_SendData(Data);
	while(!SSP_GetRNEFlag());
	return (SSP_GetData());			
}
/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
*****************************************************************************/

/***************************************************************************
 ** \brief	 SPI_Master_Mode
 **
 ** \param [in]  none   
 ** \return none
 ** \note
***************************************************************************/
void SPI_Master_Mode(void)
{
	/*
	(1)设置SSP的时钟
	*/
	CGC_PER12PeriphClockCmd(CGC_PER12Periph_SPI,ENABLE);		/*开启SSP模块时钟*/
	SSP_ConfigClk(23,2);					/*Fapb = 48Mhz,  sclk = 1Mhz*/								
	/*
	(2)设置SSP 为SPI模式
	*/							
	SSP_ConfigRunMode(SSP_FRAME_SPI,SSP_CPO_0,SSP_CPHA_0,SSP_DAT_LENGTH_8);	/*设置为SPI模式，根据95256的时钟与相位设置CPHA =0, CP0 = 0(时钟空闲时为0)*/																		
	/*
	(3)设置SPI 控制模式
	*/
	SSP_EnableMasterMode();							/*设置SPI为主控模式*/
	SSP_DisableMasterAutoControlCS();			
	/*
	(4)设置SPI IO
	*/
	GPIO_PinAFOutConfig(P03CFG,IO_OUTCFG_P03_SCK);
	GPIO_Init(PORT0,PIN3,OUTPUT);
	
	GPIO_PinAFOutConfig(P04CFG,IO_OUTCFG_P04_MISO);
	GPIO_Init(PORT0,PIN4,INPUT);	
	
	GPIO_PinAFOutConfig(P05CFG,IO_OUTCFG_P05_MOSI);
	GPIO_Init(PORT0,PIN5,OUTPUT);	
	CGC->RSTM = 1;
	GPIO_PinAFOutConfig(P02CFG,IO_OUTCFG_P02_SSIO);
	GPIO_Init(PORT0,PIN2,OUTPUT);	

	/*
	(5)开启SPI
	*/	
	SPI_M95256_Stop();	
	SSP_Start();
	
}
/***************************************************************************
 ** \brief	 SPI_M95256_Write
 **			 
 ** \param [in]  addr: 
 **				 buf:
 ** \return 
 ** \note
***************************************************************************/
void SPI_M95256_Write(uint32_t addr, uint32_t buf)
{
	uint32_t temp;
	
	SPI_M95256_Start();
	SPI_Transmit(M95256_WREN);
	SPI_M95256_Stop();			
	for(temp=2;temp>0;temp--);	
	
	SPI_M95256_Start();
	SPI_Transmit(M95256_WRITE);	
	SPI_Transmit(addr>>16);
	SPI_Transmit(addr>>8);
	SPI_Transmit(addr);		
	SPI_Transmit(buf);	
	SPI_M95256_Stop();	

	for(temp=2;temp>0;temp--);	
	SPI_M95256_Start();
	SPI_Transmit(M95256_WRDI);
	SPI_M95256_Stop();		

}
/***************************************************************************
 ** \brief	 SPI_M95256_Read_Data
 **			
** \param [in]  addr :   
 ** \return 8bit Data
 ** \note
***************************************************************************/
uint32_t SPI_M95256_Read_Data(uint32_t addr)
{
	uint32_t temp;
	SPI_M95256_Start();
	SPI_Transmit(M95256_READ);
	SPI_Transmit(addr>>16);	
	SPI_Transmit(addr>>8);
	SPI_Transmit(addr);
	temp = SPI_Transmit(0x00);		
	SPI_M95256_Stop();
	return temp;
}

/***************************************************************************
 ** \brief	 SPI_M95256_Read_SFR
 **			
 ** \param [in]  cmd:	
 ** \return 8bit Data
 ** \note
***************************************************************************/
uint32_t  SPI_M95256_Read_SFR(uint32_t cmd)
{
	uint8_t temp;
	SPI_M95256_Start();
	SPI_Transmit(cmd);
	temp = SPI_Transmit(0x00);
	SPI_M95256_Stop();	
	return temp;	
}



