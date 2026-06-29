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
/** \file main.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "CMS32M6510.h"
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
volatile uint32_t g_ticks;
/****************************************************************************/
/*	Local function prototypes('static')
*****************************************************************************/
void PowerDown_Precess(void);
void GPIO_Sleep(void);

/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
*****************************************************************************/
void delayMS(uint32_t n)
{
		g_ticks = n;
		while(g_ticks);
}
/*****************************************************************************
 ** \brief	 main
 **
 ** \param [in]  none   
 **
 ** \return 0
 ** \note  调试口会影响功耗,关闭调试口功耗更低
*****************************************************************************/
int main(void)
{		
	uint32_t msCnt; 	// count value of 1ms
    
//-----------------------------------------------------------------------
// Systick setting 
//-----------------------------------------------------------------------
	g_ticks = 1000; 	// 1000ms
	SystemCoreClockUpdate();
	msCnt = SystemCoreClock / 1000;
	SysTick_Config(msCnt);

	//休眠前先设置GPIO,关闭外设耗电
	GPIO_Sleep();
	PowerDown_Precess();
  /*--EXT INT P22唤醒休眠-----------------------------------------------*/
	EXTInt_Init();
	//不用展示口功耗会更低
	GPIO_Init(PORT0,PIN1,OUTPUT);
//	DBG->DBGSTOPCR = 0x1000000;//关闭调试口(需通过烧写器烧写使能程序恢复调试口)
	__STOP(); 				     // Enter deep sleep mode
	while(1)
	{	
		PORT_ToggleBit(PORT0,PIN1);
		delayMS(2);
	}
		
}

void PowerDown_Precess(void)
{
	NVIC->ICER[0]=0xFFFFFFFF;//关中断
	CGC->PER13=0x01;//ADC时钟使能
	ADC->LOCK=0x55;
	ADC->PWMTGDLY=(0x01<<12);//关LPF	
}
/**悬空/未使用/未封装的管脚需要配置为输出，并配置固定电平。******/
void GPIO_Sleep(void)
{
	RESTPinGpio_Set(ENABLE);//P02 as GPIO
	
	PORT->P00CFG= 0X00;			//Set to GPIO
	PORT->P01CFG= 0X00;
	PORT->P02CFG= 0X00;
	PORT->P03CFG= 0X00;
	PORT->P04CFG= 0X00;
	PORT->P05CFG= 0X00;
//	PORT->P06CFG= 0X00;		//swdclk
//	PORT->P07CFG= 0X00;		//swddat

	PORT->P10CFG= 0X00;			//Set to GPIO
	PORT->P11CFG= 0X00;
	PORT->P12CFG= 0X00;
	PORT->P13CFG= 0X00;
	PORT->P14CFG= 0X00;
	PORT->P15CFG= 0X00;
	PORT->P16CFG= 0X00;

	PORT->P20CFG= 0X00;			//Set to GPIO
	PORT->P21CFG= 0X00;
	PORT->P22CFG= 0X00;
	PORT->P23CFG= 0X00;
	PORT->P24CFG= 0X00;
	PORT->P25CFG= 0X00;
	PORT->P26CFG= 0X00;
	PORT->P27CFG= 0X00;

  PORT->POM0 =0X00;  		//normal out
  PORT->POM1 =0X00;  		//normal out
	PORT->POM2 =0X00;  		//normal out

  PORT->PMC0 =0X00;  		//digital io
  PORT->PMC1 =0X00;  		//digital io
	PORT->PMC2 =0X00;  		//digital io
	
  PORT->PU0 =0X00;  		//disable pull up
  PORT->PU1 =0X00;  		//disable pull up
  PORT->PU2 =0X00;  		//disable pull up
	
  PORT->PD0 =0X00;  		//disable pull down
  PORT->PD1 =0X00;  		//disable pull down
  PORT->PD2 =0X00;  		//disable pull down
	
	PORT->PM0 =0x00;  		//out mode
	PORT->P0 =0x00;				//out low
	
	PORT->PM1 =0X00;  		//out mode
	PORT->P1 =0x00;				//out low
	
	PORT->PM2 =0X00;  		//out mode
	PORT->P2 =0x00;				//out low	
}
















