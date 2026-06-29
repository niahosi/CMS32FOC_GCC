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
/** \file demo_ccp.c
**
**	History:
**	
*****************************************************************************/
/****************************************************************************/
/*	include files
*****************************************************************************/
#include "demo_ccp.h"

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
 ** \brief	CCP_Capture_Mode0_Config
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void CCP_Capture_Mode0_Config(void)
{
	/*
	(1)设置CCP的时钟
	*/
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_CCP,ENABLE);				/*开启CCP模块时钟*/
	CCP_ConfigCLK(CCP0,CCP_CLK_DIV_64,CCP_RELOAD_CCPLOAD,4800);			/*Fccp = 48Mhz /64 = 0.75Mh ->捕获计数周期 = 4800/0.75Mhz = 6400us*/
																		/*CCP0模块溢出后重载值为4800*/
	/*
	(2)设置CCP运行模式
	*/
	CCP_EnableCAPMode0(CCP0);											/*设置CCP0的工作模式为捕获模式0*/
	CCP_ConfigCAPMode0(CCP0,CCP_CAP_MODE0_RISING_TO_FALLING);			/*设置CCP0的捕获模式0 的捕获方式为上升沿到下降沿(高电平时间)*/
																		/*上升/下降沿捕获：CCP模块使能(CCPRUN =1)后，计数值从4800(加载值)向下计数，捕获条件产生，停止计数，并锁存当前的计数值*/
													/*上升到下降沿捕获：CCP模块使能(CCPRUN =1)后，当出现上升时计数值从4800(加载值)向下计数，捕获条件(下降沿)产生，停止计数，并锁存当前的计数值*/
													/*下降到上升沿捕获：CCP模块使能(CCPRUN =1)后，当出现下降沿时计数值从4800(加载值)向下计数，捕获条件(上升沿)产生，停止计数，并锁存当前的计数值*/
	/*
	(3)设置CCP捕获通道
	*/
	CCP_SelCAPMode0Channel(CCP0,CCPxA);									/*设置捕获通道为CCPA0*/
	GPIO_PinAFInConfig(CCP0AINCFG, IO_INCFG_P04_CCP0A_I);				/*设置P04为CCPA0捕获通道*/

	GPIO_Init(PORT0,PIN4,INPUT);
	/*
	(4)设置CCP捕获中断
	*/	
	CCP_EnableOverflowInt(CCP0);					/*开启CCP0模块计数溢出中断*/
	CCP_EnableCAPMode0Int(CCP0);				    /*开启CCP0模块捕获模式0中断*/
	NVIC_EnableIRQ(CCP_IRQn);		
	/*
	(5)设置CCP 中断优先级
	*/	
	NVIC_SetPriority(CCP_IRQn,3);					/*优先级0~3， 0最高、3最低*/		
	/*
	(6)开启CCP模块
	*/	
	CCP_EnableRun(CCP0);			
	CCP_Start(CCP0);			/*CCP0模块使能*/			
}


/*****************************************************************************
 ** \brief	CCP_PWM_Config
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void CCP_PWM_Config(void)
{
	/*
	(1)设置CCP的时钟
	*/
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_CCP,ENABLE);				/*开启CCP模块时钟*/
	CCP_ConfigCLK(CCP0,CCP_CLK_DIV_1,CCP_RELOAD_CCPLOAD,4800);		/*Fahb=48M, Pccp0 = 100us*/
	CCP_ConfigCLK(CCP1,CCP_CLK_DIV_1,CCP_RELOAD_CCPLOAD,4800);		/*Fahb=48M, Pccp1 = 100us*/	
	/*
	(2)设置CCP运行模式
	*/
	CCP_EnablePWMMode(CCP0);		/*设置CCP0为PWM模式*/
	CCP_EnablePWMMode(CCP1);		/*设置CCP1为PWM模式*/
	/*
	(3)设置50%占空比输出
	*/	
	CCP_ConfigCompare(CCP0, CCPxA, 960); 	/*20us*/
	CCP_ConfigCompare(CCP0, CCPxB, 1920); 	/*40us*/	
	CCP_ConfigCompare(CCP1, CCPxA, 2880); 	/*60us*/
	CCP_ConfigCompare(CCP1, CCPxB, 3840); 	/*80us*/
	/*
	(4)设置CCP 反相输出
	*/		
	CCP_DisableReverseOutput(CCP0,CCPxA);	/*关闭反相输出*/
	CCP_DisableReverseOutput(CCP0,CCPxB);  
	CCP_DisableReverseOutput(CCP1,CCPxA);  
	CCP_DisableReverseOutput(CCP1,CCPxB); 

	/*
	(5)设置CCP 中断`
	*/	
	CCP_EnableOverflowInt(CCP0);			/*开启CCP0溢出中断*/
	CCP_EnableOverflowInt(CCP1);			/*开启CCP1溢出中断*/	
	NVIC_EnableIRQ(CCP_IRQn);
	/*
	(6)设置CCP 中断优先级
	*/	
	NVIC_SetPriority(CCP_IRQn,3);					/*优先级0~3， 0最高、3最低*/	

	/*
	(7)设置CCP IO口复用
	*/	
	GPIO_PinAFOutConfig(P26CFG,IO_OUTCFG_P26_CCP0A_O);
	GPIO_PinAFOutConfig(P04CFG,IO_OUTCFG_P04_CCP0B_O);
	GPIO_PinAFOutConfig(P23CFG,IO_OUTCFG_P23_CCP1A_O);
GPIO_PinAFOutConfig(P24CFG,IO_OUTCFG_P24_CCP1B_O);

	GPIO_Init(PORT2,PIN6,OUTPUT);
	GPIO_Init(PORT0,PIN4,OUTPUT);
	GPIO_Init(PORT2,PIN3,OUTPUT);
GPIO_Init(PORT2,PIN4,OUTPUT);
	/*
	(8)开启CCP模块
	*/	
	CCP_EnableRun(CCP0);			
	CCP_EnableRun(CCP1);		
	CCP_Start(CCP0);			/*CCP0模块使能*/
	CCP_Start(CCP1);			/*CCP1模块使能*/			

}


