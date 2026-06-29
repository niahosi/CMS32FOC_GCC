
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    main.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/
//==========================================================================//


//---------------------------------------------------------------------------/
//	include files
//---------------------------------------------------------------------------/
#include "Header_Motor.h"
#include "Header_MCU.h"
#include "Header_User.h"
//---------------------------------------------------------------------------/
//	Local pre-processor symbols/macros('#define')
//---------------------------------------------------------------------------/
#define 		UH															(0x00)
#define 		UL															(0x01)
#define 		VH															(0x02)
#define 		VL															(0x03)
#define 		WH															(0x04)
#define 		WL															(0x05)

#define REG_POREMAP_VALUE   ( ((uint32_t)0xAA     << 24)		\
														| ((uint32_t)IO_EPWM5 << 20)		\
														| ((uint32_t)IO_EPWM4 << 16)		\
														| ((uint32_t)IO_EPWM3 << 12)		\
														| ((uint32_t)IO_EPWM2 << 8 )		\
														| ((uint32_t)IO_EPWM1 << 4 )		\
														| ((uint32_t)IO_EPWM0 << 0 )		)
//---------------------------------------------------------------------------/
//	Local variable  definitions
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Global variable definitions(declared in header file with 'extern')
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Local function prototypes('static')
//---------------------------------------------------------------------------/

//#include "mcu_init.h"


//#include "Set_Hardware.h"



/*****************************************************************************
* Function Name  : Delay_us
* Description    : 64Mhz运行频率下1us延时
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/09/18    新建			RMY
******************************************************************************/
void Delay_us(int32_t m)
{
	uint16_t i,j;
	for(i = 0 ; i < m ; i++)
		for(j = 0 ; j < 6 ; j++);
}

/*****************************************************************************
 ** \brief	__EI
 ** 使能中断
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void Enable_INT(void)
{
	/* Enable EPWM_INT */	
	#if (Config_Shunt_Mode == Single_Shunt)
	{
		EPWM->ICLR |= 0xFFFFFFFF;
		EPWM_EnableZeroInt(EPWM_CH_0_MSK);	
		NVIC_EnableIRQ(EPWM_IRQn);
		NVIC_SetPriority(EPWM_IRQn,1);	
	}
	#endif	

	/* Enable ADC1_INT */	
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		ADC_EnableChannelInt(ADC_INTER_CH);
		NVIC_EnableIRQ(ADC_IRQn);
		NVIC_SetPriority(ADC_IRQn,1);
	}
	#endif
	
	/* Enable SysTick_INT */	
	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn,3);
	
	/* Enable ACMP_INT */
	NVIC_EnableIRQ(ACMP_IRQn);
	NVIC_SetPriority(ACMP_IRQn,0);
	#if ((ACMP_CH == ACMP_CH1) || (ACMP_CH == ACMP_CH01) )
	ACMP_EnableInt(ACMP1);		//使能中断
	ACMP_ClearIntFlag(ACMP1);	//清除中断
	#endif
	#if ((ACMP_CH == ACMP_CH0) || (ACMP_CH == ACMP_CH01) )
	ACMP_EnableInt(ACMP0);	    //使能中断
	ACMP_ClearIntFlag(ACMP0);   //清除中断
	#endif

	/*
	TMR_EnableOverflowInt(TMR1);
	TMR_ClearOverflowIntFlag(TMR0);
	NVIC_EnableIRQ(TMR1_IRQn);	
	NVIC_SetPriority(TMR1_IRQn,2);	
	*/
	
	// 打开全局中断
	__enable_irq();
}

/*****************************************************************************
 ** \brief	DI
 **	禁止中断		
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void __DI1(void)
{
	NVIC_DisableIRQ(EPWM_IRQn);
	NVIC_DisableIRQ(ADC_IRQn);	
	NVIC_DisableIRQ(SysTick_IRQn);	
	NVIC_DisableIRQ(UART0_IRQn);	
	NVIC_DisableIRQ(UART0_IRQn);
	NVIC_DisableIRQ(TIMER0_IRQn);	
	NVIC_DisableIRQ(TIMER1_IRQn);
	NVIC_DisableIRQ(LVI_IRQn);		
	NVIC_DisableIRQ(ACMP_IRQn);
	NVIC_DisableIRQ(CCP_IRQn);		
	NVIC_DisableIRQ(INTP0_IRQn);
	NVIC_DisableIRQ(INTP1_IRQn);
	NVIC_DisableIRQ(INTP2_IRQn);
	NVIC_DisableIRQ(INTP3_IRQn);

	
}


void LowPower_Mode(void)
{
	//关闭中断
	__DI1();
	SysTick->CTRL  = 0x00000005;
	//设置部分引脚为输入高阻态
//	GPIO_Init(PORT0,PIN0,ANALOG_INPUT);
	
	//深度睡眠模式
	SCB->SCR |= 0x04;
	__WFI();	
}

/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  :Normal_Mode(void)
* Description    :恢复中断功能
* Function Call  :唤醒中断函数调用
* Input Paragram :无
* Return Value   :无
*-----------------------------------------------------------------------------
******************************************************************************/
void Normal_Mode(void)
{
	//恢复中断
	Enable_INT();
	SysTick->CTRL  = 0x00000007;
	//恢复引脚功能
	
}



/*****************************************************************************
 ** \brief	
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void f_DelayTime(uint32_t delay)
{
   for(; delay > 0; delay--) 
	{
     __nop();
  }
}

/*****************************************************************************
 ** \brief	
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void DelayTime_ms(uint32_t delay)
{
	uint32_t i,j;
	
	for(i=0; i<delay; i++)
	{
		for(j=0; j<10000; j++) //1ms,64M
		{
			;
		}
	}
}


/*****************************************************************************
 ** \brief	void ADC1_ClearAllInt_Flag(void)
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void ADC1_ClearAllInt_Flag(void)
{
	ADC->LOCK = ADC_LOCK_WRITE_KEY;	
	ADC->ICLR |= 0xFFFFFFFF;
	ADC->LOCK = 0x00;
}
/*****************************************************************************
 ** \brief	ADC_CHANNEL_CONFIG
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void ADC_CHANNEL_CONFIG(void)
{
	GPIO_Init(PORT2,PIN2,ANALOG_INPUT); // 电压采样通道
	GPIO_Init(PORT0,PIN2,ANALOG_INPUT); // 母线电流采样通道	
	
	#if(Config_Run_Mode==RUN_MD_LHALL)
	{
		GPIO_Init(PORT2,PIN0,ANALOG_INPUT); // Hall 1采样通道	
		GPIO_Init(PORT2,PIN1,ANALOG_INPUT); // Hall 2采样通道	
	}
	#endif

	#if(Config_VOLTAGE_SAMPLE==VOLTAGE_SAMPLE_ENABLE)
	{
		GPIO_Init(PORT2,PIN0,ANALOG_INPUT); // U相电压采样
		GPIO_Init(PORT2,PIN1,ANALOG_INPUT); // V相电压采样
		GPIO_Init(PORT2,PIN2,ANALOG_INPUT); // W相电压采样
	}
	#endif
}
/*****************************************************************************
 ** \brief	ADC_TGSAMP_CONFIG
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void ADC_TGSAMP_CONFIG(void)
{
	/* 双电阻ADC1触发采样模式配置 */
	#if (Config_Shunt_Mode == Double_Shunt)
	{		
	 //相电流采样
		ADC_EnableEPWMCmp0TriggerChannel(ADC_SCAN_CHA | ADC_SCAN_CHB); 
    ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP0);

	 //母线电压和母线电流采样配置	
		ADC_EnableEPWMCmp1TriggerChannel(ADC_SCAN_VBUS); 
    ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP1);

//		ADC_EnableEPWMTriggerChannel(ADC_SCAN_IBUS);	
//		ADC_EnableHardwareTrigger(ADC_TG_EPWM0_PERIOD);
		
		
		#if(Config_VOLTAGE_SAMPLE==VOLTAGE_SAMPLE_ENABLE)
		{
			ADC_EnableEPWMTriggerChannel(ADC_SCAN_UBEMF|ADC_SCAN_VBEMF|ADC_SCAN_WBEMF);	
			ADC_EnableHardwareTrigger(ADC_TG_EPWM0_PERIOD);
		}
		#endif

		#if(Config_Run_Mode==RUN_MD_LHALL)
		{
			ADC_EnableEPWMTriggerChannel(ADC_SCAN_LHALLAlpha|ADC_SCAN_LHALLBeta);	
			ADC_EnableHardwareTrigger(ADC_TG_EPWM0_PERIOD);
		}
		#endif
	}
	/* 单电阻ADC1触发采样模式配置 */
	#else
	{	
		/* 计数比较器0/1触发转换通道配置 */
		ADC_EnableEPWMCmp0TriggerChannel(ADC_SCAN_CHA);
		ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP0);
	
		ADC_EnableEPWMCmp1TriggerChannel(ADC_SCAN_CHB);
		ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP1);
	
		ADC_EnableEPWMTriggerChannel(ADC_SCAN_VBUS|ADC_SCAN_IBUS);	
		ADC_EnableHardwareTrigger(ADC_TG_EPWM0_ZERO);
	
		#if(Config_VOLTAGE_SAMPLE==VOLTAGE_SAMPLE_ENABLE)
		{
			ADC_EnableEPWMTriggerChannel(ADC_SCAN_UBEMF|ADC_SCAN_VBEMF|ADC_SCAN_WBEMF);	
			ADC_EnableHardwareTrigger(ADC_TG_EPWM0_ZERO);
		}
		#endif

		#if(Config_Run_Mode==RUN_MD_LHALL)
		{
			ADC_EnableEPWMTriggerChannel(ADC_SCAN_LHALLAlpha|ADC_SCAN_LHALLBeta);	
			ADC_EnableHardwareTrigger(ADC_TG_EPWM0_ZERO);
		}
		#endif
	}
	#endif
	EPWM_DisableIntAccompanyWithLoad();	
}


/*****************************************************************************
 ** \brief	系统时钟初始化
 **			
 ** \param [in] none
 ** \return  none
 ** \note	64M主频
*****************************************************************************/
void SysClock_Init(void)
{
	SystemCoreClockUpdate();
}

/*****************************************************************************
 ** \brief	GPIO初始化
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void GPIO_Config(void)
{
//	GPIO_Init(PORT0,PIN3,INPUT);			GPIO_PinAFInConfig(P03CFG, IO_OUTCFG_P03_GPIO);
//	GPIO_Init(PORT0,PIN4,INPUT);			GPIO_PinAFInConfig(P04CFG, IO_OUTCFG_P04_GPIO);
//	GPIO_Init(PORT0,PIN5,INPUT);			GPIO_PinAFInConfig(P05CFG, IO_OUTCFG_P05_GPIO);
	
//	GPIO_Init(PORT0,PIN3,OUTPUT);			  GPIO_PinAFInConfig(P04CFG, IO_OUTCFG_P03_GPIO);
	
	GPIO_Init(PORT1,PIN6,OUTPUT);			 
	GPIO_PinAFInConfig(P16CFG, IO_OUTCFG_P16_GPIO);
	PORT_SetBit(PORT1,PIN6);
	
}

/*****************************************************************************
 ** \brief	EPWM模块初始化，双电阻方案
 **			
 ** \param [in] none
 ** \return  none
 ** \note	除了刹车和重映射之外，其余代码不允许改动
*****************************************************************************/
void EPWM_R2_Init(void)
{
	uint32_t T_CPMTG0;
	
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_EPWM,ENABLE);	/*开启EPWM时钟*/
	
	/* 
	(1)设置EPWM运行模式
	*/
	EPWM_ConfigRunMode(  EPWM_COUNT_UP_DOWN  | 				/*xx计数模式*/
						 EPWM_OCU_SYMMETRIC 	  |			/*对称计数模式*/
						 EPWM_WFG_COMPLEMENTARYK   |		/*互补模式*/
						 EPWM_OC_INDEPENDENT);				/*独立输出模式*/
	/*
	(2)设置EPWM时钟周期
	*/
	
	EPWM_ConfigChannelClk(EPWM0, EPWM_CLK_DIV_1);
	EPWM_ConfigChannelClk(EPWM2, EPWM_CLK_DIV_1);	
	EPWM_ConfigChannelClk(EPWM4, EPWM_CLK_DIV_1);	
	
	EPWM_ConfigChannelPeriod(EPWM0,  EPWM_PERIOD);
	EPWM_ConfigChannelPeriod(EPWM2,  EPWM_PERIOD);
	EPWM_ConfigChannelPeriod(EPWM4,  EPWM_PERIOD);
	EPWM_ConfigChannelSymDuty(EPWM0, EPWM_HALFPERIOD);	
	EPWM_ConfigChannelSymDuty(EPWM2, EPWM_HALFPERIOD);		
	EPWM_ConfigChannelSymDuty(EPWM4, EPWM_HALFPERIOD);
	
	/*
	(3)设置EPWM反向输出
	*/
	
	#if 0
	
	EPWM_EnableReverseOutput( EPWM_CH_0_MSK | EPWM_CH_1_MSK |EPWM_CH_2_MSK|
								EPWM_CH_3_MSK| EPWM_CH_4_MSK|EPWM_CH_5_MSK);

	#else
	
	EPWM_DisableReverseOutput( EPWM_CH_0_MSK | EPWM_CH_1_MSK |EPWM_CH_2_MSK|
							EPWM_CH_3_MSK| EPWM_CH_4_MSK|EPWM_CH_5_MSK);
	
	#endif
	
	
	/*
	(3)设置EPWM死区
	*/
	EPWM_EnableDeadZone(0x3F, (uint32_t)(EPWM_DT*EPWM_Tus));

	/*
	(4)设置EPWM加载方式
	*/
	EPWM_EnableAutoLoadMode(EPWM_CH_0_MSK |EPWM_CH_2_MSK|EPWM_CH_4_MSK);				/*自动加载*/
	
	/*
	(5)设置EPWM比较器0
	*/			
	T_CPMTG0 = (uint32_t)(EPWM_CPMTG*EPWM_Tus);
	if(T_CPMTG0 < 12)	T_CPMTG0 = 12; 	//触发计数器不允许为0
	if(T_CPMTG0 > 128)	T_CPMTG0 = 128; 
	EPWM_ConfigCompareTriger(EPWM_CMPTG_0,EPWM_CMPTG_FALLING,EPWM_CMPTG_EPWM0,T_CPMTG0);
//	EPWM_ConfigCompareTriger(EPWM_CMPTG_1,EPWM_CMPTG_RISING,EPWM_CMPTG_EPWM0,T_CPMTG1);
	EPWM_ConfigCompareTriger(EPWM_CMPTG_1,EPWM_CMPTG_RISING,EPWM_CMPTG_EPWM0,EPWM_HALFPERIOD);
	/*
	(6)设置中断
	*/

	
	/*
	(7)设置EPWM刹车
	*/
    #if (ACMP_CH == ACMP_CH0)
	EPWM_EnableFaultBrake(EPWM_BRK_ACMP0EE);				/*ACMP0刹车*/	
	#else
	EPWM_EnableFaultBrake(EPWM_BRK_ACMP1EE);				/*ACMP1刹车*/	
	#endif
	
	EPWM_ConfigFaultBrakeLevel(EPWM_CH_0_MSK | EPWM_CH_2_MSK |EPWM_CH_4_MSK,0);
	EPWM_ConfigFaultBrakeLevel(EPWM_CH_1_MSK | EPWM_CH_3_MSK |EPWM_CH_5_MSK,0);
	EPWM_AllBrakeEnable();
	
	EPWM_ConfigBrakeMode(EPWM_BRK_SUSPEND);
	
  Bridge_Output_Off();


	/*
	(8)设置IO口输出
	*/	

	GPIO_PinAFOutConfig(P10CFG, IO_OUTCFG_P10_EPWM0);				/*设置P10为EPWM0通道*/
	GPIO_PinAFOutConfig(P11CFG, IO_OUTCFG_P11_EPWM1);				/*设置P11为EPWM1通道*/
	GPIO_PinAFOutConfig(P12CFG, IO_OUTCFG_P12_EPWM2);				/*设置P12为EPWM2通道*/
	GPIO_PinAFOutConfig(P13CFG, IO_OUTCFG_P13_EPWM3);				/*设置P13为EPWM3通道*/
	GPIO_PinAFOutConfig(P14CFG, IO_OUTCFG_P14_EPWM4);				/*设置P14为EPWM4通道*/
	GPIO_PinAFOutConfig(P15CFG, IO_OUTCFG_P15_EPWM5);				/*设置P15为EPWM5通道*/

	GPIO_Init(PORT1,PIN0,OUTPUT);
	GPIO_Init(PORT1,PIN1,OUTPUT);
	GPIO_Init(PORT1,PIN2,OUTPUT);
	GPIO_Init(PORT1,PIN3,OUTPUT);
	GPIO_Init(PORT1,PIN4,OUTPUT);
	GPIO_Init(PORT1,PIN5,OUTPUT);

	
	EPWM_EnableOutput(EPWM_CH_0_MSK | EPWM_CH_1_MSK|
					  EPWM_CH_2_MSK | EPWM_CH_3_MSK|
					  EPWM_CH_4_MSK | EPWM_CH_5_MSK);
	

	/*
	(9)重映射,客户根据实际应用配置 
	*/	
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
  EPWM->POREMAP = (uint32_t)REG_POREMAP_VALUE;
	EPWM->LOCK = 0x0;
	
	/*
	(10)在zero点加载占空比
	*/	
	EPWM_ConfigLoadAndIntMode(EPWM0, EPWM_EACH_ZERO);			
	EPWM_ConfigLoadAndIntMode(EPWM2, EPWM_EACH_ZERO);
	EPWM_ConfigLoadAndIntMode(EPWM4, EPWM_EACH_ZERO);
	
	/*
	(11)开启EPWM
	*/		
	EPWM_Start(EPWM_CH_0_MSK | EPWM_CH_2_MSK | EPWM_CH_4_MSK);               
}



/*****************************************************************************
 ** \brief	EPWM模块初始化，单电阻方案
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/

void EPWM_R1_Init(void)
{//NA NOW
  /* 
	(1)设置EPWM时钟
	*/
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_EPWM,ENABLE);	/*开启EPWM时钟*/
	/* 
	(2)设置EPWM运行模式
	*/
	EPWM_ConfigRunMode(  EPWM_COUNT_UP_DOWN  | 				/*xx计数模式*/
						 EPWM_OCU_ASYMMETRIC 	  |			/*非对称计数模式*/
						 EPWM_WFG_COMPLEMENTARYK   |		/*互补模式*/
						 EPWM_OC_INDEPENDENT);				/*独立输出模式*/
	
	/*
	(3)设置EPWM时钟周期
	*/

	
	EPWM_ConfigChannelClk( EPWM0, EPWM_CLK_DIV_1);
	EPWM_ConfigChannelClk( EPWM2, EPWM_CLK_DIV_1);	
	EPWM_ConfigChannelClk( EPWM4, EPWM_CLK_DIV_1);	
	EPWM_ConfigChannelPeriod(EPWM0, EPWM_PERIOD);
	EPWM_ConfigChannelPeriod(EPWM2, EPWM_PERIOD);
	EPWM_ConfigChannelPeriod(EPWM4, EPWM_PERIOD);

	/*
	(4)设置EPWM反向输出
	*/
	EPWM_DisableReverseOutput( EPWM_CH_0_MSK | EPWM_CH_1_MSK |EPWM_CH_2_MSK|
								EPWM_CH_3_MSK| EPWM_CH_4_MSK|EPWM_CH_5_MSK);

	/*
	(5)设置EPWM死区
	*/
	EPWM_EnableDeadZone(0x3F, (uint32_t)(EPWM_DT*EPWM_Tus));

	/*
	(6)设置EPWM加载方式
	*/
	EPWM_EnableAutoLoadMode(EPWM_CH_0_MSK |EPWM_CH_2_MSK|EPWM_CH_4_MSK);				/*自动加载*/
	
	
	/*
	(7)设置EPWM比较器0
	*/			
	
	EPWM_ConfigCompareTriger(EPWM_CMPTG_0, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0, EPWM_HALFPERIOD>>1);
	EPWM_ConfigCompareTriger(EPWM_CMPTG_1, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0, EPWM_HALFPERIOD);


	/*
	(8)设置EPWM刹车
	*/
	#if (ACMP_CH == ACMP_CH0)
	EPWM_EnableFaultBrake(EPWM_BRK_ACMP0EE);				/*ACMP0刹车*/	
	#else
	EPWM_EnableFaultBrake(EPWM_BRK_ACMP1EE);				/*ACMP1刹车*/	
	#endif
	EPWM_ConfigFaultBrakeLevel(EPWM_CH_0_MSK | EPWM_CH_2_MSK |EPWM_CH_4_MSK,0);
	EPWM_ConfigFaultBrakeLevel(EPWM_CH_1_MSK | EPWM_CH_3_MSK |EPWM_CH_5_MSK,0);
	EPWM_AllBrakeEnable();
	
  EPWM_ConfigBrakeMode(EPWM_BRK_SUSPEND);
  
  Bridge_Output_Off();

	/*
	(9)设置IO口输出
	*/	
	GPIO_PinAFOutConfig(P10CFG, IO_OUTCFG_P10_EPWM0);				/*设置P10为EPWM0通道*/
	GPIO_PinAFOutConfig(P11CFG, IO_OUTCFG_P11_EPWM1);				/*设置P11为EPWM1通道*/
	GPIO_PinAFOutConfig(P12CFG, IO_OUTCFG_P12_EPWM2);				/*设置P12为EPWM2通道*/
	GPIO_PinAFOutConfig(P13CFG, IO_OUTCFG_P13_EPWM3);				/*设置P13为EPWM3通道*/
	GPIO_PinAFOutConfig(P14CFG, IO_OUTCFG_P14_EPWM4);				/*设置P14为EPWM4通道*/
	GPIO_PinAFOutConfig(P15CFG, IO_OUTCFG_P15_EPWM5);				/*设置P15为EPWM5通道*/

	GPIO_Init(PORT1,PIN0,OUTPUT);
	GPIO_Init(PORT1,PIN1,OUTPUT);
	GPIO_Init(PORT1,PIN2,OUTPUT);
	GPIO_Init(PORT1,PIN3,OUTPUT);
	GPIO_Init(PORT1,PIN4,OUTPUT);
	GPIO_Init(PORT1,PIN5,OUTPUT);
	
	EPWM_EnableOutput(EPWM_CH_0_MSK | EPWM_CH_1_MSK|
					  EPWM_CH_2_MSK | EPWM_CH_3_MSK|
					  EPWM_CH_4_MSK | EPWM_CH_5_MSK);
					  
	/*
	(10)重映射
	*/	
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
  EPWM->POREMAP = (uint32_t)REG_POREMAP_VALUE;
	EPWM->LOCK = 0x0;
	/*
	(11)在zero点加载占空比
	*/	
	EPWM_ConfigLoadAndIntMode(EPWM0, EPWM_EACH_ZERO);			
	EPWM_ConfigLoadAndIntMode(EPWM2, EPWM_EACH_ZERO);
	EPWM_ConfigLoadAndIntMode(EPWM4, EPWM_EACH_ZERO);	
	
	/*
	(11)开启EPWM
	*/					 
	EPWM_Start(EPWM_CH_0_MSK | EPWM_CH_2_MSK | EPWM_CH_4_MSK);
  
}

/********************************************************************************
 ** \brief	ADC1配置，双电阻方案
 **
 ** \param [in]  none
 **
 ** \note  高速ADC，分别采样两路相电流和一路母线电流（看需求）
 ******************************************************************************/
void ADC_Init(void)
{
	/*
	(1)设置ADC1时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCEN,ENABLE);

	ADC_ConfigRunMode(ADC_MODE_HIGH,ADC_CONVERT_CONTINUOUS,ADC_CLK_DIV_1,25);	/*连续模式+高速模式+1分频+12.5 ADCClk采样保持时间*/

	/*
	(2)设置ADC通道切换模式
	*/	
	ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);	/*硬件自动切换*/
  
	/*
	(3)设置ADC充电\放电功能
	*/
	ADC_DisableChargeAndDischarge();					/*关闭充电\放电功能*/
	
  /*
	(4)设置ADC参考源
	*/
    
	#if (CONFIG_LDO == ADCREF_VCC)                 //不使能ADCLDO时，ADC参考电压选择VCC
	{
		ADC_ConfigVREF(ADC_VREFP_VDD);		   
	}
	#else                                       //使能ADCLDO时，ADC参考电压选择ADCLDO的输出
	{
			ADC_ConfigVREF(ADC_VREFP_AVREFP);		
	}
	#endif
    
	/*
	(5)设置ADC1通道
	*/	
	ADC_CHANNEL_CONFIG();
	/*
	(6)开启ADC
	*/		
	ADC_Start();
	
}


/*****************************************************************************
 ** \brief	PGA0模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void  PGA0_Init(void)
{
 /*
	(1)设置PGA0时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_PGA0EN,ENABLE);
	 
  /*
	(2)设置PGA0正端、负端输入 IO口
	*/
	GPIO_Init(PORT0,PIN0,ANALOG_INPUT);  //PGA0 +
	GPIO_Init(PORT0,PIN1,ANALOG_INPUT);  //PGA0 -

  /*
	(3)设置PGA0增益
	*/
	PGA_ConfigGain(PGA0x,PGA_GAIN_10);		
  /*
	(4)设置PGA0参考电压
	*/
	PGA_VrefCtrl(PGA0x,PGA0BG);
  /*
	(5)设置PGA0模式
	*/
	PGA_ModeSet(PGA0x,PgaDiffer);
	 /*
	(6)设置PGA0输出
	*/
	#if 0
	//PGA0输出到PAD串联电阻选择  0: 内部不串电阻  1: 内部串10K电阻
	PGA0_ConfigResistorPAD(PGA_R_10K);
	
	//PPGA0输出到PAD通道使能 
	PGA_EnableOutput(PGA0x);	
	//使能引脚
	GPIO_Init(PORT0,PIN2,ANALOG_INPUT);  //PGA_OUT
	#endif
  /*
	(7)使能PGA0
	*/
	PGA_Start(PGA0x);	
}

/*****************************************************************************
 ** \brief	PGA1模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	注意使用PGA1,必须同时开启PGA0--PGA_Start(PGA1)
*****************************************************************************/
void PGA1_Init(void)
{
 /*
	(1)设置PGA1时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_PGA1EN,ENABLE);	
	  
  /*
	(2)设置PGA1正端、负端输入
	*/
	GPIO_Init(PORT2,PIN4,ANALOG_INPUT);  //PGA1 +
	GPIO_Init(PORT2,PIN5,ANALOG_INPUT);//PGA1 -
	 
  /*
	(3)设置PGA1增益
	*/
	PGA_ConfigGain(PGA1x,PGA_GAIN_10);
  /*
	(4)设置PGA1参考电压
	*/
	PGA_VrefCtrl(PGA1x,VrefHalf);
	/*
	(5)设置PGA1模式
	*/
	PGA_ModeSet(PGA1x,PgaDiffer);  		
  /*
	(6)设置PGA1输出
	*/
	#if (0)		
	//PGA1、2输出使能
	PGA_EnableOutput(PGA1x);

	GPIO_Init(PORT0,PIN4,ANALOG_INPUT);  //PGA_OUT

	#endif
	 
   /*
	(7)使能PGA1
	*/
	PGA_Start(PGA1x);
	
}

/*****************************************************************************
 ** \brief	PGA2模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	注意使用PGA1,必须同时开启PGA0--PGA_Start(PGA1)
*****************************************************************************/
void PGA2_Init(void)
{
 /*
	(1)设置PGA2时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_PGA2EN,ENABLE);	
	  
  /*
	(2)设置PGA2正端、负端输入
	*/
	GPIO_Init(PORT2,PIN6,ANALOG_INPUT);  //PGA2 +
	GPIO_Init(PORT2,PIN7,ANALOG_INPUT);//PGA2 -
	
  /*
	(3)设置PGA2增益
	*/
	PGA_ConfigGain(PGA2x,PGA_GAIN_10);
	
  /*
	(4)设置PGA2参考电压
	*/
	PGA_VrefCtrl(PGA2x,VrefHalf);
  	
  /*
	(5)设置PGA2模式
	*/
	PGA_ModeSet(PGA2x,PgaDiffer);  	
	  
  /*
	(6)设置PGA2输出
	*/
	#if 0
	//PGA1、2输出使能
	PGA_EnableOutput(PGA2x);

	GPIO_Init(PORT0,PIN4,ANALOG_INPUT);  //PGA_OUT
	
	#endif
  
  /*
	(7)使能PGA2
	*/
	//PGA2使能  
	PGA_Start(PGA2x);

}

/*****************************************************************************
 ** \brief	ACMP0模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	用于EPWM刹车保护
*****************************************************************************/
void ACMP0_Init(void)
{
 /*
	(0)设置ACMP0时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_ACMPEN,ENABLE);

	/*
	(1)配置ACMP IO口  //未使用则无需配置
	*/
 
  
  /*
	(2)设置ACMP0正端输入
	*/

  ACMP_ConfigPositive(ACMP0,ACMP_POSSEL_0PGA2O);	
 
  /*
	(3)设置ACMP0负端输入
	*/
	ACMP_ConfigNegative(ACMP0,ACMP_NEGSEL_DAC_O);		/*负端选择DAC_O*/
	 
  /*
	(4)设置ACMP0输出使能
	*/
 	ACMPOut_Enable(ACMP0);

  /*
	(5)设置ACMP0使能
	*/
	ACMP_Start(ACMP0);
	
  /*
	(6)设置ACMP0输出滤波
	*/
	ACMP_Filter_Config(ACMP0,ENABLE,ACMP_NGCLK_65_TSYS);

  /*
	(7)设置ACMP0输出极性
	*/
	ACMP_Polarity_Config(ACMP0,ACMP_POL_Pos);
	
  /*
	(8)设置ACMP0迟滞电压
	*/
	ACMP_EnableHYS(ACMP0,ACMP_HYS_POS,ACMP_HYS_S_10);

  /*
	(9)设置ACMP0 事件
	*/
	ACMP_ConfigEventAndIntMode(ACMP0,ACMP_EVENT_INT_RISING);	/*上升沿触发中断与事件，中断触发方式与事件触发方式共用*/		 
  /*
	(10)使能ACMP0 事件
	*/
	ACMP_EnableEventOut(ACMP0);


}


/*****************************************************************************
 ** \brief	ACMP1模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	用于EPWM刹车保护
*****************************************************************************/
void ACMP1_Init(void)
{
	/*
	(0)设置ACMP1时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_ACMPEN,ENABLE);
	
	/*
	(1)配置ACMP IO口  //未使用则无需配置
	*/

  
  /*
	(2)设置ACMP1正端输入 
	*/
  ACMP_ConfigPositive(ACMP1,ACMP_POSSEL_1PGA0O);		

  /*
	(3)设置ACMP1负端输入
	*/
	ACMP_ConfigNegative(ACMP1,ACMP_NEGSEL_DAC_O);		/*负端选择DAC_O*/
	
  /*
	(4)设置ACMP1输出使能
	*/
 	ACMPOut_Enable(ACMP1);
		
  /*
	(5)设置ACMP1使能
	*/
	ACMP_Start(ACMP1);
  
  /*
	(6)设置ACMP1输出滤波
	*/	
	ACMP_Filter_Config(ACMP1,ENABLE,ACMP_NGCLK_65_TSYS);
 
  /*
	(7)设置ACMP1输出极性
	*/
	ACMP_Polarity_Config(ACMP1,ACMP_POL_Pos);
	
  /*
	(8)设置ACMP1迟滞电压
	*/
	ACMP_EnableHYS(ACMP1,ACMP_HYS_POS,ACMP_HYS_S_00);
	
  /*
	(9)设置ACMP1 事件
	*/
	ACMP_ConfigEventAndIntMode(ACMP1,ACMP_EVENT_INT_RISING);
	
  /*
	(10)使能ACMP1 事件
	*/
	ACMP_EnableEventOut(ACMP1);

	
}
/*****************************************************************************
 ** \brief	DAC配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	无需改动
*****************************************************************************/
void DAC_Init(void)
{
	uint16_t	VthQ10;
	uint8_t bLdoOn,bForT;
	  
  /*
	(1)计算LDO输出电压
	*/
	//读取当前LDO设置输出电压
	bLdoOn = ((*(uint16_t*)0x40068340) >> 8) & 0x01;
	bForT = (*(uint16_t*)0x40068340) & 0xff;
	
//	HARDOVCUR_A

	if(!bLdoOn)					//未使能，5V
		  VthQ10 = (HARDOVCUR_PROTECT_VALUE * HW_RSHUNT_IBUS * HW_GAIN_IBUS + HW_OFFSET_IBUS) * 2550 / 50;		
	else
	{
		if(bForT == 0x55)		//4.2V
			VthQ10 = (HARDOVCUR_PROTECT_VALUE * HW_RSHUNT_IBUS * HW_GAIN_IBUS + HW_OFFSET_IBUS) * 2550 / 42;
			
		else					//3.6V
			VthQ10 = (HARDOVCUR_PROTECT_VALUE * HW_RSHUNT_IBUS * HW_GAIN_IBUS + HW_OFFSET_IBUS) * 2550 / 36;
	}
	
	//防止数据溢出
	if(VthQ10 > 255)	 
      VthQ10 = 255;

  /*
	(2)设置DAC时钟
	*/
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_DAC,ENABLE);
	
   /*
	(3)设置DAC输出
	*/
	DAC_ConfigInput(VthQ10);
	
  /*
	(4)设置DAC使能
	*/
	DAC_Start();

}





/*****************************************************************************
 ** \brief	硬件触发器配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	无需改动
*****************************************************************************/
void HWDIV_Init(void)
{
	CGC_PER12PeriphClockCmd(CGC_PER12Periph_DIV,ENABLE);
	DIVSQRT_EnableDIVMode();
	DIVSQRT_EnableSingedMode();
}

/*****************************************************************************
 ** \brief	UART模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	该配置主要用于调试，用户可配置为通信
*****************************************************************************/
void UART0_Init(void)
{

	uint32_t  BuadRate;
	
	BuadRate = 300000;//300000;
	
	/*
	(1)开启UARTx时钟
	*/
	CGC_PER12PeriphClockCmd(CGC_PER12Periph_UART,ENABLE);
	
	/*
	(2)设置UARTx模式
	*/	
	UART_ConfigRunMode(UART0, BuadRate, UART_WLS_8, UART_PARITY_NONE,UART_STOP_BIT_1);
	
	/*
	(3)开启UARTx输出
	*/			
	GPIO_PinAFOutConfig(P04CFG,IO_OUTCFG_P04_TXD);	
	GPIO_Init(PORT0,PIN4,OUTPUT);	
 
 
	/*
	(4)开启UARTx接收
	*/				
//	GPIO_PinAFInConfig(P03CFG,IO_INCFG_P03_RXD);
//  GPIO_Init(PORT0,PIN3,PULLUP_INPUT);	
//	GPIO_PinAFInConfig(UARTRXDCFG,IO_INCFG_P03_RXD);
	/*
	(5) 清除模块操作
	*/
	UART_Lock(UART0);
  
    
}

/*****************************************************************************
 ** \brief	SysTick_Init
 **			
 ** \param [in] none
 ** \return  none
 ** \note	无需改动
*****************************************************************************/
void SysTick_Init(void)
{
	SysTick->LOAD  = (uint32_t)(MCU_CLK/1000 - 1UL);
	SysTick->VAL   = 0UL;
	SysTick->CTRL  = 0x00000007;
}

/*****************************************************************************
 ** \brief	TMR0模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	用于hall平均角速度计算，不允许改动
*****************************************************************************/
void TMR0_Iint(void)
{
	/*
	(1)设置TMR 的时钟
	*/
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_TIMER01,ENABLE);/*打开Timer0的时钟*/
	TMR_ConfigClk(TMR0,TMR_CLK_DIV_16);		/*64MHz*/
	
	/*
	(2)设置TMR 运行模式
	*/	
	TMR_ConfigRunMode(TMR0,TMR_COUNT_PERIOD_MODE, TMR_BIT_32_MODE);
	TMR_DisableOneShotMode(TMR0);
	
	/*
	(3)设置TMR 运行周期
	*/		
	TMR_SetPeriod(TMR0,40000);			/* 4us*12500=50ms*/
	
	/*
	(4)设置TMR 中断
	*/	
	TMR_EnableOverflowInt(TMR0);

	/*
	(5)开启TMR
	*/	
	TMR_Start(TMR0);
	
}

/*****************************************************************************
 ** \brief	TMR1模块配置
 **			
 ** \param [in] none
 ** \return  none
 ** \note	参考配置
*****************************************************************************/
void TMR1_Init(void)
{
	/*
	(1)设置TMR 的时钟
	*/
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_TIMER01,ENABLE);/*打开Timer0的时钟*/
	TMR_ConfigClk(TMR1,TMR_CLK_DIV_1);		/*48MHz*/
	
	/*
	(2)设置TMR 运行模式
	*/	
	TMR_ConfigRunMode(TMR1,TMR_COUNT_PERIOD_MODE, TMR_BIT_16_MODE);
	TMR_DisableOneShotMode(TMR1);
	
	/*
	(3)设置TMR 运行周期
	*/		
	TMR_SetPeriod(TMR1,4000);			/*100us = 6400*/
	
	/*
	(4)设置TMR 中断
	*/	
	
	/*
	(5)开启TMR
	*/	
	TMR_Start(TMR1);	
}

/*****************************************************************************
 ** \brief	Flash写操作
 **			
 ** \param [in] none
 ** \return  none
 ** \note	无需改动
*****************************************************************************/
uint32_t WriteState;
void FlashWrite(uint32_t address, uint8_t datawrite)
{
	//接触保护
	FMC->FLPROT = 0xF1;
	//写入模式
	FMC ->FLOPMD1 = 0xaa;
	FMC ->FLOPMD2 = 0x55;
	
	*(uint8_t *)address = datawrite;
	
	while(!WriteState)
	{
		WriteState = FMC->FLSTS &0x01;;
	}
	//清除完成标志
	FMC->FLSTS |= 0x01;
	//关闭写状态
	FMC ->FLOPMD1 = 0x00;
	FMC ->FLOPMD2 = 0x00;
	FMC->FLPROT = 0x2e;
}




/*****************************************************************************
 ** \brief	ADCLDO_Init
 **			
 ** \param [in] none
 ** \return  none
 ** \note	无需改动
*****************************************************************************/
void ADCLDO_Init(void)
{	
	
//打开时钟配置寄存器
	CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCLDO,ENABLE);	
	
	#if (CONFIG_LDO == ADCREF_3V6)
	{
		//设置输出3.6V
		ADCLDO_OutVlotageSel(ADCLDO_OutV_3d6);
		//ADCLDO模块使能  
		ADCLDO_Enable();
	}
	#elif (CONFIG_LDO == ADCREF_4V2)
	{
		//设置输出4.2V
		ADCLDO_OutVlotageSel(ADCLDO_OutV_4d2);
		//ADCLDO模块使能  
		ADCLDO_Enable();
	}
	#elif (CONFIG_LDO == ADCREF_VCC)
	{
		//关闭LDO
		ADCLDO_Disable();
		//关闭时钟降低功耗
		CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCLDO,DISABLE);	
	}
	#endif
		
}


/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  :CCP捕获模式0配置
* Description    :需要将相应的IO配置成CCP口，其它不需要改动
* Function Call  :程序初始化调用
* Input Paragram :无
* Return Value   :无
*-----------------------------------------------------------------------------
******************************************************************************/

void CCP_Capture_Init(void)
{

    /*
  (1)开启CCP模块时钟
  */
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_CCP,ENABLE);				
	
	#if (Speed_Govern_Mode==CLK_Control)
		CCP_ConfigCLK(CCP_CAPTURE_MODULE,CCP_CLK_DIV_64,CCP_RELOAD_CCPLOAD,0xffff);			/*Fahb=64M*/	
	#else
		CCP_ConfigCLK(CCP_CAPTURE_MODULE,CCP_CLK_DIV_4 ,CCP_RELOAD_CCPLOAD,0xffff);			/*Fahb=64M*/				
	#endif
	/*
	(2)设置CCP运行模式
	*/
	CCP_EnableCAPMode2();								/* 触发模式2  */					
	CCP_ConfigCAPMode2(CAP1,CCP_CAP_MODE2_RISING);		/* 上升沿触发 */
	CCP_ConfigCAPMode2(CAP2,CCP_CAP_MODE2_FALLING);		/* 下降沿触发 */
	CCP_ConfigCAPMode2(CAP3,CCP_CAP_MODE2_RISING);		/* 上升沿触发 */	
  
	/*
	(3)设置CCP捕获通道
	*/
	CCP_SelCAPMode2Channel(CAP1_CCP0B);
	
	/*
	(4)设置捕获通道IO		
	*/
	GPIO_PinAFInConfig(CCP0BINCFG, IO_INCFG_P04_CCP0B_I);    /*设置P26为CCP1b捕获通道*/
	GPIO_Init(PORT0,PIN4,INPUT);	

	/*
	(5)开启CCP模块
	*/	
	CCP_EnableRun(CCP1);			
	CCP_Start(CCP1);			/*CCP模块使能*/

}
/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  :CCP PWM模块配置
* Description    :需要将相应的IO配置成CCP口，其它不需要改动
* Function Call  :程序初始化调用
* Input Paragram :无
* Return Value   :无
*-----------------------------------------------------------------------------
******************************************************************************/
void CCP_PWM_Init(void)
{
	/*
	(1)设置CCP的时钟
	*/
	CGC_PER11PeriphClockCmd(CGC_PER11Periph_CCP,ENABLE);			
	CCP_ConfigCLK(CCP_PWM_MODULE,CCP_CLK_DIV_64,CCP_RELOAD_CCPLOAD,10);		
	/*
	(2)设置CCP运行模式
	*/
	CCP_EnablePWMMode(CCP_PWM_MODULE);		
	/*
	(3)设置CCP 比较值
	*/	
	CCP_ConfigCompare(CCP_PWM_MODULE,CCP_PWM_CH,5); 	/*50%*/	
	/*
	(4)设置CCP 反相输出
	*/		
	CCP_DisableReverseOutput(CCP_PWM_MODULE,CCP_PWM_CH);
	/*
	(7)设置CCP IO口复用
	*/	
	
	GPIO_PinAFOutConfig(P04CFG, IO_OUTCFG_P04_CCP0B_O);				/*设置P07为CCP0A FG输出通道*/
	GPIO_Init(PORT0,PIN4,OUTPUT);
	
	/*
	(8)开启CCP模块
	*/		
}

/*****************************************************************************
 ** \brief	SPI_Transmit
 **			
** \param [in] SendData: 发送的值
 ** \return  16bit 获取的值
 ** \note	
*****************************************************************************/
uint32_t SPI_Transmit(uint32_t  Data)
{
	while(SSP_GetBusyFlag());			
	while(!SSP_GetTFEFlag());
	SSP_SendData(Data);
	while(!SSP_GetRNEFlag());
	return (SSP_GetData());			
}

/***************************************************************************
 ** \brief	 SPI_M95256_Read_SFR
 **			
 ** \param [in]  cmd:	
 ** \return 8bit Data
 ** \note
***************************************************************************/
uint32_t SPI_KTH7801_Read(uint32_t cmd)
{
	uint32_t temp = SPI_Transmit(cmd);
	return temp;	
}

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
	SSP_ConfigClk(7,4);					/*Fapb = 48Mhz,  sclk = 1Mhz*/								
	/*
	(2)设置SSP 为SPI模式
	*/							
	SSP_ConfigRunMode(SSP_FRAME_SPI,SSP_CPO_1,SSP_CPHA_1,SSP_DAT_LENGTH_16);	/*设置为SPI模式*/																		
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
	
	
	CGC->RSTM = 1;  //将P02用作普通引脚，而非外部复位引脚

	//将P24作为片选信号
	GPIO_PinAFOutConfig(P23CFG,IO_OUTCFG_P23_GPIO);
	GPIO_Init(PORT2,PIN3,OUTPUT);	

//	GPIO_PinAFOutConfig(P02CFG,IO_OUTCFG_P02_SSIO);
//	GPIO_Init(PORT0,PIN2,OUTPUT);	

	/*
	(5)开启SPI
	*/	
//	SPI_M95256_Stop();	
	SSP_Start();
	
}



/*****************************************************************************
 ** \brief	系统的初始化配置，保护模块初始化，变量参数初始化等
 **			
 ** \param [in] none
 ** \return  none
 ** \note		基本的模块初始化顺序不允许改动，允许增加其他模块功能
*****************************************************************************/
void System_Init(void)
{
	//系统时钟配置
	SysClock_Init();
	
	//取消P02复位功能			
	CGC->RSTM = 0x1;
	

	// 关闭全局中断
	__disable_irq();
	
	//调试时打开，避免上电后把仿真口刷新烧不了程序，正常模式改为300ms
	//单次延时不得超过看门狗复位时间
	DelayTime_ms(POWERON_DELAY_TIME);	
	WDT_Restart();

	//DBG->DBGSTOPCR |= (1<<24);   //禁用仿真功能
	
	
	//输入输出IO口线配置	
	GPIO_Config();

	//ADCLDO配置   ADC的基准电压
	ADCLDO_Init();
	
	//单电阻模式下PWM、ADC、PGA模块配置
	#if (Config_Shunt_Mode == Single_Shunt)	
		EPWM_R1_Init();
		ADC_Init();
		
		PGA0_Init();
	#endif

		//双电阻模式下PWM、ADC、PGA模块配置
	#if (Config_Shunt_Mode==Double_Shunt)
		EPWM_R2_Init();
		ADC_Init();
		
		PGA0_Init();
		PGA1_Init();
		PGA2_Init();
	#endif

	// 设置ACMP负端 参考电压，须在ACMP初始化之前，ADCLDO之后
	DAC_Init();

  #if ((ACMP_CH == ACMP_CH0) || (ACMP_CH == ACMP_CH01))

		ACMP0_Init();//比较器ACMP0模块配置
	
	#endif
	
  #if ((ACMP_CH == ACMP_CH1)|| (ACMP_CH == ACMP_CH01))
		#if (HARDOVCUR_PROTECT_ENABLE == 1)
			ACMP1_Init();//比较器ACMP1模块配置
		#endif
  #endif


	//调速捕捉模块配置
	#if (Config_CCP_Capture == CCP_Capture_Enable)
		CCP_Capture_Init();		
	#endif	

	//速度反馈--FG输出模块配置
	#if (Config_CCP_PWM == CCP_PWM_Enable)
		CCP_PWM_Init();
	#endif
	
	
	//SPI通信配置
	SPI_Master_Mode();

	//硬件除法器配置
	HWDIV_Init();
	
	//串口通信配置
	#if ((Config_Comm_Mode == Uart_Scope) || (Config_Comm_Mode == Uart_UI))
  UART0_Init();
  #endif
	
	//RTT配置
	#if (Config_Comm_Mode == Jlink_RTT)
  SEGGER_RTT_ConfigUpBuffer(1, "JScope_i2i2i2i2", bRttBuf, 1024, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  #endif

	//定时器TMR0模块配置
	//TMR0_Iint();
	//TMR1_Iint();

	//开启硬件触发ADC采样
  ADC_TGSAMP_CONFIG();	       

	//延时  等待母线电流滤波电容充电 
	DelayTime_ms(100);

	//SysTick初始化 
	SysTick_Init();

}


/******************************** END OF FILE *******************************/
