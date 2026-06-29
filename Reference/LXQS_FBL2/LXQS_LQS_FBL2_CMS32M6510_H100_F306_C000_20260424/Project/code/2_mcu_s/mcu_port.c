
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    main.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   本文件为MCU接口函数函数
						包括但不限于：EPWM寄存器更新、硬件除法器调用
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

//---------------------------------------------------------------------------/
//	Local variable  definitions
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Global variable definitions(declared in header file with 'extern')
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Local function prototypes('static')
//---------------------------------------------------------------------------/
 
 
 
/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  :User_CapMode2_Handle
* Description    :调速信号捕获完成标志
* Function Call  :主函数1ms调用
* Input Paragram :无
* Return Value   :无
*-----------------------------------------------------------------------------
******************************************************************************/
void User_CapMode2_Handle(void)
{
	if(((CCP->RIS & (0x1UL<<(CAP3+8)))? 1:0) && ((CCP->RIS & (0x1UL<<(CAP2+8)))? 1:0))			//捕获3与捕获2完成 
	{
		Stru_Capture.OverFlowFlag = 0;
		Stru_Capture.OverFlowTime = 0;
		
		//清除高电平捕获	
		CCP->ICLR |= (0x1UL<< (CAP3+8));
		CCP->ICLR |= (0x1UL<< (CAP2+8));
		
		//置捕获完成标志;
		Stru_Capture.CompleteFlag = 1;			
	}
}

/*****************************************************************************
* Function Name  : User_PWM_Capture
* Description    : PWm信号捕获
* Function Call  : 主函数10ms调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Lsy
******************************************************************************/
void User_PWM_Capture(void)
{
	//300ms溢出判断
	if(++ Stru_Capture.OverFlowTime >= 30)	
	{
		Stru_Capture.OverFlowTime = 0;
		Stru_Capture.OverFlowFlag	= 1;
		Stru_Capture.HighTime 		= 0;
		Stru_Capture.PeriodTime			= 0;
		
	}
	
	if(Stru_Capture.OverFlowFlag)
	{
		if (PORT_GetBit(CCP_PWM_PORT,CCP_PWM_PIN))				//判断占空比是否为100%
		{
			if (++Stru_Capture.IO_HighCnt >= 8)	//80ms计数消抖
			{		
				Stru_Capture.IO_HighCnt = 0;
				Stru_Capture.Duty = 1000;			
				Stru_Capture.Fre = 0;				
			}	
			Stru_Capture.IO_LowCnt = 0;
		}
		else
		{
			if (++Stru_Capture.IO_LowCnt >= 8)		//80ms计数消抖
			{
				Stru_Capture.IO_LowCnt = 0;
				Stru_Capture.Duty = 0;			
				Stru_Capture.Fre = 0;						
			}
			Stru_Capture.IO_HighCnt = 0;
		}
	}
	
	else if(Stru_Capture.CompleteFlag)
	{
		Stru_Capture.IO_HighCnt = 0;
		Stru_Capture.IO_LowCnt	= 0;
		
		Stru_Capture.HighTime = CCP->CAP0DAT0 & 0xffff;
		Stru_Capture.PeriodTime 	= CCP->CAP0DAT0 >> 16;
		
		#if (Speed_Govern_Mode == CLK_Control)
				Stru_Capture.FreTemp = (int32_t)(1000000*10/Stru_Capture.PeriodTime);				//0.1Hz	1/(计数值*CCP时钟周期)		
		#else
			Stru_Capture.FreTemp = (int32_t)(16000000*10/Stru_Capture.PeriodTime);				//0.1Hz	1/(计数值*CCP时钟周期)			
		#endif				
		
		Stru_Capture.DutyTemp = (int16_t)(Stru_Capture.HighTime*1000/Stru_Capture.PeriodTime);	//分辨率 0.1%		

		// 更新值
		if(ABSFUN(Stru_Capture.Fre - Stru_Capture.FreTemp) > 5)		
			Stru_Capture.Fre = Stru_Capture.FreTemp;
		if(ABSFUN(Stru_Capture.Duty - Stru_Capture.DutyTemp) > 5)		
			Stru_Capture.Duty = Stru_Capture.DutyTemp;
		
		
		Stru_Capture.CompleteFlag = 0;
	}
	else
	{
		Stru_Capture.HighTime = CCP->CAP0DAT0 & 0xffff;
		Stru_Capture.PeriodTime 	= CCP->CAP0DAT0 >> 16;
	}
}

/*****************************************************************************
* Function Name  : User_Speed_Out
* Description    : 用户FG反馈
* Function Call  : 主函数10ms调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Lsy
******************************************************************************/
void User_Speed_Out(int32_t SpeedFG)
{ 
	int32_t FgCnt;
	volatile uint32_t temp;
	// 大于反馈值，最低有效反馈值,最低15.6Hz
	if(SpeedFG > 100)
	{
		// 计算周期Cnt
		FgCnt = 60000000 / (MOTOR_PAIRS * SpeedFG);
		
		if(FgCnt > 65535)				FgCnt = 65535;
		
		//-------------------------------------------------------------------//				
		// 配置寄存器
		CCP->LOCK = CCP_P1AB_WRITE_KEY;
		#if (CCP_PWM_MODULE == CCP0)
		{
			// 设置自动重装载、重装载值
			CCP->LOAD0 = ((CCP_RELOAD_CCPLOAD<< CCP_CCPLOAD0_RELOAD_Pos) | FgCnt);	
			// 设置比较值
			#if (CCPxA == CCP_PWM_CH)
			{
				temp = (CCP->D0A & 0xFFFF0000) |(FgCnt >> 1) ;
				CCP->D0A = temp;				
			}
			#else
			{
				temp = (CCP->D0B & 0xFFFF0000) |(FgCnt >> 1) ;
				CCP->D0B = temp;		
			}
			#endif
			
			// 使能CCP0模块
			CCP->CON0 |= (CCP_CCPCON0_CCP0EN_Msk);				
		}
		#else
		{
			// 设置自动重装载、重装载值
			CCP->LOAD1 = ((CCP_RELOAD_CCPLOAD<< CCP_CCPLOAD1_RELOAD_Pos) | FgCnt);	
			
			// 设置比较值
			#if (CCPxA == CCP_PWM_CH)
			{
				temp = (CCP->D1A & 0xFFFF0000) |(FgCnt >> 1) ;
				CCP->D1A = temp;				
			}
			#else
			{
				temp = (CCP->D1B & 0xFFFF0000) |(FgCnt >> 1) ;
				CCP->D1B = temp;		
			}
			#endif
			// 使能CCP1模块
			CCP->CON1 |= (CCP_CCPCON1_CCP1EN_Msk);
		
		}
		#endif
		
		// 运行CCP0模块
		CCP->RUN |= (0x1 << CCP_PWM_MODULE); 	
		CCP->LOCK = 0x00;		
		
		//-------------------------------------------------------------------//				
		//设置IO
		GPIO_PinAFOutConfig(P04CFG, IO_OUTCFG_P04_CCP0B_O);			
		GPIO_Init(PORT0,PIN4,OUTPUT);	
	
	}
	else
	{
		GPIO_PinAFOutConfig(P04CFG, IO_OUTCFG_P04_GPIO);	
		GPIO_Init(PORT0,PIN4,OUTPUT);	
		if(Fault_Flag)			PORT_SetBit(PORT0,PIN4);
		else								PORT_ClrBit(PORT0,PIN4);
	}
}

/*****************************************************************************
 ** \brief	串口数据
 **			
 ** \param 
 ** \return  12 bits Value
 ** \note	
*****************************************************************************/

void EPWM_ResetFaultBrake(void)
{
	EPWM_ClearBrakeIntFlag(); 		/*清除刹车中断标志位*/		
	EPWM_ClearBrake();	
	EPWM_Start(EPWM_CH_0_MSK | EPWM_CH_1_MSK|EPWM_CH_2_MSK | EPWM_CH_3_MSK| EPWM_CH_4_MSK | EPWM_CH_5_MSK);
//	EPWM_GetBrakeFlag();
}


/*****************************************************************************
 ** \brief	串口数据
 **			
 ** \param 
 ** \return  12 bits Value
 ** \note	
*****************************************************************************/
uint16_t g_UartSum_u16 = 0;
uint8_t  g_UartScopeArr[10] = {0};
void UART_View(int16_t view1,int16_t view2,int16_t view3,int16_t view4)
{
	#if (0)
	sum += (uint8_t)(view1);
	sum += (uint8_t)(view1>>8);
	sum += (uint8_t)(view2);
	sum += (uint8_t)(view2>>8);
	sum += (uint8_t)(view3);
	sum += (uint8_t)(view3>>8);
	sum += (uint8_t)(view4);
	sum += (uint8_t)(view4>>8);
	
	UART0->THR = (uint8_t)(view1);
	UART0->THR = (uint8_t)(view1>>8);
	UART0->THR = (uint8_t)(view2);
	UART0->THR = (uint8_t)(view2>>8);
	UART0->THR = (uint8_t)(view3);
	UART0->THR = (uint8_t)(view3>>8);
	UART0->THR = (uint8_t)(view4);
	UART0->THR = (uint8_t)(view4>>8);
	UART0->THR = sum;		
	#else
	
	static uint8_t  l_UartScopeCnt_u8 = 0;
	
	if(l_UartScopeCnt_u8 < 8)
    g_UartSum_u16 += g_UartScopeArr[l_UartScopeCnt_u8];	
	else if(l_UartScopeCnt_u8 == 8)
    g_UartScopeArr[8] = (uint8_t)g_UartSum_u16;
  
	UART0->THR = g_UartScopeArr[l_UartScopeCnt_u8];  
		
	if(++l_UartScopeCnt_u8 >= 9)
	{					
		l_UartScopeCnt_u8 = 0;	
		g_UartSum_u16 = 0;
		g_UartScopeArr[0] = (uint8_t)(view1);  g_UartScopeArr[1] = (uint8_t)(view1>>8);	
		g_UartScopeArr[2] = (uint8_t)(view2);  g_UartScopeArr[3] = (uint8_t)(view2>>8);	
		g_UartScopeArr[4] = (uint8_t)(view3);  g_UartScopeArr[5] = (uint8_t)(view3>>8);	
		g_UartScopeArr[6] = (uint8_t)(view4);  g_UartScopeArr[7] = (uint8_t)(view4>>8);	 	
	}
	
	
	#endif	
}

/*****************************************************************************
 ** \brief	软件触发AD采样并获取ADC转换结果
 **			
 ** \param [in] Channel: ADC_CH_0 ~ ADC_CH_30
 ** \return  12 bits Value
 ** \note	
*****************************************************************************/
uint32_t Soft_Trig_ADC(int32_t channelNum)  
{
	uint32_t val;
	uint32_t channel_back=ADC->SCAN;
	uint32_t i = 0;
	if(!ADC_IS_BUSY())
	{	
			ADC->LOCK = ADC_LOCK_WRITE_KEY;
			ADC->SCAN = 1<<channelNum;
			ADC->LOCK = 0;
			
			ADC_Go();
			while(ADC_IS_BUSY()&&(i<10000))
			{
				i++;
			}
			val = ADC_GetResult(channelNum);		
	}
	ADC->LOCK = ADC_LOCK_WRITE_KEY;
	ADC->SCAN=channel_back;
	ADC->LOCK=0;
	return (val);  
}

/*****************************************************************************
 ** \brief	硬件开方
**			x:被开方数
 ** \param 
 ** \return 
 ** \note	
*****************************************************************************/
int32_t HWSqrt(int32_t x)
{
	DIVSQRT->CON |= DIVSQRT_CON_MODE_Msk;   //使能开方功能
    
  DIVSQRT->ALUB = x;
	while(!DIVSQRT_IS_IDLE())
	{
		;
	}	

  return (0xFFFF & (DIVSQRT->RES0)); 
}



/*****************************************************************************
 ** \brief	更新EPWM寄存器
**			
 ** \param 
 ** \return 
 ** \note	  348byte
*****************************************************************************/
void SetEPWMRegister()
{
	#if (Config_Shunt_Mode == Double_Shunt)
  {
		EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;			
		if(Flag_MotorDir == DIR_CW)
		{	
			EPWM->CMPDAT[0] = Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tu * Stru_SVPWM.EPWM_Period >> 15); 
			EPWM->CMPDAT[2] = Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tv * Stru_SVPWM.EPWM_Period >> 15);
		}	
		else
		{
			EPWM->CMPDAT[2] = Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tu * Stru_SVPWM.EPWM_Period>>15); 
			EPWM->CMPDAT[0] = Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tv * Stru_SVPWM.EPWM_Period>>15);
		}
		EPWM->CMPDAT[4] = Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tw * Stru_SVPWM.EPWM_Period >> 15);
		EPWM->CON3 |= 0x00001500;		
		EPWM->LOCK = 0x0;	
  }
  #else 
  {
		// 计算寄存器值

		Stru_SVPWM.CntU_up	=	Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tu_up * Stru_SVPWM.EPWM_Period >> 15);
		Stru_SVPWM.CntU_dn	=	Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tu_dn * Stru_SVPWM.EPWM_Period >> 15);		
		Stru_SVPWM.CntV_up	=	Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tv_up * Stru_SVPWM.EPWM_Period >> 15);		
		Stru_SVPWM.CntV_dn	=	Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tv_dn * Stru_SVPWM.EPWM_Period >> 15);
		Stru_SVPWM.CntW_up	=	Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tw_up * Stru_SVPWM.EPWM_Period >> 15);
		Stru_SVPWM.CntW_dn	=	Stru_SVPWM.EPWM_Period - (Stru_SVPWM.Tw_dn * Stru_SVPWM.EPWM_Period >> 15);		
		
		Stru_SVPWM.Cnt_TG1st = Stru_SVPWM.EPWM_Period - (Stru_SVPWM.T_TG1st * Stru_SVPWM.EPWM_Period >> 15);		
		Stru_SVPWM.Cnt_TG2nd = Stru_SVPWM.EPWM_Period - (Stru_SVPWM.T_TG2nd * Stru_SVPWM.EPWM_Period >> 15);	

		EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;				
		// 更新发波寄存器
		if(Flag_MotorDir ==DIR_CW)
		{	
			EPWM->CMPDAT[0] = Stru_SVPWM.CntU_up | (Stru_SVPWM.CntU_dn << 16);
			EPWM->CMPDAT[2] = Stru_SVPWM.CntV_up | (Stru_SVPWM.CntV_dn << 16);
		}	
		else
		{
			EPWM->CMPDAT[2] = Stru_SVPWM.CntU_up | (Stru_SVPWM.CntU_dn << 16);
			EPWM->CMPDAT[0] = Stru_SVPWM.CntV_up | (Stru_SVPWM.CntV_dn << 16);
		}
		EPWM->CMPDAT[4] = Stru_SVPWM.CntW_up | (Stru_SVPWM.CntW_dn << 16);
		EPWM->CON3 |= 0x00001500;		
		EPWM->LOCK = 0x0;		
		
		//更新触发比较器的触发点
		SetTrigerPoint(Stru_SVPWM.Cnt_TG1st,Stru_SVPWM.Cnt_TG2nd); 
  }
	#endif
}

/*****************************************************************************
 ** \brief	设置相占空比
**			    dutyA: A相占空比
						dutyB: B相占空比
						dutyC: C相占空比
 ** \param 
 ** \return 
 ** \note	  348byte
*****************************************************************************/
void SetEPWMDuty(int32_t dutyA,int32_t dutyB,int32_t dutyC)
{
		int32_t compa,compb,compc;
		compa = (Stru_Foc.EPWM_Period * (32768 - dutyA) >> 15);
		compb = (Stru_Foc.EPWM_Period * (32768 - dutyB) >> 15);
		compc = (Stru_Foc.EPWM_Period * (32768 - dutyC) >> 15);

		EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;		
		if(Stru_Para.Mode.Shunt == Double_Shunt)
		{
			if(Stru_Para.Mode.Dir == DIR_CW)
			{	EPWM->CMPDAT[0] = compa;		EPWM->CMPDAT[2] = compb;		EPWM->CMPDAT[4] = compc;}	
			else
			{	EPWM->CMPDAT[2] = compa;		EPWM->CMPDAT[0] = compb;		EPWM->CMPDAT[4] = compc;	}
		}
		else 
		{
			if(Stru_Para.Mode.Dir ==DIR_CW)
			{	
				EPWM->CMPDAT[0] = (compa | (compa<<16));
				EPWM->CMPDAT[2] = (compb | (compb<<16));
				EPWM->CMPDAT[4] = (compc | (compc<<16));
			}	
			else
			{
				EPWM->CMPDAT[2] = (compa | (compa << 16));
				EPWM->CMPDAT[0] = (compb | (compb << 16));
				EPWM->CMPDAT[4] = (compc | (compc << 16));
			}
		}
		EPWM->CON3 |= 0x00001500;		/*使能加载周期&&占空比*/
		EPWM->LOCK = 0x0;		
}

/*****************************************************************************
* Function Name  : ADC_IPD_SoftSAMP
* Description    : IPD电流软件采样
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/09/18    新建			RMY
******************************************************************************/
int16_t ADC_IPD_SoftSAMP(void)
{
  ADC->LOCK = ADC_LOCK_WRITE_KEY;
	ADC->SCAN = ADC_SCAN_IPD;
	ADC->CON2 |= ADC_CON2_ADCST_Msk;
  ADC->LOCK = 0x00;

	while(ADC_IS_BUSY())
	{
		;
	}
  
  return (ADC->DATA[ADC_DATA_IPD]);
}
/*****************************************************************************
* Function Name  : MC_PWM_Mask
* Description    : 掩码输出设置
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/09/18    新建			RMY
******************************************************************************/
void MC_PWM_Mask(uint16_t ucDriver)       
{ 
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY; 	
	EPWM->MASK = ucDriver; 
	EPWM->LOCK = 0x0; 
}

/*****************************************************************************
* Function Name  : MC_PWM_Mask
* Description    : IPD软件模式adc初始化
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/09/18    新建			RMY
******************************************************************************/
void ADC_IPD_CONFIG(void)
{
  ADC_DisableHardwareTrigger(ADC_TG_EPWM_CMP0);
	ADC_DisableHardwareTrigger(ADC_TG_EPWM_CMP1);
	
	
	#if(Config_Shunt_Mode==Single_Shunt)
	{
		ADC_DisableHardwareTrigger(ADC_TG_EPWM0_ZERO);	
		while(ADC_IS_BUSY());
	}
	#else
	{
		ADC_DisableHardwareTrigger(ADC_TG_EPWM0_PERIOD);	
		while(ADC_IS_BUSY());
	}
	#endif

  
	__DI;
}

/*****************************************************************************
* Function Name  : ADC_IPD_REV
* Description    : IPD软件模式adc配置恢复
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/09/18    新建			RMY
******************************************************************************/
void ADC_IPD_REV(void)
{
	ADC_ConfigRunMode(ADC_MODE_HIGH,ADC_CONVERT_CONTINUOUS,ADC_CLK_DIV_1,25);		/*高速模式+连续模式+1分频+10.5 ADCClk采样保持时间*/
	ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);
	
	ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP0);
	ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP1);	

	#if(Config_Shunt_Mode==Single_Shunt)
	{
		ADC_EnableHardwareTrigger(ADC_TG_EPWM0_ZERO);	
	}
	#else
	{
		ADC_EnableHardwareTrigger(ADC_TG_EPWM0_PERIOD);	
	}
	#endif
	__EI;	
}


/*****************************************************************************
 ** \brief	EPWM_CompareTriger_Reset
 **			
 ** \param [in] none
 ** \return  none
 ** \note	
*****************************************************************************/
void EPWM_CompareTriger_Reset(void)
{
  int16_t T_CPMTG0;
  /*
	(7)设置EPWM比较器0
	*/			
	T_CPMTG0 = EPWM_CPMTG * EPWM_Tus;
	if(T_CPMTG0 < 12)	T_CPMTG0 = 12; 	//触发计数器不允许为0
	if(T_CPMTG0 > 128)	T_CPMTG0 = 128; 
	EPWM_ConfigCompareTriger(EPWM_CMPTG_0,EPWM_CMPTG_FALLING,EPWM_CMPTG_EPWM0,T_CPMTG0);
	EPWM_ConfigCompareTriger(EPWM_CMPTG_1,EPWM_CMPTG_RISING,EPWM_CMPTG_EPWM0,EPWM_HALFPERIOD);
}


/*****************************************************************************
* Function Name  : DutyBrake
* Description    : 占空比刹车
* Function Call  : 
* Input Paragram : 占空比0-32768
* Return Value   : none
* note           : 
* Version        : V0.1    2024/09/19    新建				Lsy
******************************************************************************/
void DutyBrake(int32_t Duty)
{
	uint32_t DutyCnt;
	
	DutyCnt = 0xffff & ((Stru_Foc.EPWM_Period * Duty) >> 15);
	
	EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
	//强制关上管
	EPWM->MASK = 0x00001500;
	//下管赋值
	
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		EPWM->CMPDAT[0] = DutyCnt;
		EPWM->CMPDAT[2] = DutyCnt;
		EPWM->CMPDAT[4] = DutyCnt;	
	}
	#else
	{
		EPWM->CMPDAT[0] = DutyCnt | (DutyCnt << 16);
		EPWM->CMPDAT[2] = DutyCnt | (DutyCnt << 16);
		EPWM->CMPDAT[4] = DutyCnt | (DutyCnt << 16);	
	
	}
	#endif
	//加载使能
	EPWM->CON3 |= 0x00001500;
	EPWM->LOCK = 0x0;
}

/*****************************************************************************
* Function Name  : Flash_Write_Int32
* Description    : 写Flash
* Function Call  :   addr -> 地址；value -> 写入的值 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : 
******************************************************************************/
void Flash_Write_Int32(uint32_t* addr , int32_t value)
{
	FMC->FLPROT = 0xF1;
	__DI;
	FMC->FLOPMD1 = 0xAA;
	FMC->FLOPMD2 = 0x55;  
	*addr = value; 
	__EI;			
	// polling OVER Flag
	while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0);
	FMC->FLSTS = FMC_FLSTS_OVF_Msk;
	FMC->FLPROT = 0xF0;
}





/******************************** END OF FILE *******************************/
