
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    interrupt.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   用于存放使用的中断服务函数，未使用中断函数之于isr.c中
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
extern struct {
    int16_t data0;
    int16_t data1;
    int16_t data2;
    int16_t data3;
    int16_t data4;
    int16_t data5;
} Rttstru;
//---------------------------------------------------------------------------/
//	Local function prototypes('static')
//---------------------------------------------------------------------------/
void FOC_User_Control(void);
void RTT_VIEW(int16_t view1,int16_t view2,int16_t view3,int16_t view4);

//===========================================================================/
//***** definitions  end ****************************************************/
//===========================================================================/


/*****************************************************************************
* Function Name  : EPWM_IRQHandler
* Description    : EPWM中断服务函数
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : R1:30us/64M(2~3us_week)(2us_comm)
* Version        : V0.1    2024/06/16    新建
******************************************************************************/

void EPWM_IRQHandler(void)
{
//	PORT_SetBit(PORT0,PIN3);
	//零点中断任务
	if(EPWM->MIS & EPWM_ZIFn_Flag)
	{
		//清除标志
		EPWM_ClearZIFn_Flag();
			
    FOC_Task_HighFre();
		
		Fault_Check_FOCTask();

		FOC_User_Control();
	}
	//清所有中断标志
	else
	{
		EPWM_ClearAllInt_Flag();
	}
//	PORT_ClrBit(PORT0,PIN3);
}

/*****************************************************************************
* Function Name  : ADC_IRQHandler
* Description    : ADC中断服务函数
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建
******************************************************************************/
void ADC_IRQHandler(void)
{	
//	PORT_SetBit(PORT0,PIN3);

  //ADC采样完成中断
	if(ADC->MIS & ADC_INTER_CH)
	{		
		//清除中断标志
		ADC_ClearIntFlag_CHA();
		
    FOC_Task_HighFre();
		
		Fault_Check_FOCTask();
		
		FOC_User_Control();
	}
	//清所有中断标志
	else
	{
		ADC_ClearAllInt_Flag();
  }	
//	PORT_ClrBit(PORT0,PIN3);
}

/*****************************************************************************
* Function Name  : ACMP_IRQHandler
* Description    : ACMP中断服务函数
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建
******************************************************************************/
void ACMP_IRQHandler(void)
{
	//清除中断标志 
	#if (ACMP_CH == ACMP_CH1)
	ACMP_ClearIntFlag(ACMP1);	
	#elif (ACMP_CH == ACMP_CH0)
	ACMP_ClearIntFlag(ACMP0);	
	#endif	
	
	// 关闭桥臂输出
	Bridge_Output_Off()
	// 用户层关机
	User_Motor_Off();	
	// 短路标志置位 
	Fault_ShortCircuit = 1;
	
	//系统状态保护
	SYSTEM_STATE		= SYS_FAULT;
	MOTOR_STATE			= MC_POWERON;
	

}

/*****************************************************************************
* Function Name  : SysTick_Handler
* Description    : SysTick中断服务函数
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 1ms
* Version        : V0.1    2024/06/16    新建
******************************************************************************/
void SysTick_Handler(void)
{
	//--------------------------------------------------------------------------/
	SysTick->CTRL|=SysTick_CTRL_COUNTFLAG_Msk;
	
	//--------------------------------------------------------------------------/
	// FOC中频任务处理
	FOC_Task_MidFre();
	
	//--------------------------------------------------------------------------/
	// 时序、标志处理
	Flag_1ms_Intr = 1;
	
	if(Stru_Time.PowerOn > 0)			Stru_Time.PowerOn --;
	
	//--------------------------------------------------------------------------/
	// 捕获信号处理
	#if (Config_CCP_Capture == CCP_Capture_Enable)	
	User_CapMode2_Handle();
	#endif	
	
}


/*****************************************************************************
* Function Name  : FOC_User_Control
* Description    : 用户高频任务处理
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建		RMY
******************************************************************************/
extern uint8_t LearnSuccessFlag;
extern int16_t  Angle;
extern int32_t SpeedRef;
void FOC_User_Control(void)
{
	//--------------------------------------------------------------------------/
	// 串口虚拟示波器，用时约2us
	#if (Config_Comm_Mode == Uart_Scope) 

	VSData[0] = MOTOR_STATE*1000;
	VSData[1] = Angle;
	VSData[2] = Stru_Foc.Elec_Theta;
	VSData[3] = Stru_Cur_abc.Ia;

//	VSData[0] = Stru_OutLoop.Ramp.Out;
//	VSData[1] = Stru_Meas.PU_Value.MecSpd;
//	VSData[2] = Stru_Cur_dqRef.Iq;
//	VSData[3] = Stru_Foc.Elec_Theta;


////	VSData[0] = Stru_OutLoop.Ramp.Out;
////	VSData[1] = Stru_Meas.PU_Value.MecSpd;
////	VSData[2] = Stru_Cur_dqRef.Iq;
////	VSData[3] = Stru_Cur_dq.Iq;
	
	UART_View(VSData[0],VSData[1],VSData[2],VSData[3]);
	

	//--------------------------------------------------------------------------/
	// RTT模式波形查看，用时约3-4us
  #elif (Config_Comm_Mode == Jlink_RTT)
	

//	VSData[0] = Stru_Cur_abc.Ia;
//	VSData[1] = Stru_Cur_abc.Ib;
//	VSData[2] = Stru_Foc.Elec_Theta;
//	VSData[3] = Stru_Sample.IbAD;


	VSData[0] = Stru_Cur_abc.Ia;
	VSData[1] = Stru_Encoder.ThetaE;
	VSData[2] = Stru_Foc.Elec_Theta;
	VSData[3] = Stru_Cur_abc.Ib;




//	VSData[0] = Stru_Vol_alphabetaSample.Ualpha;
//	VSData[1] = Stru_Vol_alphabetaSample.Ubeta;
//	VSData[2] = Stru_Vol_alphabeta.Ualpha;
//	VSData[3] = Stru_Vol_alphabeta.Ubeta;


//	VSData[0] = Stru_Sincos.Sin;
//	VSData[1] = Stru_Sincos.Cos;
//	VSData[2] = Stru_Foc.Elec_Theta;
//	VSData[3] = Stru_Cur_abc.Ia;


//	VSData[0] = Stru_Foc.OmegaPUFiltered;
//	VSData[1] = SpeedRef;
//	VSData[2] = Stru_Encoder.ThetaE;
//	VSData[3] = Stru_Foc.Elec_Theta;



//	VSData[0] = Stru_LHall.Learn.thetaerrorCompened;
//	VSData[1] = Stru_LHall.LHall_OffsetTheta;
//	VSData[2] = Stru_LHall.QPLLQ30.Theta>>15;
//	VSData[3] = Stru_Foc.Elec_Theta;

//	VSData[0] = Stru_OutLoop.Ramp.Out;
//	VSData[1] = Stru_Meas.PU_Filt.Power;
//	VSData[2] = Stru_Meas.PU_Value.Power;
//	VSData[3] = MOTOR_STATE;



//	VSData[0] = Stru_Vol_dq.Ud;
//	VSData[1] = Stru_Foc.OmegaPU;
//	VSData[2] = Stru_Foc.OmegaPUFiltered;
//	VSData[3] = Stru_Encoder.ThetaE;



	RTT_VIEW(VSData[0], VSData[1], VSData[2],VSData[3]);
  SEGGER_RTT_Write(1, &Rttstru, 8);

	#endif
	
}
/******************************** END OF FILE *******************************/









