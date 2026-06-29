
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    outloop.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   本文件为电机控制的功能
						包括但不限于：转速计算、外环控制
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


Struct_OutLoop						Stru_OutLoop								= {0};				//外环参数结构体

Struct_PI									Stru_PI_OL								  = {0};				//外环PI结构体

Struct_FirstOrderLPF 			LPFVbus                     = {0};
Struct_FirstOrderLPF 			LPFIbus 										= {0};
Struct_FirstOrderLPF 			LPFPower                    = {0};
Struct_FirstOrderLPF 			LPFVctrl                    = {0};
Struct_FirstOrderLPF 			LPFVbus_Aver                = {0};
Struct_FirstOrderLPF 			LPFSpeed                    = {0};
Struct_FirstOrderLPF 			LPFOmegaPU                    = {0};
int32_t										LimDelayCnt1,LimDelayCnt2,LimDelayCnt3,LimDelayCnt4;
//---------------------------------------------------------------------------/
//	Global variable definitions(declared in header file with 'extern')
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Local function prototypes('static')
//---------------------------------------------------------------------------/



//===========================================================================/
//***** definitions  end ****************************************************/
//===========================================================================/

/*****************************************************************************
* Function Name  : OutLoop_Ctrl_Init
* Description    : 外环参数初始化
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
******************************************************************************/
void OutLoop_Ctrl_Init(void)
{
	//斜坡参数初始化
	Stru_OutLoop.Ramp.Inc = SPEED_RAMP_INC;
	Stru_OutLoop.Ramp.Dec = SPEED_RAMP_DEC;
	Stru_OutLoop.Ramp.Out = 0;
	
	#if (Config_OutLoop_Mode == Speed_Loop)
	Stru_OutLoop.Cycle = TIME_SPEED_LOOP;
	#else
	Stru_OutLoop.Cycle = TIME_POWER_LOOP;
	#endif

	// 外环PI初始化
	#if (Config_OutLoop_Mode == Speed_Loop)
	{
		PI_Para_Init(&Stru_PI_OL,Stru_Para.Run.SpdKp,Stru_Para.Run.SpdKi,12,15,Stru_Foc.Curr_Is_Max,Stru_Foc.Curr_Is_Min);
	}
	#elif (Config_OutLoop_Mode == Power_Loop)
	{
		PI_Para_Init(&Stru_PI_OL,Stru_Para.Run.PwrKp,Stru_Para.Run.PwrKi,12,15,Stru_Foc.Curr_Is_Max,Stru_Foc.Curr_Is_Min);
	}
	#elif (Config_OutLoop_Mode == Ibus_Loop)
	{
		PI_Para_Init(&Stru_PI_OL,Stru_Para.Run.PwrKp,Stru_Para.Run.PwrKi,12,15,Stru_Foc.Curr_Is_Max,Stru_Foc.Curr_Is_Min);	
	}
	#endif		
	
	
}
	
/*****************************************************************************
* Function Name  : RampHandle
* Description    : 斜坡处理
* Function Call  : 
* Input Paragram : Struct_Ramp结构体、目标值
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
******************************************************************************/
void RampHandle(Struct_Ramp *Ramp, int32_t Ref)
{
	Ramp->Target = Ref;
	
	if(Ramp->Out > (Ramp->Target + Ramp->Dec))
	{
		Ramp->Out -= Ramp->Dec;
	}
	else if(Ramp->Out < (Ramp->Target - Ramp->Inc))
	{
		Ramp->Out += Ramp->Inc;
	}	
	else
	{
		Ramp->Out = Ramp->Target;
	}
}

/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  : CurrLoop_Ctrl
* Description    : 
* Function Call  : EPWM_IRQHandler中断函数调用
* Input Paragram :
* Return Value   : 
*-----------------------------------------------------------------------------
******************************************************************************/
void CurrLoop_Ctrl(void)
{
	Stru_Foc.Curr_Is_Ref = Stru_OutLoop.Ramp.Out;
}
/*****************************************************************************
* Function Name  : Speed_Loop_Ctrl
* Description    : 速度环控制
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
******************************************************************************/
void Speed_Loop_Ctrl(void)
{
	Stru_Foc.Curr_Is_Ref = PI_Controller(&Stru_PI_OL,Stru_OutLoop.Ramp.Out - Stru_Meas.PU_Value.MecSpd);
}


/*****************************************************************************
* Function Name  : Speed_Loop_Ctrl
* Description    : 功率环控制
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
******************************************************************************/
void Power_Loop_Ctrl(void)
{
	Stru_Foc.Curr_Is_Ref = PI_Controller(&Stru_PI_OL,Stru_OutLoop.Ramp.Out - Stru_Meas.PU_Filt.Power);
}

/*****************************************************************************
* Function Name  : Ibus_Loop_Ctrl
* Description    : 母线电流环控制
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
******************************************************************************/
void Ibus_Loop_Ctrl(void)
{
	Stru_Foc.Curr_Is_Ref = PI_Controller(&Stru_PI_OL,Stru_OutLoop.Ramp.Out-Stru_Meas.PU_Filt.Ibus);
}

/*****************************************************************************
* Function Name  : OutLoop_Ctrl
* Description    : 外环控制
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
******************************************************************************/
void OutLoop_Ctrl(void)
{
	static char CycleCnt;
	
	if(MOTOR_STATE == MC_RUN)
	{
		if(++CycleCnt >= Stru_OutLoop.Cycle)
		{
			CycleCnt = 0;
			
			// 调速信号爬坡处理
			RampHandle(&Stru_OutLoop.Ramp,Stru_User.TargetPu);
			//=======================================================================//	
			// 电流环控制 
			#if (Config_OutLoop_Mode == Current_Loop)
			{
				CurrLoop_Ctrl();
			}
			//=======================================================================//	
			// 速度环控制 
			#elif (Config_OutLoop_Mode == Speed_Loop)
			{
				Speed_Loop_Ctrl();
			}
			//=======================================================================//	
			// 功率环控制 
			#elif (Config_OutLoop_Mode == Power_Loop)
			{
				Power_Loop_Ctrl();
			}
			#elif (Config_OutLoop_Mode == Ibus_Loop)
			{
				Ibus_Loop_Ctrl();
			}
			#endif
			//=======================================================================//	
		}

		//-------------------------------------------------------------------/
		// 弱磁
		#if ((Config_SpeedUp_Mode == OverAndWeaken) || (Config_SpeedUp_Mode == Flux_Weaken))
		{
			WeakenFlux_Cal();
		}
		#else
			Stru_Cur_dqRef.Iq  = Stru_Foc.Curr_Is_Ref;
			Stru_Cur_dqRef.Id = 0;		
		#endif
			
	}
}



/*****************************************************************************
* Function Name  : MC_Speed_Cal
* Description    : 电机转速计算
* Function Call  : 1ms中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建					Lsy
******************************************************************************/
void MC_Speed_Cal(void)
{
	// 转速标幺值计算   Q14
	Stru_Meas.PU_Value.MecSpd = Stru_Foc.OmegaPUFiltered>>1;
	// 真实转速计算
	Stru_Meas.PhyValue.MecSpeed	= Stru_Meas.PU_Value.MecSpd * Stru_BaseValue.MechSpd / 16384;																	
	//对转速进一步滤波，用于FG反馈
	Stru_Meas.PU_Filt.MecSpd		= LPF_Cal(&LPFSpeed,Stru_Meas.PU_Value.MecSpd, _1MS_LPF_10Hz);

	// FG反馈转速计算
	Stru_User.SpeedFG						= Stru_Meas.PhyValue.MecSpeed;
}
	



/******************************** END OF FILE *******************************/

