

//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    User.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   用于放置用户对电机控制的函数
*					 包括但不限于PWM输入信号处理，调速信号到目标信号的转换，system状态机
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
#define SpdCtrlMapLen					(8)
//---------------------------------------------------------------------------/
//	Local variable  definitions
//---------------------------------------------------------------------------/

SystStates_e						SYSTEM_STATE					= {SYS_INIT};		// sys状态机
Struct_User							Stru_User							= {0};					// 用户命令
Struct_Capture					Stru_Capture					= {0};					// 捕获结构体

//---------------------------------------------------------------------------/
// 填写说明
// 第一列为调速信号：
// VSP调速填写电压值标幺值：Phy2Pu_Fun_Vctrl(x)；频率调速填写频率：单位为0.1Hz；占空比调速填写占空比，单位0.1%
// 第二列为目标值：
// 填写物理值
const float	SpeedCtrl_Map[SpdCtrlMapLen][2]=
{
	// 调速信号												控制目标值
		{50,														0},						//低于此信号关机
																									// 迟滞缓冲
		{80,														2},						//高于此信号启动
		{200,														7.2},				
		{500,														15.6},			
		{700,														22.7},			
		{900,														32.1},			
		{0,															0}							
};

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
* Function Name  : User_Motor_On
* Description    : 用户控制电机运行
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建
******************************************************************************/
void User_Motor_On(void)
{
	Stru_User.RunFlag		= 1;
}

/*****************************************************************************
* Function Name  : User_Motor_Off
* Description    : 用户控制电机关闭
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建
******************************************************************************/
void User_Motor_Off(void)
{
		Stru_User.RunFlag		= 0;
}

/*****************************************************************************
* Function Name  : UserCtrl_Para_Init
* Description    : 用户控制参数初始化
* Function Call  : 上电运行一次
* Input Paragram : 
* Return Value   : none
* note           : 
* author         : 
* Version        : V0.1    2024/07/05    新建		Lsy
******************************************************************************/
void UserCtrl_Para_Init(void)
{
	//---------------------------------------------------------------------------/	
	// 目标值处理 Y
	#if (Config_OutLoop_Mode == Current_Loop)
	{
		Stru_User.Target_Max = Phy2Pu_Fun_Iphase(Target_MAX);
		Stru_User.Target_Min = Phy2Pu_Fun_Iphase(Target_START);	
		
		if(Stru_User.Target_Max > Phy2Pu_Fun_Iphase(IPHASE_MAX) )
			 Stru_User.Target_Max = Phy2Pu_Fun_Iphase(IPHASE_MAX);	
		if(Stru_User.Target_Min < Phy2Pu_Fun_Iphase(IPHASE_MIN) )
			 Stru_User.Target_Min = Phy2Pu_Fun_Iphase(IPHASE_MIN);			
	}
	#elif (Config_OutLoop_Mode == Speed_Loop)
	{
		Stru_User.Target_Max = Phy2Pu_Fun_Speed(Target_MAX);
		Stru_User.Target_Min = Phy2Pu_Fun_Speed(Target_START);	
	
	}
	#elif  (Config_OutLoop_Mode == Power_Loop)
	{
		Stru_User.Target_Max = Phy2Pu_Fun_Power(Target_MAX);
		Stru_User.Target_Min = Phy2Pu_Fun_Power(Target_START);
		
	}
	#elif  (Config_OutLoop_Mode == Ibus_Loop)
	{
		Stru_User.Target_Max = Phy2Pu_Fun_Ibus(Target_MAX);
		Stru_User.Target_Min = Phy2Pu_Fun_Ibus(Target_START);
		

	}
	#endif
	//---------------------------------------------------------------------------/	
	// 信号值处理	X
	#if (Speed_Govern_Mode == PWM_Control)
	{
		Stru_User.Signal_OFF = PWM_DUTY_STOP;
		Stru_User.Signal_ON  = PWM_DUTY_START;
		Stru_User.Signal_Max = PWM_DUTY_MAX;
		
	}
	#elif  (Speed_Govern_Mode == VSP_Control)
	{
		Stru_User.Signal_OFF = Phy2Pu_Fun_Vctrl(VSP_REF_OFF);
		Stru_User.Signal_ON = Phy2Pu_Fun_Vctrl(VSP_REF_ON);
		Stru_User.Signal_Max = Phy2Pu_Fun_Vctrl(VSP_REF_MAX);
	}
	#elif  (Speed_Govern_Mode == CLK_Control)
	{
		Stru_User.Signal_OFF = PWM_FRE_OFF;
		Stru_User.Signal_ON = PWM_FRE_ON;
		Stru_User.Signal_Max = PWM_FRE_MAX;
	}
	#endif
	
	//---------------------------------------------------------------------------/	
	// 斜率计算	Y = Ke * X + B
	Stru_User.TargetKe = (Stru_User.Target_Max - Stru_User.Target_Min) * _Q15_VAL \
											/(Stru_User.Signal_Max - Stru_User.Signal_ON);
	Stru_User.TargetB  = Stru_User.Target_Min - (Stru_User.Signal_ON * Stru_User.TargetKe >> 15);
	
	//---------------------------------------------------------------------------/	
	Stru_User.RunFlag = 0;
}

/*****************************************************************************
* Function Name  : User_Target_CurveCal
* Description    : 控制信号曲线拟合计算
* Function Call  : 10ms调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建			Lsy
******************************************************************************/
int32_t User_Target_CurveCal(void)
{
	int32_t  TargetBuff  = 0;		
	int32_t TargetN1,TargetN;
	
	// 目标物理值计算 & 开机信号控制
	char Mapi=0;	
	if(Stru_User.CtrlSig < SpeedCtrl_Map[0][0])
	{
		Stru_User.SigOnFlag		= 0;
		Stru_User.TargetPu 	= 0;
		TargetBuff	= 0;
		Stru_User.TargetPhy = 0;
	}
	else
	{
		// 迟滞缓冲
		if(Stru_User.CtrlSig < SpeedCtrl_Map[1][0])
		{
			#if (Config_OutLoop_Mode == Current_Loop)
			
				TargetBuff = (int32_t)(SpeedCtrl_Map[1][1] * Stru_Coff.Phy2PU_Iphase) >> 10;
			
			#elif (Config_OutLoop_Mode == Power_Loop)

				TargetBuff = (int32_t)(SpeedCtrl_Map[1][1] * Stru_Coff.Phy2PU_Power) >> 10;

			#elif (Config_OutLoop_Mode == Speed_Loop)

				TargetBuff = (int32_t)(SpeedCtrl_Map[1][1] * Stru_Coff.Phy2PU_Speed) >> 10;
			
			#elif (Config_OutLoop_Mode == Ibus_Loop)

				TargetBuff = (int32_t)(SpeedCtrl_Map[1][1] * Stru_Coff.Phy2PU_Ibus) >> 10;
			#endif	
		}
		else
		{
			for(Mapi = 1; Mapi< SpdCtrlMapLen; Mapi ++)
			{
				#if (Config_OutLoop_Mode == Current_Loop)
				{
					TargetN1 = (int32_t)(Stru_Coff.Phy2PU_Iphase * SpeedCtrl_Map[Mapi+1][1]) >> 10;
					TargetN	= (int32_t)(Stru_Coff.Phy2PU_Iphase * SpeedCtrl_Map[Mapi][1]) >> 10;				
				}
				#elif (Config_OutLoop_Mode == Power_Loop)
				{
					TargetN1 = (int32_t)(Stru_Coff.Phy2PU_Power * SpeedCtrl_Map[Mapi+1][1]) >> 10;
					TargetN	= (int32_t)(Stru_Coff.Phy2PU_Power * SpeedCtrl_Map[Mapi][1]) >> 10;				
				}
				#elif (Config_OutLoop_Mode == Speed_Loop)
				{
					TargetN1 = (int32_t)(Stru_Coff.Phy2PU_Speed * SpeedCtrl_Map[Mapi+1][1]) >> 10;
					TargetN	= (int32_t)(Stru_Coff.Phy2PU_Speed * SpeedCtrl_Map[Mapi][1]) >> 10;				
				}
				#elif (Config_OutLoop_Mode == Ibus_Loop)
				{
					TargetN1 = (int32_t)(Stru_Coff.Phy2PU_Ibus * SpeedCtrl_Map[Mapi+1][1]) >> 10;
					TargetN	= (int32_t)(Stru_Coff.Phy2PU_Ibus * SpeedCtrl_Map[Mapi][1]) >> 10;				
				}
				#endif					
		
				// 查表超出范围结束
				if(Mapi == SpdCtrlMapLen -1)
				{
					TargetBuff = TargetN;
					break;
				}
				// 表格异常或查表提前结束
				if(SpeedCtrl_Map[Mapi][0] > SpeedCtrl_Map[Mapi+1][0])		
				{
					TargetBuff = TargetN;
					break;											
				}			
				// 正常查表
				if(Stru_User.CtrlSig < SpeedCtrl_Map[Mapi+1][0])
				{
					TargetBuff = (TargetN1 - TargetN)																	\
											* (Stru_User.CtrlSig - SpeedCtrl_Map[Mapi][0])				\
											/ (SpeedCtrl_Map[Mapi+1][0] - SpeedCtrl_Map[Mapi][0])	\
											+ TargetN ;
					break;
				}
			}		
			
			// 信号标志置位
			Stru_User.SigOnFlag		= 1;
		}
	}
	
	return TargetBuff;
}

/*****************************************************************************
* Function Name  : User_Target_LinearCal
* Description    : 控制信号直线拟合计算
* Function Call  : 10ms调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建			
******************************************************************************/
int32_t User_Target_LinearCal(void)
{
	int32_t  TargetBuff  = 0;		
	// 目标物理值计算 & 开机信号控制
	if(Stru_User.CtrlSig < Stru_User.Signal_OFF)
	{
		Stru_User.SigOnFlag	= 0;
		Stru_User.TargetPu 	= 0;
		TargetBuff = 0;
		Stru_User.TargetPhy = 0;
	}
	else
	{
		if(Stru_User.CtrlSig > Stru_User.Signal_ON)
		{
			if(Stru_User.CtrlSig > Stru_User.Signal_Max)
				TargetBuff = Stru_User.Target_Max;
			else
				TargetBuff = (Stru_User.CtrlSig * Stru_User.TargetKe >> 15 )+ Stru_User.TargetB  ;
			
			// 信号标志置位
			Stru_User.SigOnFlag		= 1;
		}
		else
			TargetBuff = Stru_User.Target_Min;

		if(TargetBuff > Stru_User.Target_Max)		TargetBuff = Stru_User.Target_Max;
		if(TargetBuff < Stru_User.Target_Min)		TargetBuff = Stru_User.Target_Min;	
	}
	
	return TargetBuff;
}


/*****************************************************************************
* Function Name  : User_Ctrl_Motor
* Description    : 用户控制目标计算
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建		Lsy
******************************************************************************/
float	DebufTarget = DEBUG_TARGET;
void User_Ctrl_Motor(void)
{
	#if (Speed_Govern_Mode != Debug_Control)
	{
		//---------------------------------------------------------------------------/
		// 调速信号给定
		#if  (Speed_Govern_Mode == VSP_Control)
		{
			Stru_User.CtrlSig = Stru_Meas.PU_Filt.Vctrl;
		}	
		#elif (Speed_Govern_Mode == PWM_Control)
		{
			Stru_User.CtrlSig = (int32_t)Stru_Capture.Duty;
		}
		#elif  (Speed_Govern_Mode == CLK_Control)
		{
			Stru_User.CtrlSig = Stru_Capture.Fre;
		}
		#endif
		
		//---------------------------------------------------------------------------/
		// 目标值计算
		#if (Table_LookUp_En)
		//查表拟合曲线
		Stru_User.TargetPu = User_Target_CurveCal();
		#else
		//直线拟合
		Stru_User.TargetPu = User_Target_LinearCal();
		#endif

		//---------------------------------------------------------------------------/	
		// 电机启动、停止控制
		if((SYSTEM_STATE == SYS_RUN) && (Fault_Flag == 0) && (Stru_User.SigOnFlag))
		{
			User_Motor_On();
		}
		else
		{
			User_Motor_Off();
		}
	}
	#else
	{
		//---------------------------------------------------------------------------/
		// 调试模式
		#if (Config_OutLoop_Mode == Current_Loop)
		{
			if(DebufTarget > IPHASE_MAX)			DebufTarget = IPHASE_MAX;
			if(DebufTarget < IPHASE_MIN)			DebufTarget = IPHASE_MIN;
		
			Stru_User.TargetPu = (int32_t)(Stru_Coff.Phy2PU_Iphase * DebufTarget) >> 10;			
		}
		#elif (Config_OutLoop_Mode == Power_Loop)
		{
			Stru_User.TargetPu = (int32_t)(Stru_Coff.Phy2PU_Power * DebufTarget) >> 10;			
		}
		#elif (Config_OutLoop_Mode == Speed_Loop)
		{
			Stru_User.TargetPu = (int32_t)(Stru_Coff.Phy2PU_Speed * DebufTarget) >> 10;		
		}
		#elif (Config_OutLoop_Mode == Ibus_Loop)
		{
			Stru_User.TargetPu = (int32_t)(Stru_Coff.Phy2PU_Ibus * DebufTarget) >> 10;		
		}
		#endif	
//   	User_Motor_On();		
	}
	#endif

	//---------------------------------------------------------------------------/	
	// 控制目标真实值计算
	#if (Config_OutLoop_Mode == Current_Loop)
		Stru_User.TargetPhy = Stru_Coff.PU2Phy_Iphase * Stru_User.TargetPu >> 15;
	#elif (Config_OutLoop_Mode == Power_Loop)
		Stru_User.TargetPhy = Stru_Coff.PU2Phy_Power * Stru_User.TargetPu >> 15;
	#elif (Config_OutLoop_Mode == Speed_Loop)
	{
		Stru_User.TargetPhy = (int32_t)(MECH_SPEED_BASE * ((float)Stru_User.TargetPu / 16384.0)) ;
	}
	#elif (Config_OutLoop_Mode == Ibus_Loop)
		Stru_User.TargetPhy = Stru_Coff.PU2Phy_Ibus * Stru_User.TargetPu >> 15;
	#endif		

}

/*****************************************************************************
* Function Name  : Task1ms_Main
* Description    : 主函数时间任务
* Function Call  : 主函数
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建		Lsy、nmg
******************************************************************************/
void Task1ms_Main(void)
{
	static char msTimeCnt[4] ;
	
	Flag_1ms_Intr = 0;
	//==============================1ms任务=========================================//
	//-------------------------------------------------------------------//	
	

	//-------------------------------------------------------------------//	
	// 故障重启
	Fault_ReStart_Process();
	
	//==============================10ms任务=========================================//
	
	msTimeCnt[0] ++;
	//-------------------------------------------------------------------//
	// 物理值计算
	if(msTimeCnt[0] == 1)
	{
		PhyVal_Cal();	
	}
	//-------------------------------------------------------------------//
	// 用户控制电机运行、停机、快慢
	if(msTimeCnt[0] == 2)
	{
		User_Ctrl_Motor();
	}
	//-------------------------------------------------------------------//
	// PWM捕获
	if(msTimeCnt[0] == 3)
	{
		#if (Config_CCP_Capture == CCP_Capture_Enable)	
		
			User_PWM_Capture();   
		
		#endif	
	}	
	//-------------------------------------------------------------------//
	//FG反馈
	if(msTimeCnt[0] == 4)
	{
		#if (Config_CCP_PWM == CCP_PWM_Enable)
		
			User_Speed_Out(Stru_User.SpeedFG);
		
		#endif
	}	
	//-------------------------------------------------------------------//
	// 故障代码显示
	if( msTimeCnt[0] >= 10)
	{
		msTimeCnt[0] = 0;
		
		Fault_Show_Source();
	}

	//==============================100ms任务=========================================//	
	if( ++msTimeCnt[1] >= 100)
	{
		msTimeCnt[1] = 0;
		
		#if (Sleep_Control_Mode == Sleep_Enable)
		User_Sleep_Manage();
		#endif
		
	}
}

/*****************************************************************************
* Function Name  : System_Control
* Description    : 系统状态机
* Function Call  : 主函数
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建
******************************************************************************/
void System_Control(void)
{
	switch(SYSTEM_STATE)
	{
		//===================================================================//
		// 初始化状态机，等待电源等稳定
		case SYS_INIT:
		{
			if(Stru_Time.PowerOn == 0)
			{
				SYSTEM_STATE = SYS_RUN;					
			}
			break;	
		}
		//===================================================================//
		//  系统正常工作状态  
		case SYS_RUN:
		{
			if(Fault_Flag != 0)
			{
				SYSTEM_STATE = SYS_FAULT;
				User_Motor_Off();
			}
			break;
		}
		//===================================================================//
		//  系统故障状态      
		case SYS_FAULT:
		{
			// 系统复位
			if(Fault_Flag == 0)
				SYSTEM_STATE = SYS_RUN;

			// 用户层关机
			User_Motor_Off();
			
			// 关闭桥臂输出
			Bridge_Output_Off();
			
			// 允许调速信号复位重启
			#if ((FAULTCLEAR_SIGNRESET_EN) && (Speed_Govern_Mode != Debug_Control))
			{
				// 清零调速信号后，重新给调速信号，再清零故障
				#if (1)
				{
					static uint8_t ResetStep = 0;
					if(Fault_Flag == 0)					ResetStep = 0;
					if((Stru_User.SigOnFlag == 0) && (Fault_Flag))		ResetStep = 1;
					if((Stru_User.SigOnFlag == 1) && (ResetStep == 1))
					{
						ResetStep = 0;
						// EPWM故障刹车保护重启 
						EPWM_ResetFaultBrake();
						// 故障码清零
						Fault_Flag = 0;
						// 重启次数清零
						Fault_Clear_RestartTimes();
					}							
				}
				// 清零调速信号后，直接清零故障
				#else	
				{
					if(Stru_User.SigOnFlag == 0)
					{
						// EPWM故障刹车保护重启 
						EPWM_ResetFaultBrake();
						// 故障码清零
						Fault_Flag = 0;
						// 重启次数清零
						Fault_Clear_RestartTimes();
					}							
				}
				#endif				
			}
			#endif
			break;					
		}
		//===================================================================//
		default:
		break;			
	}
}

/******************************** END OF FILE *******************************/


