
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    main.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   该文件主要存放故障保护代码
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
Struct_Fault            FaultMessag						= {0};					//故障信息
Struct_FaultCheck				Stru_Fault						= {0};					//故障保护参数

//---------------------------------------------------------------------------/
//	Global variable definitions(declared in header file with 'extern')
//---------------------------------------------------------------------------/

//---------------------------------------------------------------------------/
//	Local function prototypes('static')
//---------------------------------------------------------------------------/



/*****************************************************************************
* Function Name  : Fault_Clear_RestartTimes
* Description    : 清除重启次数
* Function Call  : 
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_Clear_RestartTimes(void)
{
	Stru_Fault.ShortCircuit.ReStartTimes	= 0;
	Stru_Fault.OverPhaseCur.ReStartTimes	= 0;
	Stru_Fault.PhaseLoss.ReStartTimes			= 0;
	Stru_Fault.Block.ReStartTimes					= 0;
	Stru_Fault.Start.ReStartTimes					= 0;
}
	
/*****************************************************************************
* Function Name  : Fault_ParaInit
* Description    : 故障检测参数初始化
* Function Call  : 上电开启中断前初始化一次
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_ParaInit(void)
{
	//----------------------------------------------------------------------------/
	// 硬件过流
	#if (HARDOVCUR_PROTECT_ENABLE == 1)
	{
		Stru_Fault.ShortCircuit.ReStartGAP		= HARDOVCUR_RESTART_GAP;
		Stru_Fault.ShortCircuit.ReStartNum		= HARDOVCUR_RESTART_NUM;
		Stru_Fault.ShortCircuit.ReStartTimes	= 0;	
	}
	#endif
	
	//----------------------------------------------------------------------------/
	// 软件过流
	#if (OVERPHASECUR_PROTECT_ENABLE == 1)
	{
		Stru_Fault.OverPhaseCur.OC_Value 						= Phy2Pu_Fun_Iphase(OVERPHASECUR_PROTECT_VALUE);
		Stru_Fault.OverPhaseCur.Protect_Times				= OVERPHASECUR_PROTECT_TIMES;
		
		Stru_Fault.OverPhaseCur.ReStartGap 					= OVERPHASECUR_RESTART_GAP;
		Stru_Fault.OverPhaseCur.ReStartNum					= OVERPHASECUR_RESTART_NUM;	
		Stru_Fault.OverPhaseCur.ReStartTimes				= 0;	
	}
	#endif
	
	//----------------------------------------------------------------------------/
	// 电压保护
	#if (VBUS_PROTECT_ENABLE == 1)
	{
		Stru_Fault.Vbus.Over_Volt_Value							= Phy2Pu_Fun_Vbus(OVER_VOLT_VALUE);
		Stru_Fault.Vbus.Under_Volt_Value						= Phy2Pu_Fun_Vbus(UNDER_VOLT_VALUE);
		Stru_Fault.Vbus.OV_Recover_Value						= Phy2Pu_Fun_Vbus(OV_RECOVER_VALUE);
		Stru_Fault.Vbus.UV_Recover_Value						= Phy2Pu_Fun_Vbus(UV_RECOVER_VALUE);
		Stru_Fault.Vbus.Protect_Time								= VBUS_PROTECT_TIME;
		Stru_Fault.Vbus.Recover_Time								= VBUS_RECOVER_TIME;
	}
	#endif
	
	//----------------------------------------------------------------------------/
	// 堵转保护
	#if (BLOCK_PROTECT_ENABLE == 1)  
	{
		Stru_Fault.Block.BlockSpeed_Max							= BLOCK_SPEED_MAX;
		Stru_Fault.Block.BlockSpeed_Min							= BLOCK_SPEED_MIN;
		Stru_Fault.Block.Protect_Time								= BLOCK_PROTECT_TIME;
		
		Stru_Fault.Block.ReStartGAP									= BLOCK_RESTART_GAP;
		Stru_Fault.Block.ReStartNum									= BLOCK_RESTART_NUM;	
		Stru_Fault.Block.ReStartTimes								= 0;	
	}
	#endif
	
	//----------------------------------------------------------------------------/
	// 缺相保护
	#if (PHASELOSS_PROTECT_ENABLE == 1)  
	{
		Stru_Fault.PhaseLoss.Curr_Min								= Phy2Pu_Fun_Iphase(PHASELOSS_CUR_MIN);
		Stru_Fault.PhaseLoss.Protect_Time						= PHASELOSS_PROTECT_TIME;
		Stru_Fault.PhaseLoss.ReStartGAP							= PHASELOSS_RESTART_GAP;
		Stru_Fault.PhaseLoss.ReStartNum							= PHASELOSS_RESTART_NUM;
		Stru_Fault.PhaseLoss.ReStartTimes						= 0;	
	}
	#endif
	
	//----------------------------------------------------------------------------/
	// 温度保护
	#if (TEMP_PROTECT_ENABLE == 1)
	{
		Stru_Fault.Temp_Mos.Over_Temp_Value					= OVER_TEMP_VALUE;
		Stru_Fault.Temp_Mos.Under_Temp_Value				= UNDER_TEMP_VALUE;
		Stru_Fault.Temp_Mos.Protect_Time						= TEMP_PROTECT_TIME;
		
		Stru_Fault.Temp_Mos.OT_Recover_Value				= OTEMP_RECOVER_VALUE;
		Stru_Fault.Temp_Mos.UT_Recover_Value				= UTEMP_RECOVER_VALUE;
		Stru_Fault.Temp_Mos.Recover_Time						= TEMP_RECOVER_TIME;	
	}
	#endif
	
	//----------------------------------------------------------------------------/
	// 启动失败保护
	#if (STARTFAIL_PROTECT_ENABLE == 1)
	{
		Stru_Fault.Start.Protect_Time								= STARTFAIL_PROTECT_TIME;
		
		Stru_Fault.Start.ReStartGAP									= STARTFAIL_RESTART_GAP;
		Stru_Fault.Start.ReStartNum									= STARTFAIL_RESTART_NUM;
		Stru_Fault.Start.ReStartTimes								= 0;	
	}
	#endif
	
}

/*****************************************************************************
* Function Name  : Fault_Vbus_Check
* Description    : 电压检测\恢复
* Function Call  : 1ms中断调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_Vbus_Check(void)
{
	static uint16_t Fault1msCnt_Vbus;
	
	if((Stru_Meas.PU_Value.Vbus > Stru_Fault.Vbus.Over_Volt_Value) || (Stru_Meas.PU_Value.Vbus < Stru_Fault.Vbus.Under_Volt_Value) )
		Fault1msCnt_Vbus ++;
	else
		Fault1msCnt_Vbus = 0;
	
	// 故障识别
	if(Fault1msCnt_Vbus > Stru_Fault.Vbus.Protect_Time) 
	{
		if(Stru_Meas.PU_Value.Vbus > Stru_Fault.Vbus.Over_Volt_Value)
		{
			Fault_OverVoltage = 1;
		}
		if(Stru_Meas.PU_Value.Vbus < Stru_Fault.Vbus.Under_Volt_Value)
		{
			Fault_UnderVoltage = 1;
		}
		
		SYSTEM_STATE 			= SYS_FAULT;
		Fault1msCnt_Vbus	= 0;
		
	}
	
	#if (VBUS_RECOVER_ENABLE)
	{
		static uint16_t Re1msCnt_Vbus;
		
		if(	(Stru_Meas.PU_Value.Vbus < Stru_Fault.Vbus.OV_Recover_Value) 		\
			&& (Stru_Meas.PU_Value.Vbus > Stru_Fault.Vbus.UV_Recover_Value) )
		{
			Re1msCnt_Vbus ++;	
		}
		else
		{
			Re1msCnt_Vbus = 0;
		}
			
		
		if(Re1msCnt_Vbus > Stru_Fault.Vbus.Recover_Time)
		{
			Fault_OverVoltage 	= 0;
			Fault_UnderVoltage 	= 0;
			Re1msCnt_Vbus 			= 0;
			Fault1msCnt_Vbus		= 0;
		}	
	}
	#endif
}

/*****************************************************************************
* Function Name  : Fault_PhaseLoss_Check
* Description    : 缺相检测
* Function Call  : 1ms中断调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
									 V0.2    2024/09/18    修改每相独立判断				Lsy
******************************************************************************/
void Fault_PhaseLoss_Check(void)
{
	static uint16_t ALossCnt = 0,BLossCnt = 0,CLossCnt = 0;
	if((MOTOR_STATE == MC_SW) ||(MOTOR_STATE == MC_RUN))
	{
		// A相
		if(Stru_Fault.Ia_Max < Stru_Fault.PhaseLoss.Curr_Min)		
			ALossCnt ++;
		else
			ALossCnt = 0;

		// B相
		if(Stru_Fault.Ib_Max < Stru_Fault.PhaseLoss.Curr_Min)		
			BLossCnt ++;
		else
			BLossCnt = 0;
		
		// C相
		if(Stru_Fault.Ic_Max < Stru_Fault.PhaseLoss.Curr_Min)		
			CLossCnt ++;
		else
			CLossCnt = 0;		
		
		Stru_Fault.Ia_Max = 0;	Stru_Fault.Ib_Max	= 0;	Stru_Fault.Ic_Max = 0;
		
		if(	 (ALossCnt > Stru_Fault.PhaseLoss.Protect_Time)
			|| (BLossCnt > Stru_Fault.PhaseLoss.Protect_Time)
			|| (CLossCnt > Stru_Fault.PhaseLoss.Protect_Time))
		{
			Fault_PhaseLoss	= 1;
			SYSTEM_STATE		= SYS_FAULT;
			MOTOR_STATE			= MC_POWERON;
			ALossCnt				= 0;
			BLossCnt				= 0;
			CLossCnt				= 0;
		}	
	}
	else
	{
		ALossCnt = 0;
		BLossCnt = 0;
		CLossCnt = 0;
	}
}

/*****************************************************************************
* Function Name  : Fault_Block_Check
* Description    : 堵转检测
* Function Call  : 1ms中断调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_Block_Check(void)
{
	static uint16_t FaultBlockCnt = 0;
	if((MOTOR_STATE == MC_SW) || (MOTOR_STATE == MC_RUN))
	{
		
		#if (Config_Obser_Run == Run_OBSER2)
		{
			if( (Stru_Meas.PhyValue.MecSpeed > Stru_Fault.Block.BlockSpeed_Max) 
					|| (Stru_Meas.PhyValue.MecSpeed < Stru_Fault.Block.BlockSpeed_Min)
					|| (Stru_Ob2.Sig14<200) )
			{
				FaultBlockCnt ++;
			}
			else
			{
				FaultBlockCnt = 0;
			}
		}
		#else
		{
			if( (Stru_Meas.PhyValue.MecSpeed > Stru_Fault.Block.BlockSpeed_Max) 
					|| (Stru_Meas.PhyValue.MecSpeed < Stru_Fault.Block.BlockSpeed_Min) )
			{
				FaultBlockCnt ++;
			}
			else
			{
				FaultBlockCnt = 0;
			}
		}
		#endif
	
		if(FaultBlockCnt > Stru_Fault.Block.Protect_Time)
		{
			FaultBlockCnt = 0;
			Fault_Block		= 1;
			
			SYSTEM_STATE		= SYS_FAULT;
			MOTOR_STATE			= MC_POWERON;
		}
	}
	else
		FaultBlockCnt = 0;
	
}
	
/*****************************************************************************
* Function Name  : Fault_StartFail_Check
* Description    : 启动失败检测
* Function Call  : 1ms中断调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_StartFail_Check(void)
{
	static uint32_t StartFailCnt;
	if(MOTOR_STATE == MC_STARTUP)
	{
		StartFailCnt ++;
		if(StartFailCnt > Stru_Fault.Start.Protect_Time)
		{
			SYSTEM_STATE		= SYS_FAULT;
			MOTOR_STATE			= MC_POWERON;			
			Fault_StartFail = 1;
			StartFailCnt = 0;
		}
	}
	else
		StartFailCnt = 0;
}

/*****************************************************************************
* Function Name  : Fault_Temp_Check
* Description    : 温度检测\恢复
* Function Call  : 1ms中断调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_Temp_Check(void)
{
	static uint16_t Fault1msCnt_Temp;
	
	if((Stru_Meas.PhyValue.TempMos > Stru_Fault.Temp_Mos.Over_Temp_Value) || (Stru_Meas.PhyValue.TempMos < Stru_Fault.Temp_Mos.Under_Temp_Value) )
		Fault1msCnt_Temp ++;
	else
		Fault1msCnt_Temp = 0;
	
	// 故障识别
	if(Fault1msCnt_Temp > Stru_Fault.Temp_Mos.Protect_Time) 
	{
		if(Stru_Meas.PhyValue.TempMos > Stru_Fault.Temp_Mos.Over_Temp_Value)
		{
			Fault_OverTemp = 1;
		}
		if(Stru_Meas.PhyValue.TempMos < Stru_Fault.Temp_Mos.Under_Temp_Value)
		{
			Fault_UnderTemp = 1;
		}
		
		SYSTEM_STATE 			= SYS_FAULT;
		Fault1msCnt_Temp	= 0;
	}
	
	#if (TEMP_RECOVER_ENABLE)
	{
		static uint16_t Re1msCnt_Temp;

		if(	(Stru_Meas.PhyValue.TempMos < Stru_Fault.Temp_Mos.OT_Recover_Value) 		\
			&& (Stru_Meas.PhyValue.TempMos > Stru_Fault.Temp_Mos.UT_Recover_Value) )
		{
			Re1msCnt_Temp ++;	
		}
		else
		{
			Re1msCnt_Temp = 0;
		}
			
		if(Re1msCnt_Temp > Stru_Fault.Temp_Mos.Recover_Time)
		{
			Fault_OverTemp 			= 0;
			Fault_UnderTemp 		= 0;
			Re1msCnt_Temp 			= 0;
			Fault1msCnt_Temp		= 0;
		}	
	}
	#endif
}



/*****************************************************************************
* Function Name  : Fault_OFFSET_Check
* Description    : 偏置异常检测
* Function Call  : 中断开启前偏置计算调用
* Input Paragram : 
* Return Value   : none
* note           : 故障不强制转换SYSTEM_STATE
* Version        : V0.1    2024/07/16    新建			Lsy
******************************************************************************/
void Fault_OFFSET_Check(void)
{
	int32_t IbusOffMax,IbusOffMin,PhaseOffMax,PhaseOffMin;
	
	IbusOffMax = (HW_OFFSET_IBUS + OFFSET_ERROR_VALUE) * _Q12_VAL / HW_ADC_REF;
	IbusOffMin = (HW_OFFSET_IBUS - OFFSET_ERROR_VALUE) * _Q12_VAL / HW_ADC_REF;

	PhaseOffMax = (HW_OFFSET_IPHASE + OFFSET_ERROR_VALUE) * _Q12_VAL / HW_ADC_REF;
	PhaseOffMin = (HW_OFFSET_IPHASE - OFFSET_ERROR_VALUE) * _Q12_VAL / HW_ADC_REF;

	if((Stru_Sample.IBusOffset > IbusOffMax) || (Stru_Sample.IBusOffset < IbusOffMin) || \
		 (Stru_Sample.IaOffset > PhaseOffMax) || (Stru_Sample.IaOffset < PhaseOffMin)		|| \
		 (Stru_Sample.IbOffset > PhaseOffMax) || (Stru_Sample.IbOffset < PhaseOffMin) )
	{
		Fault_OFFSET = 1;
	}	
	// 故障不强制转换SYSTEM_STATE，程序正常执行完sys_Init中内容
}
/*****************************************************************************
* Function Name  : Fault_ReStart_Process
* Description    : 故障重启处理
* Function Call  : 主函数1ms任务
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_ReStart_Process(void)
{
	if(FaultMessag.Code == NOERROR)
	{
		// 运行后清除故障
		#if (FAULTCLEAR_RUN_EN)
		{
			if(MOTOR_STATE == MC_RUN)
			{
				// 重启次数清零
				Fault_Clear_RestartTimes();
			}		
		}
		#endif
	}
	else
	{
		//-------------------------------------------------------------------//
		// 硬件过流重启
		#if	(HARDOVCUR_RESTART_ENABLE)
		{
			static uint32_t RSTRTShortCircuitCnt;
			if((Stru_Fault.ShortCircuit.ReStartTimes < Stru_Fault.ShortCircuit.ReStartNum) && Fault_ShortCircuit)
			{
				RSTRTShortCircuitCnt ++;
				if(RSTRTShortCircuitCnt > Stru_Fault.ShortCircuit.ReStartGAP)
				{
					RSTRTShortCircuitCnt = 0;
					Fault_ShortCircuit = 0;
					// EPWM故障刹车保护重启 
					EPWM_ResetFaultBrake();
					
					// 重启次数
					#if (HARDOVCUR_RESTART_NUM == 72)
					Stru_Fault.ShortCircuit.ReStartTimes = 0;
					#else
					Stru_Fault.ShortCircuit.ReStartTimes ++;
					#endif
				}
			}
		}
			
		#endif

		//-------------------------------------------------------------------//
		// 软件过流重启
		#if (OVERPHASECUR_RESTART_ENABLE)
		{
			static uint16_t RestarOCCnt = 0;
			if((Stru_Fault.OverPhaseCur.ReStartTimes < Stru_Fault.OverPhaseCur.ReStartNum) && Fault_OverPhaseCur )
			{
				RestarOCCnt ++;
				
				if(RestarOCCnt > Stru_Fault.OverPhaseCur.ReStartGap)
				{
					Fault_OverPhaseCur	= 0;
					RestarOCCnt 				= 0;
					
					// 重启次数
					#if (OVERPHASECUR_RESTART_NUM == 72)
					Stru_Fault.OverPhaseCur.ReStartTimes = 0;
					#else
					Stru_Fault.OverPhaseCur.ReStartTimes ++;
					#endif
				}
			}		
		}
		#endif
		//-------------------------------------------------------------------//
		// 缺相保护重启
		#if (PHASELOSS_RESTART_ENABLE)
		{
			static uint16_t RSTRTPhaseLossCnt = 0;
			if((Stru_Fault.PhaseLoss.ReStartTimes < Stru_Fault.PhaseLoss.ReStartNum) && Fault_PhaseLoss)
			{
				RSTRTPhaseLossCnt ++;
				
				if(RSTRTPhaseLossCnt > Stru_Fault.PhaseLoss.ReStartGAP)
				{
					Fault_PhaseLoss		= 0;
					RSTRTPhaseLossCnt = 0;
					
					// 重启次数
					#if (PHASELOSS_RESTART_NUM == 72)
					Stru_Fault.PhaseLoss.ReStartTimes = 0;
					#else
					Stru_Fault.PhaseLoss.ReStartTimes ++;
					#endif
				}
			}
		}
		#endif
			//-------------------------------------------------------------------//
		// 堵转保护重启
		#if (BLOCK_RESTART_ENABLE)
		{
			static uint16_t RSTRTBlockCnt = 0;
			if((Stru_Fault.Block.ReStartTimes < Stru_Fault.Block.ReStartNum) && (Fault_Block))
			{
				RSTRTBlockCnt ++;
				if(RSTRTBlockCnt > Stru_Fault.Block.ReStartGAP)
				{
					RSTRTBlockCnt = 0;
					Fault_Block		= 0;
					
					// 重启次数
					#if (BLOCK_RESTART_NUM == 72)
					Stru_Fault.Block.ReStartTimes = 0;
					#else
					Stru_Fault.Block.ReStartTimes ++;
					#endif
				}
			}

		}
		#endif
		//-------------------------------------------------------------------//
		// 启动失败重启
		#if (STARTFAIL_RESTART_ENABLE)
		{
			static uint16_t RSTRT_StartFailCnt = 0;
			if((Stru_Fault.Start.ReStartTimes < Stru_Fault.Start.ReStartNum) && Fault_StartFail)
			{
				RSTRT_StartFailCnt ++;
				if(RSTRT_StartFailCnt > Stru_Fault.Start.ReStartGAP)
				{
					Fault_StartFail = 0;
					RSTRT_StartFailCnt = 0;

					// 重启次数
					#if (STARTFAIL_RESTART_NUM == 72)
					Stru_Fault.Start.ReStartTimes = 0;
					#else
					Stru_Fault.Start.ReStartTimes ++;
					#endif
				}
			}		
		}
		#endif
	}
}
  

/*****************************************************************************
* Function Name  : Fault_Check_FOCTask
* Description    : 故障检测高频任务
* Function Call  : EPWM中断/ADC中断
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_Check_FOCTask(void)
{
	static uint8_t OCcnt = 0;
	
	Stru_Fault.Ia_ABS = ABSFUN(Stru_Cur_abc.Ia);
	Stru_Fault.Ib_ABS = ABSFUN(Stru_Cur_abc.Ib);
	Stru_Fault.Ic_ABS = ABSFUN(Stru_Cur_abc.Ic);

	// 查找最大电流,用于缺相检测
	if(Stru_Fault.Ia_Max < Stru_Fault.Ia_ABS)		Stru_Fault.Ia_Max = Stru_Fault.Ia_ABS;
	if(Stru_Fault.Ib_Max < Stru_Fault.Ib_ABS)		Stru_Fault.Ib_Max = Stru_Fault.Ib_ABS;
	if(Stru_Fault.Ic_Max < Stru_Fault.Ic_ABS)		Stru_Fault.Ic_Max = Stru_Fault.Ic_ABS;
	
	//-------------------------------------------------------------------//
	// 软件过流
	#if (OVERPHASECUR_PROTECT_ENABLE)
	if((MOTOR_STATE == MC_STARTUP) || (MOTOR_STATE == MC_SW )|| (MOTOR_STATE == MC_RUN ))
	{
		if( ((Stru_Fault.Ia_ABS > Stru_Fault.OverPhaseCur.OC_Value)		\
			|| (Stru_Fault.Ib_ABS > Stru_Fault.OverPhaseCur.OC_Value)		\
			|| (Stru_Fault.Ic_ABS > Stru_Fault.OverPhaseCur.OC_Value))	)
		{
			OCcnt ++;
		}
		else
			OCcnt = 0;	
	}
	else
		OCcnt = 0;	

	if(OCcnt > Stru_Fault.OverPhaseCur.Protect_Times)
	{
		Fault_OverPhaseCur	= 1;
		SYSTEM_STATE 				= SYS_FAULT;
		MOTOR_STATE					= MC_POWERON;
		OCcnt								= 0;
		
		// 关闭桥臂输出
		Bridge_Output_Off();
	}
	#endif
	
}
	

/*****************************************************************************
* Function Name  : Fault_Check_1msTask
* Description    : 故障检测1ms任务
* Function Call  : EPWM中断/ADC中断
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_Check_1msTask(void)
{
	//-------------------------------------------------------------------//
	// 电压保护
	#if (VBUS_PROTECT_ENABLE)
	
		Fault_Vbus_Check();
	#endif
	//-------------------------------------------------------------------//
	// 缺相保护	
	#if (PHASELOSS_PROTECT_ENABLE )
	
		Fault_PhaseLoss_Check();
	#endif
	//-------------------------------------------------------------------//
	// 堵转保护
	#if (BLOCK_PROTECT_ENABLE )
	
		Fault_Block_Check();
	#endif
	//-------------------------------------------------------------------//
	// 启动失败保护
	#if (STARTFAIL_PROTECT_ENABLE )
	
		Fault_StartFail_Check();
	#endif

	//-------------------------------------------------------------------//
	// 温度保护
	#if (TEMP_PROTECT_ENABLE )
	
		Fault_Temp_Check();
	#endif
}
/*****************************************************************************
* Function Name  : Fault_Show_Source
* Description    : 按照优先级显示故障代码
* Function Call  : 主函数10ms任务调用
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/05    新建				Nmg,Lsy
******************************************************************************/
void Fault_Show_Source(void)
{
	if(Fault_Flag == 0)							FaultMessag.Code = NOERROR;
	
	else if(Fault_ShortCircuit)			FaultMessag.Code = Short_Circuit;
	else if(Fault_OverPhaseCur)			FaultMessag.Code = Over_PhaseCurr;
	else if(Fault_OverVoltage)			FaultMessag.Code = Over_Volt;
	else if(Fault_UnderVoltage)			FaultMessag.Code = Under_Volt;
	else if(Fault_Block)						FaultMessag.Code = Motor_Block;
	else if(Fault_PhaseLoss)				FaultMessag.Code = Phase_Loss;
	else if(Fault_OverTemp)					FaultMessag.Code = Over_Temp;
	else if(Fault_UnderTemp)				FaultMessag.Code = Under_Temp;
	else if(Fault_OFFSET)						FaultMessag.Code = Offset_Error;
	else if(Fault_StartFail)				FaultMessag.Code = Start_Fail;
	else if(Fault_MosError)					FaultMessag.Code = Mos_Error;
	else if(Fault_HallError)				FaultMessag.Code = Hall_Error;
	
}





/******************************** END OF FILE *******************************/



