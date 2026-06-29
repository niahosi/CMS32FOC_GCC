
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    foc.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   该文件主要存放Foc控制框架中的基本函数，以及电机状态机
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

Struct_Cur_abc            Stru_Cur_abc 								= {0};				//Ia、Ib、Ic的标幺值的Q15格式      

Struct_Cur_alphabeta      Stru_Cur_alphabeta 					= {0};				//Ialpha、Ibeta的标幺值的Q15格式

Struct_Cur_dq             Stru_Cur_dqRef 							= {0};				//dq轴参考电流的标幺值的Q15格式

Struct_Cur_dq             Stru_Cur_dq 								= {0};				//dq轴反馈电流的标幺值的Q15格式

Struct_Vol_abc            Stru_Vol_abc 								= {0};				//a、Ub、Uc的调制信号的Q15格式

Struct_Vol_alphabeta      Stru_Vol_alphabeta 					= {0};				//Ualpha、Ubeta的调制信号的Q15格式

Struct_Vol_alphabeta      Stru_Vol_alphabeta_Comp 		= {0};				//补偿后的Ualpha、Ubeta调制信号的Q15格式

Struct_Vol_dq             Stru_Vol_dq 								= {0};				//dq轴电压的标幺值的Q15格式

Struct_Vol_alphabeta      Stru_Vol_alphabetaSample 		= {0};				//通过反电动势采样电路得到的Ua、Ub、Uc的调制信号的Q15格式

Struct_Vol_abc            Stru_Vol_abcSample 					= {0};				//通过反电动势采样电路得到的Ualpha、Ubeta的调制信号的Q15格式

Struct_Vol_dq             Stru_Vol_dqSample 					= {0};				//通过反电动势采样电路得到的Ud、Uq的调制信号的Q15格式

Struct_Sincos             Stru_Sincos 								= {0};				//转子位置的正余弦值

Struct_Foc                Stru_Foc 										= {0};				//Foc的相关参数

MotorState_e              MOTOR_STATE									= {MC_POWERON};	//电机状态机

Struct_FOCCount           Stru_FocCnt 								= {0};				//FOC状态机计数器

Struct_AlgorPara          Stru_AlgorPara 							= {0};				//算法库参数

Struct_Test								Stru_Test										= {0};				// 测试结构体

uint16_t 	                VSData[4] 									= {0};				//串口发送数据缓存

Struct_Encoder						Stru_Encoder                = {0};				//编码器结构体 


//---------------------------------------------1号电机
const uint8_t TableLength = 14;
int32_t OmegaPU_Table[TableLength]={0,2143,4365,6583,8751,10940,13160,15160,17549,19711,21850,24083,26101,28284};
int32_t Ud_Table[TableLength] = {0,-124,-267,-673,-1125,-2054,-3236,-4370,-6051,-7655,-8406,-10726,-13295,-15623};
int32_t Solpe_K[TableLength-1] = {0,0,0,0,0,0,0,0,0,0,0,0,0};






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
* Function Name  : Cal_Slope
* Description    : 对查表斜率进行计算
* Function Call  : FOC中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void Cal_Slope(void)
{
	for(uint8_t i=0;i<(TableLength-1);i++)
	{
		Solpe_K[i]= ((Ud_Table[i+1]-Ud_Table[i])*32768)/(OmegaPU_Table[i+1]-OmegaPU_Table[i]);
	}
}

/*****************************************************************************
* Function Name  : LookupUd_Table
* Description    : 查表法给出Ud
* Function Call  : FOC中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
int32_t LookupUd_Table(int32_t OmegaPU)
{
	int32_t UdRef = 0;
	uint8_t Index = 0;
	for(uint8_t i=0;i<(TableLength-1);i++)
	{
		if( (OmegaPU>=OmegaPU_Table[i]) && (OmegaPU<OmegaPU_Table[i+1]))
		{
			Index=i;
			break;
		}
		else    
		{
			Index = TableLength-2;
		}

	}
	UdRef = Ud_Table[Index]+ ((OmegaPU-OmegaPU_Table[Index])*Solpe_K[Index]>>15);
	
	return FIXP_sat(UdRef, MAX_MODULATE, -MAX_MODULATE);
}



/*****************************************************************************
* Function Name  : GetEncoderAngle
* Description    : 对编码器角度进行解算
* Function Call  : FOC中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void GetEncoderAngle(Struct_Encoder	*hEncoder	,int16_t AngleIn  )
{
	hEncoder->ThetaM = AngleIn + hEncoder->Offset_ThetaM;
	hEncoder->ThetaE = hEncoder->ThetaM*hEncoder->Pairs;
	
	hEncoder->SinCosSig = SinCos_Cal(hEncoder->ThetaE);
	int32_t temp = 0;
	temp = (hEncoder->SinCosSig.Sin * hEncoder->Sincos.Cos - hEncoder->SinCosSig.Cos * hEncoder->Sincos.Sin)>>15;
	
	hEncoder->Omega = PI_Controller(&hEncoder->pi,temp);
	hEncoder->OmegaPU = (hEncoder->Omega*Stru_AlgorPara.OmegaPLL2SpdPUQ15>>8);
	hEncoder->ThetaEPLL += hEncoder->Omega;
	hEncoder->Sincos = SinCos_Cal(hEncoder->ThetaEPLL);
}





/*****************************************************************************
* Function Name  : Clark
* Description    : Clark变换
* Function Call  : FOC中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void Clark(void)
{
    //iAlpha计算 
    Stru_Cur_alphabeta.Ialpha = Stru_Cur_abc.Ia;
    //iBeta计算 
    Stru_Cur_alphabeta.Ibeta = (Stru_Cur_abc.Ia * 9459 + Stru_Cur_abc.Ib * 18919) >> 14;
}

/*****************************************************************************
* Function Name  : Park
* Description    : 
* Function Call  : FOC中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void Park(void)
{
    Stru_Cur_dq.Id = (Stru_Cur_alphabeta.Ialpha  * Stru_Sincos.Cos + Stru_Cur_alphabeta.Ibeta * Stru_Sincos.Sin) >> 15;
    //iq计算
    Stru_Cur_dq.Iq = (-Stru_Cur_alphabeta.Ialpha * Stru_Sincos.Sin + Stru_Cur_alphabeta.Ibeta * Stru_Sincos.Cos) >> 15;  
}

/*****************************************************************************
* Function Name  : RevPark
* Description    : 
* Function Call  : FOC中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void RevPark(void)
{ 	
    //vAlpha计算 
    Stru_Vol_alphabeta.Ualpha = (Stru_Vol_dq.Ud * Stru_Sincos.Cos - Stru_Vol_dq.Uq * Stru_Sincos.Sin) >> 15;
    //vBeta计算 
    Stru_Vol_alphabeta.Ubeta  = (Stru_Vol_dq.Ud * Stru_Sincos.Sin + Stru_Vol_dq.Uq * Stru_Sincos.Cos) >> 15;  
}


/*****************************************************************************
* Function Name  : SwitchFromWindToRun
* Description    : 状态机从MC_WIND切换到MC_RUN时需要执行的操作
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void SwitchFromWindToRun()
{
////强顺风切入时的初始化操作
//如果采用刹车的方式，需要给dq轴电流控制器的积分项赋初值
	#if (CONFIG_WINDCHECK_MODE == WINDCHECK_MODE1)
	{
		Stru_PI_Id.IntegralTerm = HWDivider(Stru_Wind.Ud<<14,Stru_Coff.ModuSig2PU_Vphase)<<Stru_PI_Id.qKi;
		Stru_PI_Iq.IntegralTerm = HWDivider(Stru_Wind.Uq<<14,Stru_Coff.ModuSig2PU_Vphase)<<Stru_PI_Iq.qKi;
	}
	#endif
	
	Stru_PI_OL.IntegralTerm = ((ABSFUN(Stru_Cur_dq.Iq))<<Stru_PI_OL.qKi);
	Stru_Foc.Curr_Is_Ref = ABSFUN(Stru_Cur_dq.Iq);
	#if (Config_Obser_Run == Run_OBSER2)
	{
		Switch2Ob2(Stru_Foc.Elec_Theta,Stru_Foc.Elec_Omega);
	}
	#elif (Config_Obser_Run == Run_OBSER3)
	{
		Switch2Ob3(Stru_Foc.Elec_Theta,Stru_Foc.Elec_Omega);
	}
	#endif
	Ob1_ParaRefresh(Stru_Para.Start.PLLKp,Stru_Para.Start.PLLKi,START_ADP_MAX,START_ADP_MIN);
}

/*****************************************************************************
* Function Name  : SwitchFromWindToRun
* Description    : 状态机从MC_WIND切换到MC_RUN时需要执行的操作
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void StartupFromStandstill()
{
	#if (CONFIG_INITIAL_POSITION == POSITION_ALIGN)
	{
		MOTOR_STATE = MC_ALIGN;
		Stru_Cur_dqRef.Iq = Stru_Para.Start.AlignCurMin;
		Bridge_Output_On();
	}
	#elif  (CONFIG_INITIAL_POSITION == POSITION_PULSE_INJECTION)
	{
		MOTOR_STATE = MC_IPD;
	}
	#else
	{
		MOTOR_STATE = MC_STARTUP;
		Bridge_Output_On();
	}
	#endif

}


/*****************************************************************************
* Function Name  : SwitchToRun
* Description    : 从其他状态机切换到运行状态机需要执行的代码
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void SwitchToRun(void)
{
	PI_Set_Integrater(&Stru_PI_OL,Stru_Cur_dqRef.Iq);
	#if (Config_OutLoop_Mode == Current_Loop)
	{
		Stru_OutLoop.Ramp.Out = Stru_Cur_dq.Iq;
	}
	#elif (Config_OutLoop_Mode == Speed_Loop)
	{
		Stru_OutLoop.Ramp.Out = Stru_Meas.PU_Filt.MecSpd;		
	}
	#elif (Config_OutLoop_Mode == Power_Loop)
	{
		Stru_OutLoop.Ramp.Out = Stru_Meas.PU_Filt.Power;	
	}
	#elif (Config_OutLoop_Mode == Ibus_Loop)
	{
		Stru_OutLoop.Ramp.Out = Stru_Meas.PU_Filt.Ibus;	
	}
	#endif	
}
/*****************************************************************************
* Function Name  : FOC_Read_Voltage
* Description    : 电压读取
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_Read_Voltage(void)
{
	#if(Config_VOLTAGE_SAMPLE==VOLTAGE_SAMPLE_ENABLE)
	{
		Stru_Coff.AD2ModuSig_Bef = 	HWDivider(Stru_Coff.AD2ModuSig_Bef_temp*256,Stru_Meas.PU_Filt.Vbus);	//转化系数Q8，将采样到的反电动势转化为调制信号
		if (Flag_MotorDir == DIR_CW)
		{
			Stru_Sample.BefUAD = Get_ADC_Result(ADC_DATA_UBEMF);
			Stru_Sample.BefVAD = Get_ADC_Result(ADC_DATA_VBEMF);
			Stru_Sample.BefWAD = Get_ADC_Result(ADC_DATA_WBEMF);
		}
		else
		{
			Stru_Sample.BefVAD = Get_ADC_Result(ADC_DATA_UBEMF);
			Stru_Sample.BefUAD = Get_ADC_Result(ADC_DATA_VBEMF);
			Stru_Sample.BefWAD = Get_ADC_Result(ADC_DATA_WBEMF);
		}

		static int32_t UaN=0,UbN=0,UcN=0,UoN=0;
		UaN = ((Stru_Sample.BefUAD - Stru_Sample.BefUOffset)*Stru_Coff.AD2ModuSig_Bef>>8);
		UbN = ((Stru_Sample.BefVAD - Stru_Sample.BefVOffset)*Stru_Coff.AD2ModuSig_Bef>>8);
		UcN = ((Stru_Sample.BefWAD - Stru_Sample.BefWOffset)*Stru_Coff.AD2ModuSig_Bef>>8);
		UoN = ((UaN+UbN+UcN)*10923>>15);

		Stru_Vol_abcSample.Ua = UaN - UoN;
		Stru_Vol_abcSample.Ub = UbN - UoN;
		Stru_Vol_abcSample.Uc = UcN - UoN;


		Stru_Vol_alphabetaSample.Ualpha = Stru_Vol_abcSample.Ua;
		Stru_Vol_alphabetaSample.Ubeta = (Stru_Vol_abcSample.Ua * 9459 + Stru_Vol_abcSample.Ub * 18919) >> 14;

		Stru_Vol_dqSample.Ud = (Stru_Vol_alphabetaSample.Ualpha  * Stru_Sincos.Cos + Stru_Vol_alphabetaSample.Ubeta * Stru_Sincos.Sin) >> 15;
		Stru_Vol_dqSample.Uq = (-Stru_Vol_alphabetaSample.Ualpha * Stru_Sincos.Sin + Stru_Vol_alphabetaSample.Ubeta * Stru_Sincos.Cos) >> 15; 
	}
	#endif	
}
/*****************************************************************************
* Function Name  : FOC_Read_Current
* Description    : 电流读取
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void  FOC_Read_Current(void)
{
	Stru_Sample.IaAD = Get_ADC_Result(ADC_DATA_CHA);
	Stru_Sample.IbAD = Get_ADC_Result(ADC_DATA_CHB);
	
	//-------------------------------------------------------------------//
	// 单电阻
  #if (Config_Shunt_Mode == Single_Shunt)
	{		
		FOC_SingleCurrent_Calc();	  
	}
	// 双电阻
	#else
	{		
		//-------------------------------------------------------------------//
		// 相序匹配
		int32_t CurrBuff_PGA1, CurrBuff_PGA2;
		int32_t CurrBuff_A, CurrBuff_B,CurrBuff_C;
	
		CurrBuff_PGA1 = ((Stru_Sample.IaOffset - Stru_Sample.IaAD) * Stru_Coff.AD2PU_Ip >> 10);
		CurrBuff_PGA2 = ((Stru_Sample.IbOffset - Stru_Sample.IbAD) * Stru_Coff.AD2PU_Ip >> 10);		
		
		#if (IP_SAMP_CH == IP_UV)
		{
			CurrBuff_A = CurrBuff_PGA1;
			CurrBuff_B = CurrBuff_PGA2;
			CurrBuff_C = - CurrBuff_PGA1 - CurrBuff_PGA2;
		}
		#elif (IP_SAMP_CH == IP_UW)
		{
			CurrBuff_A = CurrBuff_PGA1;
			CurrBuff_C = CurrBuff_PGA2;
			CurrBuff_B = - CurrBuff_PGA1 - CurrBuff_PGA2;
		}
		#elif (IP_SAMP_CH == IP_VU)
		{
			CurrBuff_B = CurrBuff_PGA1;
			CurrBuff_A = CurrBuff_PGA2;
			CurrBuff_C = - CurrBuff_PGA1 - CurrBuff_PGA2;
		}
		#elif (IP_SAMP_CH == IP_VW)
		{
			CurrBuff_B = CurrBuff_PGA1;
			CurrBuff_C = CurrBuff_PGA2;
			CurrBuff_A = - CurrBuff_PGA1 - CurrBuff_PGA2;
		}
		#elif (IP_SAMP_CH == IP_WU)
		{
			CurrBuff_C = CurrBuff_PGA1;
			CurrBuff_A = CurrBuff_PGA2;
			CurrBuff_B = - CurrBuff_PGA1 - CurrBuff_PGA2;
		
		}
		#elif (IP_SAMP_CH == IP_WV)
		{
			CurrBuff_C = CurrBuff_PGA1;
			CurrBuff_B = CurrBuff_PGA2;
			CurrBuff_A = - CurrBuff_PGA1 - CurrBuff_PGA2;
		}
		#endif

		//-------------------------------------------------------------------//
		// 正反处理
		if (Flag_MotorDir == DIR_CW)
		{
        Stru_Cur_abc.Ia = CurrBuff_A;
        Stru_Cur_abc.Ib = CurrBuff_B;
        Stru_Cur_abc.Ic = CurrBuff_C; 
		}			
		else
		{
        Stru_Cur_abc.Ia = CurrBuff_B;
        Stru_Cur_abc.Ib = CurrBuff_A;
        Stru_Cur_abc.Ic = CurrBuff_C;
		}	
	
  }
	#endif
	
	//-------------------------------------------------------------------//
	// 交流供电纹波电压补偿
	#if (Config_VbusRipple_Comp == Vbus_Comp_ENABLE)
	{
	  Stru_Sample.VbusAD 			= Get_ADC_Result(ADC_DATA_VBUS); 
		Stru_Meas.PU_Value.Vbus = Stru_Sample.VbusAD * Stru_Coff.AD2PU_Vbus >> 10;     
		VbusRipple_Comp_Cal(Stru_Meas.PU_Value.Vbus);		
	}
	#endif
}

/*****************************************************************************
* Function Name  : FOC_VsLimit
* Description    : 电压限制圆处理
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_VsLimit(void)
{
	#if (0)            //通过查表的方式来计算电压极限圆
	{
		int32_t		v_Vq_Limit = 0; 
		v_Vq_Limit = (Stru_Vol_dq.Ud * Stru_Vol_dq.Ud)>>15;
		v_Vq_Limit = (Stru_Foc.Q15_VsmaxSquare - v_Vq_Limit)>>6;  
		Stru_PI_Iq.UpperOutputLimit 	= (int32_t)Q15Sqrt_Table[v_Vq_Limit];
		Stru_PI_Iq.UpperIntegralLimit = (Stru_PI_Iq.UpperOutputLimit<<Stru_PI_Iq.qKi);
	}

	#else            //通过硬件开方来计算电压极限圆
	{
		int32_t		v_Vq_Limit = 0; 
		v_Vq_Limit = (Stru_Vol_dq.Ud * Stru_Vol_dq.Ud);
		
		v_Vq_Limit = Stru_Foc.VsmaxSquare - v_Vq_Limit;  
		
		Stru_PI_OL.UpperOutputLimit 	= HWSqrt(v_Vq_Limit);
		Stru_PI_OL.UpperIntegralLimit = (Stru_PI_OL.UpperOutputLimit<<Stru_PI_Iq.qKi);
		
		Stru_PI_OL.LowerOutputLimit 	= -Stru_PI_OL.UpperOutputLimit;
		Stru_PI_OL.LowerIntegralLimit = (Stru_PI_OL.LowerOutputLimit<<Stru_PI_Iq.qKi);
		
	}
	#endif
	
}

/*****************************************************************************
* Function Name  : Foc_Ctrl_Init
* Description    : FOC初始化参数
* Function Call  : MC_Init状态机调用
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void Foc_Ctrl_Init()
{
	Ob1_Init();               
  Ob2_Init();               
  Ob3_Init();  
	Wind_CtrlInit();

	
	
//编码器结构体初始化
	Stru_Encoder.Pairs = Stru_Motor.pole;
	Stru_Encoder.ThetaE = 0;
	Stru_Encoder.ThetaM = 0;
	Stru_Encoder.Offset_ThetaM = ENCODER_OFFSET;               //
	Stru_Encoder.SinCosSig = SinCos_Cal(Stru_Encoder.ThetaE);		//
	Stru_Encoder.Omega = 0;
	Stru_Encoder.OmegaPU = 0;
	Stru_Encoder.OmegaPUFiltered = 0;
	
	Stru_Encoder.ThetaEPLL = 0;;					
	Stru_Encoder.Sincos = SinCos_Cal(Stru_Encoder.ThetaEPLL);				
	
  Stru_Encoder.pi.qKp						= 15;
  Stru_Encoder.pi.qKi						= 15;              
  Stru_Encoder.pi.Error					= 0;
  Stru_Encoder.pi.IntegralTerm		= 0;
  Stru_Encoder.pi.Kp							= 5000;
  Stru_Encoder.pi.Ki							= 30;
  
  Stru_Encoder.pi.LowerOutputLimit		= -16384;
  Stru_Encoder.pi.LowerIntegralLimit = (Stru_Encoder.pi.LowerOutputLimit<<Stru_Encoder.pi.qKi);
  Stru_Encoder.pi.UpperOutputLimit		= 16384;
  Stru_Encoder.pi.UpperIntegralLimit	= (Stru_Encoder.pi.UpperOutputLimit<<Stru_Encoder.pi.qKi);
	
	
	
	
	
	
	
	
	//------------------------------------------------------------------------/	
	// 运行方向
	Flag_MotorDir = Stru_Para.Mode.Dir;
	//------------------------------------------------------------------------/	
	//电流初始化
  Stru_Cur_abc.Ia						= 0;  
  Stru_Cur_abc.Ib						= 0;  
  Stru_Cur_abc.Ic						= 0;  
  Stru_Cur_alphabeta.Ialpha	= 0;
  Stru_Cur_alphabeta.Ibeta	= 0;

  Stru_Cur_dq.Id						= 0;
  Stru_Cur_dq.Iq						= 0;
	
  Stru_Cur_dqRef.Id					= 0;
  Stru_Cur_dqRef.Iq					= Stru_Para.Start.StartCurMin;
	Stru_Foc.Curr_Is_Ref			= Stru_Para.Start.StartCurMax;
    
	//------------------------------------------------------------------------/	
	//电压初始化    
  Stru_Vol_abc.Ua						= 0;
  Stru_Vol_abc.Ub						= 0;
  Stru_Vol_abc.Uc						= 0;
  Stru_Vol_alphabeta.Ualpha	= 0;
  Stru_Vol_alphabeta.Ubeta	= 0;
  Stru_Vol_dq.Ud						= 0;
  Stru_Vol_dq.Uq						= 0;
  
	Stru_Vol_abcSample.Ua = 0;
	Stru_Vol_abcSample.Ub = 0;
	Stru_Vol_abcSample.Uc = 0;
	Stru_Vol_alphabetaSample.Ualpha = 0;
	Stru_Vol_alphabetaSample.Ubeta = 0;
	Stru_Vol_dqSample.Ud = 0;
	Stru_Vol_dqSample.Uq = 0;
	
	//------------------------------------------------------------------------/	
	//FOC参数、角度、极限圆初始化 
	Stru_Foc.OmegaPU					= 0;
  Stru_Foc.Elec_Omega				= 0;
  Stru_Foc.OmegaPUFiltered	= 0;
  Stru_Foc.Elec_Theta				= 0;
  Stru_Sincos.Sin						= 0;
  Stru_Sincos.Cos						= 32768;
	//------------------------------------------------------------------------/	
	//SVPWM初始化 
	Stru_SVPWM.SectorNum			= 1;
  Stru_SVPWM.Last_Sector		= 1;
	Stru_SVPWM.Cnt_TG1st			= Stru_SVPWM.EPWM_Period >> 1;
	Stru_SVPWM.Cnt_TG2nd			= Stru_SVPWM.EPWM_Period >> 2;
	
	Stru_SVPWM.LN_Ctrl				= 0;
	Stru_SVPWM.LN_State				= 0;
	Stru_SVPWM.LN_StateLast		= 0;
	//------------------------------------------------------------------------/	


	//------------------------------------------------------------------------/	
	//Cnt值初始化 
  Stru_FocCnt.CntCaloffset		= 0;
  Stru_FocCnt.CntIdle				= 0;
  Stru_FocCnt.CntInit				= 0;
  Stru_FocCnt.CntCharge			= 0;
  Stru_FocCnt.Cnttesthd			= 0;
  Stru_FocCnt.CntWind				= 0;
  Stru_FocCnt.CntAlign1			= 0;
  Stru_FocCnt.CntAlign2			= 0;
  Stru_FocCnt.CntStartup1		= 0;
  Stru_FocCnt.CntStartup2		= 0;
  Stru_FocCnt.CntSw					= 0;
  Stru_FocCnt.CntRun					= 0;
  Stru_FocCnt.CntStop				= 0;
  Stru_FocCnt.CntFault				= 0;

	//------------------------------------------------------------------------/	
	//ID 电流环初始化
	PI_Para_Init(&Stru_PI_Id,Stru_Para.Run.Dkp,Stru_Para.Run.Dki,12,15,Stru_Para.Run.Dout_Max,Stru_Para.Run.Dout_Min);
	//------------------------------------------------------------------------/	
	//IQ 电流环初始化
	PI_Para_Init(&Stru_PI_Iq,Stru_Para.Run.Qkp,Stru_Para.Run.Qki,12,15,Stru_Para.Run.Qout_Max,Stru_Para.Run.Qout_Min);
	
	
	//------------------------------------------------------------------------/	
	//外环参数初始化
	OutLoop_Ctrl_Init();
	
	
	#if ((Config_SpeedUp_Mode == OverAndWeaken) || (Config_SpeedUp_Mode == Flux_Weaken))
	{
		WeakenFlux_Ctrl_Init();
	}
	#endif

}


/*****************************************************************************
* Function Name  : FOC_TASK_POWERON
* Description    : MC_POWERON状态机
* Function Call  : FOC中断调用
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2025/01/08   新建			Xh
******************************************************************************/
uint8_t Hall_LearnTime = 0;
void FOC_TASK_POWERON()
{
  Bridge_Output_Off();  
	//参数初始化
  Foc_Ctrl_Init(); 
	
	if(Fault_Flag == 0 && SYSTEM_STATE == SYS_RUN)
	{
		#if (Config_Run_Mode == RUN_MD_LHALL && Config_HWTEST_MODE == HWTEST_OFF)
		{
			LHall_Ctrl_Init();
			if( Stru_Foc.LHall_LearnState != LEARN_FINISHED)
			{
				MOTOR_STATE = MC_LHALL_LEARN;
			  Hall_LearnTime++;
				Bridge_Output_On();
			}
			else
			{
				MOTOR_STATE = MC_CONVERGENCE;
			}
		}
		#else
		  MOTOR_STATE = MC_IDLE;
		#endif
	}
	
}

/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  : FOC_LHall_LEARN
* Description    : 线性霍尔角度强拖标定
* Function Call  : FOC中断调用
* Input Paragram :
* Return Value   : 
* Note           :V0.1    2025/01/06    新建			WJ、Xh
*-----------------------------------------------------------------------------
******************************************************************************/
void FOC_LHall_LEARN(void)
{
	static int32_t Cnt = 0;
	static int32_t LearnSuccessCnt = 0;
	Stru_Ob2.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);              
	Stru_Ob2.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
	Stru_Ob2.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
	Stru_Ob2.Ibeta = Stru_Cur_alphabeta.Ibeta;          
	Ob2_Cal();	
	Stru_Foc.Elec_Theta = Stru_Ob2.Theta;  	
	Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);

	switch (Stru_LHall.Learn.LearnState)
  {
  	case SIG_OFFSET: 
			
			Stru_Sample.LHallAlphaOffset = LPF_Cal(&Stru_LHall.Learn.LPFLHall_AlphaOffset,Stru_Sample.LHallAlphaAD, _Foc_LPF_1Hz);

			Stru_Sample.LHallBetaOffset = LPF_Cal(&Stru_LHall.Learn.LPFLHall_BetaOffset,Stru_Sample.LHallBetaAD, _Foc_LPF_1Hz);
		
			Cnt++;
			if(Cnt>2*EPWM_FREQ)
			{
				Cnt = 0;
				Stru_LHall.Learn.LearnState = SIG_ORDER;
			}
			break;
			
		case SIG_ORDER:
			Stru_LHall.Ualpha =  Stru_Sample.LHallAlphaAD - Stru_Sample.LHallAlphaOffset;
			Stru_LHall.Ubeta = Stru_Sample.LHallBetaAD - Stru_Sample.LHallBetaOffset;

			LHall_Theta_Cal(&Stru_LHall, Stru_LHall.Ualpha<<5, Stru_LHall.Ubeta<<5);
			Cnt++;
			if(Cnt>EPWM_FREQ)
			{
				Cnt = 0;
				Stru_LHall.Learn.LearnState = ANGLE_OFFSET;
				if(Stru_LHall.QPLLQ30.OmegaPU > 0 )
				{
					Stru_LHall.Hall_Seq = HALL_SEQ_FORWARD;
				}
				else
				{
					Stru_LHall.Hall_Seq = HALL_SEQ_REVERSE;
					Stru_LHall.QPLLQ30.PI.IntegralTerm = 0;
					Stru_LHall.QPLLQ30.Theta = (Stru_Foc.Elec_Theta<<15);
					
				}
				
			}
		  break;

  	case ANGLE_OFFSET:
			if(Stru_LHall.Hall_Seq == HALL_SEQ_FORWARD)
			{
				Stru_LHall.Ualpha =  Stru_Sample.LHallAlphaAD - Stru_Sample.LHallAlphaOffset;
				Stru_LHall.Ubeta = Stru_Sample.LHallBetaAD - Stru_Sample.LHallBetaOffset;
			}
			else
			{
				Stru_LHall.Ualpha = Stru_Sample.LHallBetaAD - Stru_Sample.LHallBetaOffset;
				Stru_LHall.Ubeta =  Stru_Sample.LHallAlphaAD - Stru_Sample.LHallAlphaOffset;
			}
			LHall_Theta_Cal(&Stru_LHall, Stru_LHall.Ualpha<<5, Stru_LHall.Ubeta<<5);

			if(Stru_LHall.QPLLQ30.OmegaPU > OMEGAPU_THRESHOLD)
			{
				LearnSuccessCnt++;
				if(LearnSuccessCnt > EPWM_FREQ)
				{
					Stru_LHall.Learn.LearnSuccessFlag = 1;
				}
			}
			else
			{
				LearnSuccessCnt = 0;
				Stru_LHall.Learn.LearnSuccessFlag = 0;
			}
					
			// -----------------------------------计算偏置角度---------------------------------------------------//
			Stru_LHall.Learn.thetaerrorPre = Stru_LHall.Learn.thetaerror;
			
			Stru_LHall.Learn.thetaerror = Stru_Ob2.Theta - Stru_LHall.ThetaE;
			
			Stru_LHall.Learn.deltaError = Stru_LHall.Learn.thetaerror - Stru_LHall.Learn.thetaerrorPre;
			
			//判断是否需要补偿误差
			if(Stru_LHall.Learn.deltaError>_Q16(0.8))
			{
				Stru_LHall.Learn.Flag_ErrorNeed_Compen = !Stru_LHall.Learn.Flag_ErrorNeed_Compen;
				if(Stru_LHall.Learn.Flag_CompenAngle_Determined == 0)
				{
					Stru_LHall.Learn.Flag_CompenAngle_Determined = -1;					
				}
			}
			else if (Stru_LHall.Learn.deltaError< - _Q16(0.8))
			{
				Stru_LHall.Learn.Flag_ErrorNeed_Compen = !Stru_LHall.Learn.Flag_ErrorNeed_Compen;
				if(Stru_LHall.Learn.Flag_CompenAngle_Determined == 0)
				{
					 Stru_LHall.Learn.Flag_CompenAngle_Determined = 1;
				}
			}
			
			//对误差进行补偿
			if(Stru_LHall.Learn.Flag_ErrorNeed_Compen)
			{
				 if(Stru_LHall.Learn.Flag_CompenAngle_Determined == -1)
				 {
						Stru_LHall.Learn.thetaerrorCompened = Stru_LHall.Learn.thetaerror - 65536;
				 }
				 else if(Stru_LHall.Learn.Flag_CompenAngle_Determined == 1)
				 {
						Stru_LHall.Learn.thetaerrorCompened = Stru_LHall.Learn.thetaerror + 65536;
				 }
			}
			else
			{
				 Stru_LHall.Learn.thetaerrorCompened = Stru_LHall.Learn.thetaerror;
			}	
			
			
			Stru_LHall.LHall_OffsetTheta = LPF_Cal(&Stru_LHall.Learn.LPFLHall_OffsetTheta,Stru_LHall.Learn.thetaerrorCompened, _Foc_LPF_1Hz);

			
			//flash写入
			Cnt++;
			if(Cnt > EPWM_FREQ*2)
			{
				Bridge_Output_Off();
				if(Stru_LHall.Learn.LearnSuccessFlag == 1)
				{
					//先擦除Flash
					EraseSector(FLASH_PARA_ADDR); 
					//写入学习状态位
					uint32_t*addr = (uint32_t *)(FLASH_PARA_ADDR+LEARN_STATE_INDEX*4);
					Flash_Write_Int32(addr, LEARN_FINISHED);
					//写入霍尔信号顺序偏置
					addr = (uint32_t *)(FLASH_PARA_ADDR+LHALL_ORDER_INDEX*4);
					Flash_Write_Int32(addr, Stru_LHall.Hall_Seq);
					//写入Hall_Alpha信号偏置
					addr = (uint32_t *)(FLASH_PARA_ADDR+OFFSET_LHALLALPHA_INDEX*4);
					Flash_Write_Int32(addr, Stru_Sample.LHallAlphaOffset);
					//写入Hall_Beta信号偏置
					addr = (uint32_t *)(FLASH_PARA_ADDR+OFFSET_LHALLBETA_INDEX*4);
					Flash_Write_Int32(addr, Stru_Sample.LHallBetaOffset);
					//写入角度偏置
					addr = (uint32_t *)(FLASH_PARA_ADDR+LHALL_OFFSET_ANGLE_INDEX*4);
					Flash_Write_Int32(addr, Stru_LHall.LHall_OffsetTheta);
					//写入和校验
					int32_t checksum  = LEARN_FINISHED + Stru_LHall.Hall_Seq + Stru_Sample.LHallAlphaOffset + Stru_Sample.LHallBetaOffset + Stru_LHall.LHall_OffsetTheta;
					addr = (uint32_t *)(FLASH_PARA_ADDR+CHECKSUM_INDEX*4);
					Flash_Write_Int32(addr , checksum);
					
					Stru_Foc.LHall_LearnState = LHall_Para[LEARN_STATE_INDEX];
					Stru_Sample.LHallAlphaOffset = LHall_Para[OFFSET_LHALLALPHA_INDEX];
					Stru_Sample.LHallBetaOffset = LHall_Para[OFFSET_LHALLBETA_INDEX];
					Stru_Foc.OffsetTheta = LHall_Para[LHALL_OFFSET_ANGLE_INDEX];
					
					MOTOR_STATE = MC_POWERON;	
				}
				else
				{
					if(Hall_LearnTime > 2)//如果学习次数超过三次仍然失败，则表明霍尔信号可能存在问题，需重新上电再学习
					{
						Fault_HallError = 1;
					}
					MOTOR_STATE = MC_POWERON;	
				}
				Cnt = 0;
				LearnSuccessCnt = 0;
				Stru_LHall.Learn.LearnSuccessFlag = 0;
				Stru_Sample.LHallAlphaOffset = 0;
				Stru_Sample.LHallBetaOffset = 0;
				Stru_LHall.LHall_OffsetTheta = 0;
				Stru_LHall.Learn.Flag_CompenAngle_Determined = 0;
				Stru_LHall.Learn.Flag_ErrorNeed_Compen = 0;
				Stru_LHall.Learn.thetaerrorCompened = 0;
				Stru_LHall.Learn.thetaerrorPre = 0;
				Stru_LHall.Learn.deltaError = 0;
				Stru_LHall.Learn.thetaerror = 0;
			}
			break;
		
		default:
  		break;

	}
	//-------------------------------------------------------------------/
	// 电流环
	Stru_Cur_dqRef.Id = 0;
	Stru_Cur_dqRef.Iq = Phy2Pu_Fun_Iphase(HALL_LEARN_CUR);  
	
  Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));         
  FOC_VsLimit();                                                                              
  Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq)); 
	Stru_Foc.VsAmp = HWSqrt(Stru_Vol_dq.Ud * Stru_Vol_dq.Ud + Stru_Vol_dq.Uq*Stru_Vol_dq.Uq);
	//-------------------------------------------------------------------/
	//反Park变换
  RevPark(); 
	
	//-------------------------------------------------------------------/
	// 死区补偿
	#if (CONFIG_DeadComp_MODE == DeadComp_Enable)
	{
		FOC_DeadComp_Cal();
		Stru_Vol_alphabeta_Comp.Ualpha = Stru_Vol_alphabeta.Ualpha + Stru_DeadComp.s32_Comp_Alpha;
		Stru_Vol_alphabeta_Comp.Ubeta  = Stru_Vol_alphabeta.Ubeta + Stru_DeadComp.s32_Comp_Beta;
	}
	#else
	{
		Stru_Vol_alphabeta_Comp.Ualpha = Stru_Vol_alphabeta.Ualpha;
		Stru_Vol_alphabeta_Comp.Ubeta  = Stru_Vol_alphabeta.Ubeta;
	}
	#endif
	

	//-------------------------------------------------------------------/	
	// 计算空间矢量
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta_Comp.Ualpha,Stru_Vol_alphabeta_Comp.Ubeta);
	}
	#else
	{
		SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta_Comp.Ualpha,Stru_Vol_alphabeta_Comp.Ubeta);
	}
	#endif
	// 更新寄存器
	SetEPWMRegister();  

	//-----------------------------------------------------------------------/
	if(Fault_Flag !=0)
	{
		MOTOR_STATE   = MC_POWERON;
		Bridge_Output_Off();
		Cnt = 0;
		LearnSuccessCnt = 0;
		Stru_LHall.Learn.LearnSuccessFlag = 0;
		Stru_LHall.Learn.Flag_CompenAngle_Determined = 0;
		Stru_LHall.Learn.Flag_ErrorNeed_Compen = 0;
		Stru_LHall.Learn.thetaerrorCompened = 0;
		Stru_LHall.Learn.thetaerrorPre = 0;
		Stru_LHall.Learn.deltaError = 0;
		Stru_LHall.Learn.thetaerror = 0;
		}
}




/*****************************************************************************
* Function Name  : FOC_TASK_CONVERGENCE
* Description    : 线性霍尔收敛状态机
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : 
******************************************************************************/
void FOC_TASK_CONVERGENCE()
{
	static int32_t cnt = 0;
	cnt++;

	if(cnt>(EPWM_FREQ>>2))
	{
		MOTOR_STATE = MC_IDLE;
		Bridge_Output_Off();
		cnt = 0;
	}
	
	SetEPWMDuty(16384,16384,16384);
	
  if(Fault_Flag)
  {
		cnt = 0;
		MOTOR_STATE = MC_POWERON;
  }
}

/*****************************************************************************
* Function Name  : FOC_TASK_IDLE
* Description    : FOC_IDLE状态机
* Function Call  : FOC中断调用
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_TASK_IDLE()
{
  Bridge_Output_Off();     

	if(Fault_Flag == 0)
	{
		#if (Config_Run_Mode == RUN_MD_LHALL && Config_HWTEST_MODE == HWTEST_OFF)
		{
			if(Stru_Foc.LHall_LearnState != LEARN_FINISHED)
			{
				MOTOR_STATE = MC_POWERON;
			}
		}
		#endif
		
		if(Stru_User.RunFlag == 1)
		{
			MOTOR_STATE = MC_INIT;
		}
	}
	else 
	{
		MOTOR_STATE = MC_POWERON;
	}
}


/*****************************************************************************
* Function Name  : FOC_TASK_INIT
* Description    : FOC_Init状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_TASK_INIT()
{
	//-------------------------------------------------------------------/	
	//Foc参数初始化
  Foc_Ctrl_Init();               
  
	//------------------------------------------------------------------------/	
	//三相的初始占空比设置为50%
	SetEPWMDuty(16384,16384,16384);
	//------------------------------------------------------------------------/	
	//状态切换
  #if (Config_HWTEST_MODE == HWTEST_OFF)
	{
		#if (Config_CHARGE_MODE==CHARGE_ENABLE)
		{
			MOTOR_STATE = MC_CHARGE; 
		}
		#else
		{
			#if (Config_Wind_Mode == Start_NoWind)
			{
				StartupFromStandstill();
			}
			#elif (Config_Wind_Mode == Start_Wind)
			{
				MOTOR_STATE = MC_WIND;
				Bridge_Output_On();
				Ob1_ParaRefresh(Stru_Wind.Kp,0,START_ADP_MAX,-START_ADP_MAX);
			}
			#endif
		}
		#endif
	}
  #else
	{
		MOTOR_STATE = MC_TEST;
		Bridge_Output_On();
	
	}
  #endif  

  if(Stru_User.RunFlag==0)
  {
    MOTOR_STATE = MC_IDLE;
  }
    
}

/*****************************************************************************
* Function Name  : FOC_TASK_CHARGE
* Description    : FOC_CHarge状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_TASK_CHARGE()
{
	SetEPWMDuty(16384,16384,16384);   //设置自举充电的占空比

	Stru_FocCnt.CntCharge ++;
	if(Stru_FocCnt.CntCharge <= (CHARGE_TIME * EPWM_FREQ / 3))
	{
		MC_PWM_Mask(DRIVER_ALPWM);			//充A相
	}
	//=======================================================================//	
	else if(Stru_FocCnt.CntCharge <=  (2 * CHARGE_TIME * EPWM_FREQ / 3))
	{
		MC_PWM_Mask(DRIVER_BLPWM);			//充B相
	}
	//=======================================================================//	
	else if(Stru_FocCnt.CntCharge  <=  (CHARGE_TIME * EPWM_FREQ ))
	{
		MC_PWM_Mask(DRIVER_CLPWM);			//充C相
	}	
	//-------------------------------------------------------------------/	
	// 充电完成
	if(Stru_FocCnt.CntCharge  >  (CHARGE_TIME * EPWM_FREQ ))
	{
		MC_PWM_Mask(DRIVER_OFF);
		Stru_FocCnt.CntCharge = 0;

		#if (Config_Wind_Mode == Start_NoWind)
		{
				StartupFromStandstill();
		}
		#elif (Config_Wind_Mode == Start_Wind)
		{
			MOTOR_STATE = MC_WIND;
			Bridge_Output_On();
			Ob1_ParaRefresh(Stru_Wind.Kp,0,START_ADP_MAX,-START_ADP_MAX);
		}
		#endif
	}	
	
	// 关机处理
  if(Stru_User.RunFlag==0)
  {
    MOTOR_STATE = MC_IDLE;
		Stru_FocCnt.CntCharge = 0;
  }
	if(Fault_Flag)
  {
    MOTOR_STATE = MC_POWERON;
  }
}

/*****************************************************************************
* Function Name  : FOC_TASK_TEST
* Description    : FOC_Test状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj、Lsy
******************************************************************************/
int32_t Iqref = 3000;
int16_t Angle;
uint32_t time1 = 0;
uint32_t time2 = 0;
int32_t Uqref= 5000;
int16_t thetaUd = 0;
int32_t SpeedRef = 0;
void FOC_TASK_TEST(void)
{
	#if (Config_HWTEST_MODE == HWTEST_PWM)
	{
		Bridge_Output_On();
	
		SetEPWMDuty(6553,9830,13107);
	}
	#elif (Config_HWTEST_MODE == HWTEST_VOLT_DRAG)
	{
		static int16_t DragACCcnt;
		Bridge_Output_On();
		
		//加速
		if(++DragACCcnt > Stru_Test.ACCCycle)
		{
			DragACCcnt = 0;
			
			if(Stru_Test.OmegaE < Stru_Test.OmegaEMax)
				Stru_Test.OmegaE +=1;
			if(Stru_Test.OmegaE > Stru_Test.OmegaEMax)
				Stru_Test.OmegaE -=1;	
			
			if(Stru_Test.DragCurr <= Stru_Test.OpenCurr_Final - Stru_Test.DeataCur)
				Stru_Test.DragCurr += Stru_Test.DeataCur;
			else if (Stru_Test.DragCurr >= Stru_Test.OpenCurr_Final + Stru_Test.DeataCur)
				Stru_Test.DragCurr -= Stru_Test.DeataCur;
			else
				Stru_Test.DragCurr = Stru_Test.OpenCurr_Final;
		}

		Stru_Vol_dq.Ud = 0;
		Stru_Vol_dq.Uq = Stru_Test.DragCurr;				
		//-------------------------------------------------------------------/	

		Stru_Foc.Elec_Theta += Stru_Test.OmegaE;
		Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);
		
		//-------------------------------------------------------------------/	
		//	反Park变换
		RevPark();                                                                             

		//-------------------------------------------------------------------/	
		//SVPWM计算、更新
		#if (Config_Shunt_Mode == Double_Shunt)
		{
			SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha +Stru_DeadComp.s32_Comp_Alpha,Stru_Vol_alphabeta.Ubeta+Stru_DeadComp.s32_Comp_Beta);
		}
		#else
		{
				
			SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
		}
		#endif                                                                         
		SetEPWMRegister();    	
	}
	#elif (Config_HWTEST_MODE == HWTEST_CURR_DRAG)
	{
		static int16_t DragACCcnt;
		Bridge_Output_On();
		
		//加速
		if(++DragACCcnt > Stru_Test.ACCCycle)
		{
			DragACCcnt = 0;
			
			if(Stru_Test.OmegaE < Stru_Test.OmegaEMax)
				Stru_Test.OmegaE +=1;
			if(Stru_Test.OmegaE > Stru_Test.OmegaEMax)
				Stru_Test.OmegaE -=1;	
			
			if(Stru_Test.DragCurr <= Stru_Test.OpenCurr_Final - Stru_Test.DeataCur)
				Stru_Test.DragCurr += Stru_Test.DeataCur;
			else if (Stru_Test.DragCurr >= Stru_Test.OpenCurr_Final + Stru_Test.DeataCur)
				Stru_Test.DragCurr -= Stru_Test.DeataCur;
			else
				Stru_Test.DragCurr = Stru_Test.OpenCurr_Final;
		}
		//-------------------------------------------------------------------/	
		Stru_Cur_dqRef.Id = 0;
		Stru_Cur_dqRef.Iq = Stru_Test.DragCurr;

		// 电流环处理
		Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));        
		FOC_VsLimit();                                                                             
		Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq));         
	
		//-------------------------------------------------------------------/	

//		Stru_Foc.Elec_Theta += Stru_Test.OmegaE;
		Stru_Foc.Elec_Theta = Stru_Foc.Elec_Theta + 10;
		Angle = Angle + 10;
		Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);
		
		//-------------------------------------------------------------------/	
		//	反Park变换
		RevPark();                                                                             
		
		//-------------------------------------------------------------------/	
		//SVPWM计算、更新
		#if (Config_Shunt_Mode == Double_Shunt)
		{
			SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha +Stru_DeadComp.s32_Comp_Alpha,Stru_Vol_alphabeta.Ubeta+Stru_DeadComp.s32_Comp_Beta);
		}
		#else
		{
			SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
		}
		#endif                                                                        
		SetEPWMRegister();    	
	}
	#elif (Config_HWTEST_MODE == HWTEST_Hall_Learn)
	{
		// 霍尔角度自学习
		Stru_Sample.LHallBetaOffset = 0;
		Stru_Sample.LHallAlphaOffset = 0;
		FOC_LHall_LEARN();
	}
	#elif (Config_HWTEST_MODE == HWTEST_Debug)
	{

//		Stru_Ob2.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);              
//		Stru_Ob2.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
//		Stru_Ob2.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
//		Stru_Ob2.Ibeta = Stru_Cur_alphabeta.Ibeta;          
//		Ob2_Cal();		 
//		Stru_Foc.Elec_Theta = Stru_Ob2.Theta+Stru_Foc.OffsetTheta;  
		
//		if(Stru_LHall.Hall_Seq == HALL_SEQ_FORWARD)
//		{
//			Stru_LHall.Ualpha =  Stru_Sample.LHallAlphaAD - Stru_Sample.LHallAlphaOffset;
//			Stru_LHall.Ubeta = Stru_Sample.LHallBetaAD - Stru_Sample.LHallBetaOffset;
//		}
//		else
//		{
//			Stru_LHall.Ualpha = Stru_Sample.LHallBetaAD - Stru_Sample.LHallBetaOffset;
//			Stru_LHall.Ubeta =  Stru_Sample.LHallAlphaAD - Stru_Sample.LHallAlphaOffset;
//		}
//		Stru_LHall.Ubeta = Stru_LHall.Ubeta;
//		LHall_Theta_Cal(&Stru_LHall, Stru_LHall.Ualpha<<5, Stru_LHall.Ubeta<<5);
//		Stru_Foc.Elec_Theta = Stru_LHall.Theta + Stru_LHall.LHall_OffsetTheta; 


//从SPI通信获取转子机械位置
		PORT_ClrBit(PORT2,PIN3);
		Angle = SPI_KTH7801_Read(CMD_READ_ANGLE);
		PORT_SetBit(PORT2,PIN3);
//对机械位置进行锁相，获取电角度
		GetEncoderAngle(&Stru_Encoder	,Angle);
		Stru_Foc.Elec_Theta = Stru_Encoder.ThetaE;  
		Stru_Foc.Elec_Omega = Stru_Encoder.Omega;
		Stru_Foc.OmegaPU = Stru_Encoder.OmegaPU;
		Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);				
		Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);
		
		
		
//		Stru_Foc.Elec_Theta = 0;
////		Stru_Foc.Elec_Theta = Stru_Foc.Elec_Theta + 50;
//		Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);
		//-------------------------------------------------------------------/	
		//启动电流给定
		Stru_Cur_dqRef.Iq = Iqref;
		Stru_Cur_dqRef.Id = 0;

		//-------------------------------------------------------------------/	
		// 电流环处理
		Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));
		FOC_VsLimit();                                                                              
		Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq));
		
//		Stru_Vol_dq.Ud = 0;
//		Stru_Vol_dq.Uq = 5000;


		
//		static uint32_t cnt = 0;
//		cnt++;
//		if(cnt>5)
//		{
//			cnt = 0;
//			Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_OL,SpeedRef - Stru_Foc.OmegaPUFiltered);
//		}


		//-------------------------------------------------------------------/	
		// 反Park变换
		RevPark();                                                                             
		
		//-------------------------------------------------------------------/	
		//SVPWM计算、更新
		#if (Config_Shunt_Mode == Double_Shunt)
		{
			SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
		}
		#else
		{
			SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
		}
		#endif   
		
		//更新EPWM寄存器
		SetEPWMRegister();                                                                      
	}
	#endif


	//-------------------------------------------------------------------/
  if(Stru_User.RunFlag==0)
  {
		#if ((Config_HWTEST_MODE == HWTEST_VOLT_DRAG) || (Config_HWTEST_MODE == HWTEST_CURR_DRAG))
			Stru_Test.DragCurr = Stru_Test.OpenCurr_Final >> 1;;
			Stru_Test.OmegaE = 1;
		#endif
    MOTOR_STATE = MC_IDLE;
  }
	if(Fault_Flag !=0 )
	{
		MOTOR_STATE = MC_POWERON;
		Bridge_Output_Off();
	}

}



/*****************************************************************************
* Function Name  : FOC_TASK_WIND
* Description    : FOC_Wind状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
int32_t deltaTheta = 0;
void FOC_TASK_WIND(void)
{
  //-------------------------------------------------------------------/	
	Stru_Wind.Ed = ((Stru_Sincos.Cos * Stru_Ob1.Sig5)>>15)  + ((Stru_Sincos.Sin * Stru_Ob1.Sig6)>>15);  
	Stru_Wind.Eq = ((Stru_Sincos.Cos * Stru_Ob1.Sig6)>>15)  - ((Stru_Sincos.Sin * Stru_Ob1.Sig5)>>15);

	Stru_Wind.Ud = Stru_Wind.Ed + (Stru_Motor.RsPU*Stru_Cur_dq.Id>>15) - ((Stru_Ob1.OmegaPu*Stru_Motor.LqPU>>15)*Stru_Cur_dq.Iq>>15);
	Stru_Wind.Uq = Stru_Wind.Eq + (Stru_Motor.RsPU*Stru_Cur_dq.Iq>>15) + ((Stru_Ob1.OmegaPu*Stru_Motor.LdPU>>15)*Stru_Cur_dq.Id>>15);
	
	Stru_Cur_dqRef.Id = 0;
	Stru_Cur_dqRef.Iq = 0;
	
	
	#if (CONFIG_WINDCHECK_MODE == WINDCHECK_MODE1)
	{
		Stru_Vol_dq.Ud = 0;
		Stru_Vol_dq.Uq = 0;
	}
	#else
	{
		Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));
		FOC_VsLimit();                                                                              
		Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq));
	}
	#endif
	

	
	//-------------------------------------------------------------------/	
	// 反Park变换
  RevPark();                                                                             

	//-------------------------------------------------------------------/	
	//SVPWM计算、更新
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#else
	{
		SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#endif   
	
	//更新EPWM寄存器
  SetEPWMRegister();      

	
	//顺逆风判定
	switch(Stru_Wind.State)
	{
		case SPEED_OB:
		{
			static int32_t Cnt_SPEED_OB = 0;    //观测器稳定计数器
			Cnt_SPEED_OB++;
			
			if(Cnt_SPEED_OB>5000)				//5000个Cnt之后，观测器已经稳定，才开始判定顺逆风
			{
				if( (Stru_Ob1.OmegaPu < Stru_Wind.LowSpeedJudgeThreshold) && (Stru_Ob1.OmegaPu > -Stru_Wind.LowSpeedJudgeThreshold) )
				{
					Stru_Wind.Cnt_LowSPeedJudge++;
					if(Stru_Wind.Cnt_LowSPeedJudge>200)
					{
						Stru_Wind.State = LOWSPEED;
						Stru_Wind.Cnt_LowSPeedJudge = 0;
						Cnt_SPEED_OB = 0;
						Stru_Wind.Cnt_TailWindJudge = 0;
						Stru_Wind.Cnt_HeadWindJudge = 0;
					}
					if(Stru_Wind.Cnt_LowSPeedJudge>50)
					{
						Stru_Wind.Cnt_TailWindJudge = 0;
						Stru_Wind.Cnt_HeadWindJudge = 0;
					}
				}
				else if(Stru_Ob1.OmegaPu>Stru_Wind.LowSpeedJudgeThreshold)
				{
					Stru_Wind.Cnt_TailWindJudge++;
					if(Stru_Wind.Cnt_TailWindJudge>200)
					{
						Stru_Wind.State = TAILWIND;
						Stru_Wind.Cnt_TailWindJudge = 0;
						Cnt_SPEED_OB = 0;
						Stru_Wind.Cnt_LowSPeedJudge = 0;
						Stru_Wind.Cnt_HeadWindJudge = 0;
					}
					if(Stru_Wind.Cnt_TailWindJudge>50)
					{
						Stru_Wind.Cnt_LowSPeedJudge = 0;
						Stru_Wind.Cnt_HeadWindJudge = 0;
					}
				}
				else    //   Stru_Ob1.OmegaPu<-Stru_Wind.LowSpeedJudgeThreshold
				{
					Stru_Wind.Cnt_HeadWindJudge++;
					if(Stru_Wind.Cnt_HeadWindJudge>200)
					{
						Stru_Wind.State = HEADWIND;
						Stru_Wind.Cnt_HeadWindJudge = 0;
						Cnt_SPEED_OB = 0;
						Stru_Wind.Cnt_TailWindJudge = 0;
						Stru_Wind.Cnt_LowSPeedJudge = 0;
					}
					if(Stru_Wind.Cnt_HeadWindJudge>50)
					{
						Stru_Wind.Cnt_TailWindJudge = 0;
						Stru_Wind.Cnt_LowSPeedJudge = 0;
					}
				}
			}
		}
			break;
		case TAILWIND:													//顺风情况
		{
			if(Stru_Ob1.OmegaPu>Stru_Wind.LowSpeedJudgeThreshold)     
			{
				Stru_Wind.Cnt_HighSpeedJudge++;
				if(Stru_Wind.Cnt_HighSpeedJudge>200)				//判定为强顺风
				{
					MOTOR_STATE = MC_RUN;
					Stru_Wind.Cnt_HighSpeedJudge = 0;
					Stru_Wind.Cnt_LowSPeedJudge = 0;
					
					//状态机切换时的衔接操作
					SwitchFromWindToRun();

				}
				if(Stru_Wind.Cnt_HighSpeedJudge>50)
				{
					Stru_Wind.Cnt_LowSPeedJudge = 0;
				}
			}
			else                                                               
			{
				Stru_Wind.Cnt_LowSPeedJudge++;
				if(Stru_Wind.Cnt_LowSPeedJudge>200)
				{
					Stru_Wind.State = LOWSPEED;							//判定为小顺风
					Stru_Wind.Cnt_LowSPeedJudge = 0;
					Stru_Wind.Cnt_HighSpeedJudge = 0;
				}
				if(Stru_Wind.Cnt_LowSPeedJudge>50)
				{
					Stru_Wind.Cnt_HighSpeedJudge = 0;
				}
			}	
		}
			break;

		case HEADWIND:
		{
			if(Stru_Ob1.OmegaPu<-Stru_Wind.LowSpeedJudgeThreshold)     
			{
				Stru_Wind.Cnt_HighSpeedJudge++;
				if(Stru_Wind.Cnt_HighSpeedJudge>200)				//判定为强逆风，一直等待
				{
					/*
					强逆风情况下，一直等待  或者插入强逆风处理
					*/
					Stru_Wind.Cnt_HighSpeedJudge = 0;
					Stru_Wind.Cnt_LowSPeedJudge = 0;
				}
				
				if(Stru_Wind.Cnt_HighSpeedJudge>50)
				{
					Stru_Wind.Cnt_LowSPeedJudge = 0;
				}
			}
			else                                                               
			{
				Stru_Wind.Cnt_LowSPeedJudge++;
				if(Stru_Wind.Cnt_LowSPeedJudge>200)
				{
					Stru_Wind.State = LOWSPEED;							//判定为小逆风
					Stru_Wind.Cnt_LowSPeedJudge = 0;
					Stru_Wind.Cnt_HighSpeedJudge = 0;
				}
				if(Stru_Wind.Cnt_LowSPeedJudge>50)
				{
					Stru_Wind.Cnt_HighSpeedJudge = 0;
				}
			}	
		}
			break;
		
		case LOWSPEED:
		{
			Stru_Wind.Cnt_StillJudge++;

			Stru_Wind.ThetaTotal = Stru_Wind.ThetaTotal + Stru_Ob1.Omega;
			
			if(Stru_Wind.ThetaTotal > Stru_Wind.ThetaMax)
			{
				Stru_Wind.ThetaMax = Stru_Wind.ThetaTotal;
			}
			if(Stru_Wind.ThetaTotal < Stru_Wind.ThetaMin)
			{
				Stru_Wind.ThetaMin = Stru_Wind.ThetaTotal;
			}
			deltaTheta = Stru_Wind.ThetaMax - Stru_Wind.ThetaMin;

			if( Stru_Wind.Cnt_StillJudge > Stru_Wind.StillJudgeThreShold_Cnt )
			{
				Stru_Wind.Cnt_StillJudge = 0;
				if( deltaTheta > Stru_Wind.StillJudgeThreShold_ThetaError)   //速度过大，等待静止下来
				{
					Stru_Wind.Cnt_StillJudge = 0;
					Stru_Wind.ThetaTotal = 0;
					Stru_Wind.ThetaMax = 0;
					Stru_Wind.ThetaMin = 0;
				}
				else			//认为已经静止     
				{
					Ob1_ParaRefresh(Stru_Para.Start.PLLKp,Stru_Para.Start.PLLKi,START_ADP_MAX,START_ADP_MIN);
					StartupFromStandstill();
				}
			}
		}
			break;
		
		
		
		default:
			break;	
	}

	//-------------------------------------------------------------------/
	// 停机判断
  if(Stru_User.RunFlag==0)
  {
    MOTOR_STATE = MC_IDLE;
  }	
	
}
/*****************************************************************************
* Function Name  : FOC_TASK_IPD
* Description    : FOC_IPD状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_TASK_IPD()
{
//	// 更改触发机制
	MC_IPD_Ctrl_Init();
	Stru_Foc.IPD_Theta = FOC_IPD_Square_SWMode(&Stru_IPD_Pulse);
	// 发波后恢复触发机制
	MC_IPD_Ctrl_Recv();

	// 缺相判断
	#if (PHASELOSS_PROTECT_ENABLE)
	{
		if((Stru_IPD_Pulse.IPD_Cur_AdMin - Stru_Sample.IBusOffset) < 10)
		{
			Fault_PhaseLoss = 1;
		}		
	}
	#endif
	//---------------------------------------------------------------------------/
	// 根据IPD结果给观测器赋初始位置
	Stru_Foc.Elec_Theta = Stru_Foc.IPD_Theta;
	Stru_Ob1.Theta = Stru_Foc.IPD_Theta;

	SetEPWMDuty(16384,16384,16384);

	Stru_Cur_dqRef.Iq = Stru_Para.Start.AlignCurMin;

	MOTOR_STATE = MC_STARTUP;
	Bridge_Output_On();
}
/*****************************************************************************
* Function Name  : FOC_TASK_ALIGN
* Description    : FOC_Align状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
int32_t RsPu = 0;
void FOC_TASK_ALIGN()
{
	//给定角度
	Stru_Foc.Elec_Theta = 0;  
	//正余弦计算     
	Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);

	//-------------------------------------------------------------------/	
	//启动电流给定
  Stru_Cur_dqRef.Id = 0;

  Stru_FocCnt.CntAlign1++;
  if( Stru_FocCnt.CntAlign1 > Stru_Para.Start.AlignCurIncInterval)
  {
    Stru_FocCnt.CntAlign1 = 0;
    Stru_Cur_dqRef.Iq = Stru_Cur_dqRef.Iq + Stru_Para.Start.AlignCurInc;
    if(Stru_Cur_dqRef.Iq > Stru_Para.Start.AlignCurMax)
    {
      Stru_Cur_dqRef.Iq = Stru_Para.Start.AlignCurMax;
    }
  }

	//-------------------------------------------------------------------/	
	// 电流环处理
  Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));
  FOC_VsLimit();                                                                              
  Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq));
	
	int32_t temp = Stru_Vol_dqSample.Uq*Stru_Coff.ModuSig2PU_Vphase>>14;
	RsPu = HWDivider(temp*32768,Stru_Cur_dq.Iq);
	
	//-------------------------------------------------------------------/	
	// 反Park变换
  RevPark();                                                                             
	
	//-------------------------------------------------------------------/	
	//SVPWM计算、更新
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#else
	{
		SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#endif   
	
	//更新EPWM寄存器
  SetEPWMRegister();                                                                      

	//状态机切换条件
  Stru_FocCnt.CntAlign2++;
	if( (Stru_FocCnt.CntAlign2>Stru_Para.Start.AlignTime) && (Stru_Cur_dqRef.Iq == Stru_Para.Start.AlignCurMax) )
	{
		Stru_FocCnt.CntAlign2 = 0;
		MOTOR_STATE = MC_STARTUP;	
		Stru_Cur_dqRef.Iq = Stru_Para.Start.StartCurMin; 
		
		Stru_Foc.Elec_Theta = 0;
		Stru_Ob1.Theta = 0;

	}
	//---------------------------------------------------------------------------/
	// 关机
  if(Stru_User.RunFlag == 0)
  {
    MOTOR_STATE = MC_IDLE;
		Stru_FocCnt.CntAlign2 = 0;
  }
	if(Fault_Flag)
  {
    MOTOR_STATE = MC_POWERON;
  }
}

/*****************************************************************************
* Function Name  : FOC_TASK_SLSTART
* Description    : FOC_Start状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_TASK_START()
{
	//-------------------------------------------------------------------/	
	//启动电流给定
  Stru_FocCnt.CntStartup1++;
  if( Stru_FocCnt.CntStartup1 > Stru_Para.Start.StartCurIncInterval)
  {
    Stru_FocCnt.CntStartup1 = 0;
    Stru_Cur_dqRef.Iq = Stru_Cur_dqRef.Iq + Stru_Para.Start.StartCurInc;
    if(Stru_Cur_dqRef.Iq > Stru_Foc.Curr_Is_Ref)
    {
      Stru_Cur_dqRef.Iq = Stru_Foc.Curr_Is_Ref;
    }
  }

	//-------------------------------------------------------------------/	
	// 电流环处理
  Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));
  FOC_VsLimit();                                                                              
  Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq));
	
	//-------------------------------------------------------------------/	
	// 反Park变换
  RevPark();                                                                             

	//-------------------------------------------------------------------/	
	//SVPWM计算、更新
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#else
	{
		SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#endif   
	
	//更新EPWM寄存器
  SetEPWMRegister();                                                                      

	//-------------------------------------------------------------------/	  
	//启动成功判断
  if(Stru_Meas.PhyValue.MecSpeed>Stru_Para.Start.Speed_Close)
  {
    Stru_FocCnt.CntStartup2++;
    if(Stru_FocCnt.CntStartup2 > Stru_Para.Start.HoldTime)
    {
      Stru_FocCnt.CntStartup2 = 0;
			#if(Config_Run_Mode == RUN_MD_SENSORLESS)
			{
				MOTOR_STATE = MC_SW;
			}
			#else
			{
				MOTOR_STATE = MC_RUN;
				SwitchToRun();
			}
			#endif
    }
  }

	//-------------------------------------------------------------------/
	// 停机判断
  if(Stru_User.RunFlag==0)
  {
    MOTOR_STATE = MC_IDLE;
  }
}

/*****************************************************************************
* Function Name  : FOC_SW
* Description    : FOC_SW状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_TASK_SW()
{
	
	//-------------------------------------------------------------------/
	// 电流给定(外环给定)
//  Stru_Cur_dqRef.Iq  = Stru_Foc.Curr_Is_Ref;
//  Stru_Cur_dqRef.Id = 0;

	//-------------------------------------------------------------------/
	// 电流环

	if(Stru_Cur_dqRef.Id>0)  Stru_Cur_dqRef.Id--;
	if(Stru_Cur_dqRef.Id<0)  Stru_Cur_dqRef.Id++;
	
	if(Stru_Cur_dqRef.Iq > Stru_Foc.Curr_Is_Ref)  Stru_Cur_dqRef.Iq--;
	if(Stru_Cur_dqRef.Iq < Stru_Foc.Curr_Is_Ref)  Stru_Cur_dqRef.Iq++;
	
  Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));         
  FOC_VsLimit();                                                                              
  Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq)); 
	Stru_Foc.VsAmp = HWSqrt(Stru_Vol_dq.Ud * Stru_Vol_dq.Ud + Stru_Vol_dq.Uq*Stru_Vol_dq.Uq);
	//-------------------------------------------------------------------/
	//反Park变换
  RevPark();                             
	
	//-------------------------------------------------------------------/	
	// 计算空间矢量
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#else
	{
		SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#endif

	// 更新寄存器
  SetEPWMRegister();  

  //-------------------------------------------------------------------/
	// 状态判断
	Stru_FocCnt.CntSw ++;
	
	if((Stru_FocCnt.CntSw > (Stru_Para.Start.SwitchTime>>1))&&(Stru_Cur_dqRef.Id == 0))
	{
		MOTOR_STATE = MC_RUN;
		Stru_FocCnt.CntSw = 0;
		
		SwitchToRun();
	}
  //-------------------------------------------------------------------/	
	// 停机判断
  if(Stru_User.RunFlag==0)
  {
    MOTOR_STATE = MC_IDLE;
		Stru_FocCnt.CntSw = 0;
  }
}

/*****************************************************************************
* Function Name  : FOC_TASK_RUN
* Description    : FOC_RUN状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_TASK_RUN()
{
	//-------------------------------------------------------------------/
	// 电流环
  Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));         
  FOC_VsLimit();                                                                              
  Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq)); 
	Stru_Foc.VsAmp = HWSqrt(Stru_Vol_dq.Ud * Stru_Vol_dq.Ud + Stru_Vol_dq.Uq*Stru_Vol_dq.Uq);
	//-------------------------------------------------------------------/
	//纹波电压补偿
	#if (Config_VbusRipple_Comp == Vbus_Comp_ENABLE)
	{
		Stru_Vol_dq.Ud = (Stru_Vol_dq.Ud * Stru_VbusRippleComp.CompGain_Q14) >> 14;
		Stru_Vol_dq.Uq = (Stru_Vol_dq.Uq * Stru_VbusRippleComp.CompGain_Q14) >> 14;
	}
	#endif
	//-------------------------------------------------------------------/
	//反Park变换
  RevPark();                             
	
	//-------------------------------------------------------------------/
	// 死区补偿
	#if (CONFIG_DeadComp_MODE == DeadComp_Enable)
	{
		FOC_DeadComp_Cal();
		Stru_Vol_alphabeta_Comp.Ualpha = Stru_Vol_alphabeta.Ualpha + Stru_DeadComp.s32_Comp_Alpha;
		Stru_Vol_alphabeta_Comp.Ubeta  = Stru_Vol_alphabeta.Ubeta + Stru_DeadComp.s32_Comp_Beta;
	}
	#else
	{
		Stru_Vol_alphabeta_Comp.Ualpha = Stru_Vol_alphabeta.Ualpha;
		Stru_Vol_alphabeta_Comp.Ubeta  = Stru_Vol_alphabeta.Ubeta;
	}
	#endif
	
	//-------------------------------------------------------------------/	
	// 计算空间矢量
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta_Comp.Ualpha,Stru_Vol_alphabeta_Comp.Ubeta);
	}
	#else
	{
		SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta_Comp.Ualpha,Stru_Vol_alphabeta_Comp.Ubeta);
	}
	#endif

	// 更新寄存器
  SetEPWMRegister();  

  //-------------------------------------------------------------------/	
	// 停机判断
  if((Stru_User.RunFlag==0) || (Stru_Para.Mode.Dir != Flag_MotorDir))
  {
    MOTOR_STATE = MC_STOP;
  }
	
	if(Fault_Flag !=0 )
	{
		MOTOR_STATE = MC_POWERON;
		Bridge_Output_Off();
	}

}

/*****************************************************************************
* Function Name  : FOC_STOP
* Description    : FOC_STOP状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj、Lsy
******************************************************************************/
void FOC_TASK_STOP()
{
	
	Stru_FocCnt.CntStop ++;
	//-------------------------------------------------------------------/
	Stru_Foc.Curr_Is_Ref = 0;
	// 电流给定
	if(Stru_Cur_dqRef.Id > 0 )	Stru_Cur_dqRef.Id --;
	if(Stru_Cur_dqRef.Id < 0 )	Stru_Cur_dqRef.Id ++;	
	if(Stru_Cur_dqRef.Iq > 0)		Stru_Cur_dqRef.Iq --;
	if(Stru_Cur_dqRef.Iq < 0)		Stru_Cur_dqRef.Iq ++;
	
	//-------------------------------------------------------------------/
	// 电流环
  Stru_Vol_dq.Ud = PI_Controller(&Stru_PI_Id,(Stru_Cur_dqRef.Id - Stru_Cur_dq.Id));         
  FOC_VsLimit();                                                                              
  Stru_Vol_dq.Uq = PI_Controller(&Stru_PI_Iq,(Stru_Cur_dqRef.Iq - Stru_Cur_dq.Iq)); 
	
	//-------------------------------------------------------------------/
	//反Park变换
  RevPark();                                                                              
	
	//-------------------------------------------------------------------/	
	// 计算空间矢量
	#if (Config_Shunt_Mode == Double_Shunt)
	{
		SVPWM_DoubleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#else
	{
		SVPWM_SingleShunt(&Stru_SVPWM,Stru_Vol_alphabeta.Ualpha,Stru_Vol_alphabeta.Ubeta);
	}
	#endif 
	// 更新寄存器
  SetEPWMRegister();  

	//-------------------------------------------------------------------/	
	// 停机判断
	#if (MOTOR_STOP_MODE == MOTOR_FREE_STOP)
	{
		if((Stru_FocCnt.CntStop > Stru_Time.Motor_PowerDown) || (ABSFUN(Stru_Meas.PhyValue.MecSpeed) < MOTOR_SPEED_STOP) || Fault_Flag)
		{
			Bridge_Output_Off();     
			MOTOR_STATE = MC_IDLE;
			Stru_FocCnt.CntStop = 0;
		}
	}
	#else
	{
		if(Stru_Meas.PhyValue.MecSpeed < BRAKE_STOP_SPEED)
		{
			Bridge_Output_Off();     
			MOTOR_STATE = MC_BRAKE;
			Stru_FocCnt.CntStop = 0;
		}	
	}
	#endif

	
}

/*****************************************************************************
* Function Name  : FOC_BRAKE
* Description    : FOC_BRAKE状态机函数
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建		Lsy	
******************************************************************************/
void FOC_TASK_BRAKE()
{
	Stru_FocCnt.CntBrake ++;

	// 下管全开刹车
  Bridge_Output_Down();
	
	//下管占空比刹车，可能存在母线过冲!
	//DutyBrake(500);
	
	// 状态切换
	if(Fault_Flag || (Stru_FocCnt.CntBrake > Stru_Time.Motor_Brake))		// || Stru_User.RunFlag 
	{
		MOTOR_STATE = MC_IDLE;
		Bridge_Output_Off();
		Stru_FocCnt.CntBrake = 0;
	}
}


/*****************************************************************************
* Function Name  : FOC_RotorPosition_Detection
* Description    : 无感观测器调用
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
                   V0.2    2025/01/07    新建			Xh
******************************************************************************/
void FOC_RotorPosition_Detection(void)
{
	#if(Config_HWTEST_MODE == HWTEST_OFF)
	{
		#if(Config_Run_Mode == RUN_MD_LHALL)
		{
			if(MOTOR_STATE != MC_POWERON && MOTOR_STATE != MC_LHALL_LEARN)
			{
				if(Stru_LHall.Hall_Seq == HALL_SEQ_FORWARD)
				{
					Stru_LHall.Ualpha =  Stru_Sample.LHallAlphaAD - Stru_Sample.LHallAlphaOffset;
					Stru_LHall.Ubeta = Stru_Sample.LHallBetaAD - Stru_Sample.LHallBetaOffset;
				}
				else
				{
					Stru_LHall.Ualpha = Stru_Sample.LHallBetaAD - Stru_Sample.LHallBetaOffset;
					Stru_LHall.Ubeta =  Stru_Sample.LHallAlphaAD - Stru_Sample.LHallAlphaOffset;
				}
				LHall_Theta_Cal(&Stru_LHall, Stru_LHall.Ualpha<<5, Stru_LHall.Ubeta<<5);
				Stru_Foc.Elec_Theta = Stru_LHall.ThetaE + Stru_LHall.LHall_OffsetTheta;   
				Stru_Foc.Elec_Omega = Stru_LHall.QPLLQ30.Omega>>15;
				Stru_Foc.OmegaPU = Stru_LHall.QPLLQ30.OmegaPU;
				Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);				
				Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);
			}
		}
		#else
		{
			if(MOTOR_STATE==MC_IDLE||(MOTOR_STATE==MC_INIT)||(MOTOR_STATE==MC_CHARGE)||
				(MOTOR_STATE==MC_IPD)||(MOTOR_STATE==MC_ALIGN)|| (MOTOR_STATE==MC_CONVERGENCE)||  
				(MOTOR_STATE==MC_LHALL_LEARN) ||(MOTOR_STATE==MC_POWERON))
			{
				Stru_Foc.Elec_Omega = 0;
				Stru_Foc.OmegaPU = 0;
				Stru_Foc.Elec_Theta = Stru_Ob1.Theta+Stru_Foc.OffsetTheta;  
				Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);		
			}
			else if(MOTOR_STATE == MC_WIND || MOTOR_STATE == MC_STARTUP)
			{
				Stru_Ob1.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);              
				Stru_Ob1.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
				Stru_Ob1.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
				Stru_Ob1.Ibeta = Stru_Cur_alphabeta.Ibeta;          
				Ob1_Cal();		
				
				Stru_Sincos = Stru_Ob1.SinCos;  
				Stru_Foc.Elec_Omega = Stru_Ob1.Omega;
				Stru_Foc.OmegaPU = Stru_Ob1.OmegaPu;
				Stru_Foc.Elec_Theta = Stru_Ob1.Theta+Stru_Foc.OffsetTheta;  
				Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);		
			}
			else if(MOTOR_STATE == MC_SW)
			{
				// 启动观测器1
				Stru_Ob1.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);              
				Stru_Ob1.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
				Stru_Ob1.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
				Stru_Ob1.Ibeta = Stru_Cur_alphabeta.Ibeta;          
				Ob1_Cal();		
				
				// 运行观测器1或2或3
				#if (Config_Obser_Run == Run_OBSER1)
				Stru_Sincos = Stru_Ob1.SinCos;    
				Stru_Foc.Elec_Theta = Stru_Ob1.Theta + Stru_Foc.OffsetTheta;  
				Stru_Foc.Elec_Omega = Stru_Ob1.Omega;
				Stru_Foc.OmegaPU = Stru_Ob1.OmegaPu;
				Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);				
				#elif (Config_Obser_Run == Run_OBSER2)
				{
					Stru_Ob2.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);             
					Stru_Ob2.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
					Stru_Ob2.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
					Stru_Ob2.Ibeta = Stru_Cur_alphabeta.Ibeta;                          
					Ob2_Cal();
					if(Stru_FocCnt.CntSw<(Stru_Para.Start.SwitchTime>>1))
					{
						Stru_Sincos = Stru_Ob1.SinCos;    
						Stru_Foc.Elec_Theta = Stru_Ob1.Theta + Stru_Foc.OffsetTheta;  
						Stru_Foc.Elec_Omega = Stru_Ob1.Omega;
						Stru_Foc.OmegaPU = Stru_Ob1.OmegaPu;
						Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);					
					}
					else
					{
						if(Stru_FocCnt.CntSw == (Stru_Para.Start.SwitchTime>>1))
						{
							Coordinate_Switch(Stru_Ob1.Theta, Stru_Ob2.Theta);
						}
						Stru_Sincos = Stru_Ob2.SinCos;    
						Stru_Foc.Elec_Theta = Stru_Ob2.Theta+Stru_Foc.OffsetTheta; 
						Stru_Foc.Elec_Omega = Stru_Ob2.Omega;
						Stru_Foc.OmegaPU = Stru_Ob2.OmegaPu;
						Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);		
					}
				}
				#else //(Config_Obser_Run == Run_OBSER3)
				{
					Stru_Ob3.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);             
					Stru_Ob3.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
					Stru_Ob3.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
					Stru_Ob3.Ibeta = Stru_Cur_alphabeta.Ibeta;                          
					Ob3_Cal();
					
					if(Stru_FocCnt.CntSw<(Stru_Para.Start.SwitchTime>>1))
					{
						Stru_Sincos = Stru_Ob1.SinCos;    
						Stru_Foc.Elec_Theta = Stru_Ob1.Theta + Stru_Foc.OffsetTheta;  
						Stru_Foc.Elec_Omega = Stru_Ob1.Omega;
						Stru_Foc.OmegaPU = Stru_Ob1.OmegaPu;
						Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);		
					}
					else
					{
						if(Stru_FocCnt.CntSw == (Stru_Para.Start.SwitchTime>>1))
						{
							Coordinate_Switch(Stru_Ob1.Theta, Stru_Ob3.Theta);
						}
						Stru_Sincos = Stru_Ob3.SinCos;    
						Stru_Foc.Elec_Theta = Stru_Ob3.Theta+Stru_Foc.OffsetTheta;  
						Stru_Foc.Elec_Omega = Stru_Ob3.Omega;
						Stru_Foc.OmegaPU = Stru_Ob3.OmegaPu;
						Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);		
					}
				}
				#endif	
			}
			else 
			{
				#if (Config_Obser_Run == Run_OBSER1)
				Stru_Ob1.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);              
				Stru_Ob1.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
				Stru_Ob1.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
				Stru_Ob1.Ibeta = Stru_Cur_alphabeta.Ibeta;          
				Ob1_Cal();		
				Stru_Sincos = Stru_Ob1.SinCos;  
				Stru_Foc.Elec_Omega = Stru_Ob1.Omega;
				Stru_Foc.OmegaPU = Stru_Ob1.OmegaPu;
				Stru_Foc.Elec_Theta = Stru_Ob1.Theta+Stru_Foc.OffsetTheta;  
				Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);			
				#elif (Config_Obser_Run == Run_OBSER2)
				Stru_Ob2.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);             
				Stru_Ob2.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
				Stru_Ob2.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
				Stru_Ob2.Ibeta = Stru_Cur_alphabeta.Ibeta;                          
				Ob2_Cal();
				Stru_Sincos = Stru_Ob2.SinCos;
				Stru_Foc.Elec_Omega = Stru_Ob2.Omega;
				Stru_Foc.OmegaPU = Stru_Ob2.OmegaPu;
				Stru_Foc.Elec_Theta = Stru_Ob2.Theta+Stru_Foc.OffsetTheta;                                  
				Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);			
				#else
				Stru_Ob3.Ualpha = (Stru_Vol_alphabeta.Ualpha*Stru_Coff.ModuSig2PU_Vphase>>14);             
				Stru_Ob3.Ubeta = (Stru_Vol_alphabeta.Ubeta*Stru_Coff.ModuSig2PU_Vphase>>14); 
				Stru_Ob3.Ialpha = Stru_Cur_alphabeta.Ialpha;                         
				Stru_Ob3.Ibeta = Stru_Cur_alphabeta.Ibeta;                          
				Ob3_Cal();
				Stru_Sincos = Stru_Ob3.SinCos;
				Stru_Foc.Elec_Theta = Stru_Ob3.Theta+Stru_Foc.OffsetTheta; 
				Stru_Foc.Elec_Omega = Stru_Ob3.Omega;
				Stru_Foc.OmegaPU = Stru_Ob3.OmegaPu;
				Stru_Foc.OmegaPUFiltered = LPF_Cal(&LPFOmegaPU,Stru_Foc.OmegaPU, _Fc_FOR_OMEGAPU_);				
				#endif	
			}
		}
		#endif
	}
	#endif
}

/*****************************************************************************
* Function Name  : FOC_Task_MidFre
* Description    : FOC中频任务处理
* Function Call  : 1ms中断
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_Task_MidFre(void)
{
	//--------------------------------------------------------------------------/	
	// 低速信号处理
	Sample_MidFre();
  
	//--------------------------------------------------------------------------/
	// 电机故障检测 	
  Fault_Check_1msTask();   

	//--------------------------------------------------------------------------/
	// 电机转速计算 
	MC_Speed_Cal();
	//--------------------------------------------------------------------------/
	// 电机调速模块
	OutLoop_Ctrl();
}

/*****************************************************************************
* Function Name  : FOC_Task_HighFre
* Description    : FOC高频任务处理
* Function Call  : EPWM中断调用（单电阻）/ADC中断调用（双电阻）
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Wj
******************************************************************************/
void FOC_Task_HighFre(void)
{
	//-------------------------------------------------------------------/
	//位置传感器处理
	#if(Config_Run_Mode==RUN_MD_LHALL)
	{
		Stru_Sample.LHallAlphaAD = Get_ADC_Result(ADC_DATA_LHALLAlpha);
		Stru_Sample.LHallBetaAD = Get_ADC_Result(ADC_DATA_LHALLBeta);
	}
  #endif
	
	//-------------------------------------------------------------------/
	//电压采样
	FOC_Read_Voltage();
	//-------------------------------------------------------------------/
	//电流采样
	FOC_Read_Current();          
	
	//-------------------------------------------------------------------/
	//Clark变换
	Clark();           	         
	
	//-------------------------------------------------------------------/
	//Park变换
  Park();	
	
	//-------------------------------------------------------------------/	
	// 计算转子位置
	FOC_RotorPosition_Detection();
	
	switch(MOTOR_STATE)
	{
		//-------------------------------------------------------------------/
		// 电机上电状态
		case MC_POWERON:
			
			FOC_TASK_POWERON();
		
			break;
		//-------------------------------------------------------------------/		
		// 线性霍尔学习状态
		case MC_LHALL_LEARN:
		
			FOC_LHall_LEARN();
		
			break;
		
		//-------------------------------------------------------------------/	
		// 线性霍尔等待收敛状态
		case MC_CONVERGENCE:
			
			FOC_TASK_CONVERGENCE();
		
			break;
		
		//-------------------------------------------------------------------/
		// 空闲状态
		case MC_IDLE:
			
			FOC_TASK_IDLE();
		
			break;
		//-------------------------------------------------------------------/		
		// 初始化状态
		case MC_INIT:
			
			FOC_TASK_INIT();
		
			break;

		//-------------------------------------------------------------------/	
		// 预充电状态
		case MC_CHARGE:
			
			FOC_TASK_CHARGE();
		
			break;
		//-------------------------------------------------------------------/
		// 测试状态
		case MC_TEST:
			
			FOC_TASK_TEST();
		
			break;

		//-------------------------------------------------------------------/
		// 顺逆风状态
		case MC_WIND:
			
			FOC_TASK_WIND();
		
			break;	
		//-------------------------------------------------------------------/
		// IPD状态
		case MC_IPD:
			
			FOC_TASK_IPD();
		
			break;	
		//-------------------------------------------------------------------/
		// 对齐状态
		case MC_ALIGN:
			
			FOC_TASK_ALIGN();
		
			break;
		//-------------------------------------------------------------------/
		// 启动状态
		case MC_STARTUP:
			
			FOC_TASK_START();
		
			break;
		//-------------------------------------------------------------------/
		// 切换状态
		case MC_SW:
			
			FOC_TASK_SW();
		
			break;
		//-------------------------------------------------------------------/
		// 运行状态
		case MC_RUN: 
			
			FOC_TASK_RUN();
		
			break;
		//-------------------------------------------------------------------/
		// 停机状态
		case MC_STOP:
			
			FOC_TASK_STOP();
		
			break;
		//-------------------------------------------------------------------/
		// 故障状态
		case MC_BRAKE:
			
			FOC_TASK_BRAKE();
		
			break;   
		//-------------------------------------------------------------------/
		default:
			break;
	}	
	
}



/******************************** END OF FILE *******************************/



