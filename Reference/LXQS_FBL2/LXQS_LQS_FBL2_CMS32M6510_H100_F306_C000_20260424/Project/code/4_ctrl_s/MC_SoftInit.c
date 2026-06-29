
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    MC_SoftInit.c
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

//---------------------------------------------------------------------------/
//	Local variable  definitions
//---------------------------------------------------------------------------/
Struct_BaseValue				Stru_BaseValue				= {0};					// 基值结构体
Struct_Motor						Stru_Motor						= {0};					// 电机参数的标幺值的Q15格式
Struct_Para							Stru_Para							= {0};					// 参数设定的结构体
ByteFlag								B_Flag								= {0};					// 标志位
struct_Time							Stru_Time							= {0};					// 时间结构体

//从FLASH_PARA_ADDR处起，依次存放线性霍尔自学习的Q15结果
int32_t	LHall_Para[LHall_Para_Length]	__attribute__((at(FLASH_PARA_ADDR)))	=	
{	
	LEARN_ING,
	HALL_SEQ_FORWARD,
	2048,
	2048,
	0,
	(LEARN_ING + HALL_SEQ_FORWARD + 2048 + 2048 + 0),
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
* Function Name  : BaseValue_Init
* Description    : 基值初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建			Wj
******************************************************************************/
void BaseValue_Init(void)
{
		//直流侧基值
    Stru_BaseValue.Vdc				= V_DC_BASE;																								//电压基值
		Stru_BaseValue.Ibus 			= I_DC_BASE;

		//电机侧基值
    Stru_BaseValue.Vphase			= V_PHASE_BASE;																							//相电压基值 
		Stru_BaseValue.Iphase			= I_PHASE_BASE;																							//电流基值
    Stru_BaseValue.MechSpd		= MECH_SPEED_BASE;																					//机械转速基值
	
    Stru_BaseValue.ElecSpd		= (float)(Stru_BaseValue.MechSpd * MOTOR_PAIRS);						//电转速基值
    Stru_BaseValue.Rs					= (float)(Stru_BaseValue.Vphase / Stru_BaseValue.Iphase);   //相电阻基值
    Stru_BaseValue.MechOmega	= (float)((Stru_BaseValue.MechSpd * _2PI()) / 60);					//机械角速度基值
    Stru_BaseValue.ElecOmega	= (float)(Stru_BaseValue.MechOmega * MOTOR_PAIRS);					//电角速度基值
    Stru_BaseValue.Psi				= (float)(Stru_BaseValue.Vphase / Stru_BaseValue.ElecOmega);
																																													//磁链基值
    Stru_BaseValue.Ls					= (float)(Stru_BaseValue.Rs / Stru_BaseValue.ElecOmega);		//电感基值
    Stru_BaseValue.Te					= (float)(1.5 * MOTOR_PAIRS * Stru_BaseValue.Iphase * Stru_BaseValue.Psi);			
																																													//转矩基值
    Stru_BaseValue.Ts					= (float)(1.0 / Stru_BaseValue.ElecOmega);									//开关周期基值
    Stru_BaseValue.J					= (float)(Stru_BaseValue.Te / Stru_BaseValue.MechOmega);		//转动惯量基值
	
}

/*****************************************************************************
* Function Name  : Motor_Para_Init
* Description    : 电机标幺参数初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建		Wj
******************************************************************************/
void Motor_Para_Init(void)        
{
  Stru_Motor.pole		= (uint8_t)MOTOR_PAIRS;																				// 极对数
  Stru_Motor.RsPU		= MOTOR_RS  * _Q15_VAL / Stru_BaseValue.Rs;										// Rs值(标幺值的Q15格式)
  Stru_Motor.LdPU		= MOTOR_LD  * _Q15_VAL * 0.001 / Stru_BaseValue.Ls;						// Ld感量(标幺值的Q15格式)
  Stru_Motor.LqPU		= MOTOR_LQ  * _Q15_VAL * 0.001 / Stru_BaseValue.Ls;						// Lq感量(标幺值的Q15格式)
  Stru_Motor.PsiPU	= MOTOR_PSI * _Q15_VAL / Stru_BaseValue.Psi;									// 永磁磁链值(标幺值的Q15格式)
  Stru_Motor.Jpu		= 0.001 * _Q15_VAL / Stru_BaseValue.J;												// 转动惯量(标幺值的Q15格式)
}

/*****************************************************************************
* Function Name  : Convert_Para_Init
* Description    : 转换系数参数初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建			Wj、Lsy
******************************************************************************/
void Convert_Para_Init(void)  
{
//--------------------------------AD到标幺转换系数 Q10-------------------------------------------/	
	Stru_Coff.AD2PU_Ibus		= AD2Pu_Coeff_Ibus		* _Q10_VAL;					// Q10
	Stru_Coff.AD2PU_Vbus		= AD2Pu_Coeff_Vbus		* _Q10_VAL;					// Q10
	Stru_Coff.AD2PU_Ip			= AD2Pu_Coeff_Iphase	* _Q10_VAL;					// Q10
	Stru_Coff.AD2PU_Vctrl		= AD2Pu_Coeff_Vctrl		* _Q10_VAL;					// Q10
	
	Stru_Coff.AD2ModuSig_BEMF = AD2ModuSig_Coeff_BEMF		* _Q10_VAL;		// Q10	
//--------------------------------物理值到标幺转换系数 Q10---------------------------------------/

	Stru_Coff.Phy2PU_Ibus		= Phy2Pu_Coeff_Ibus		* _Q10_VAL;
	Stru_Coff.Phy2PU_Vbus		= Phy2Pu_Coeff_Vbus		* _Q10_VAL;
	Stru_Coff.Phy2PU_Power	= Phy2Pu_Coeff_Power	* _Q10_VAL;
	Stru_Coff.Phy2PU_Iphase = Phy2Pu_Coeff_Iphase * _Q10_VAL;
	Stru_Coff.Phy2PU_Speed	= Phy2Pu_Coeff_Speed	* _Q10_VAL;
	Stru_Coff.Phy2PU_Vctrl	= Phy2Pu_Coeff_Vctrl	* _Q10_VAL; 
	
//--------------------------------标幺值到物理值转换系数 Q15---------------------------------------/
	Stru_Coff.PU2Phy_Vbus		= Pu2Phy_Coeff_Vbus;											
	Stru_Coff.PU2Phy_Ibus		= Pu2Phy_Coeff_Ibus;
	Stru_Coff.PU2Phy_Power	= Pu2Phy_Coeff_Power;
	Stru_Coff.PU2Phy_Iphase = Pu2Phy_Coeff_Iphase;
	Stru_Coff.PU2Phy_Vctrl	= Pu2Phy_Coeff_Vctrl;
	
	//电角速度的锁相环值到机械角速度的转化系数
	Stru_Coff.OmegaePLL2MecSpeed = Coeff_OmegaPLL2MecSpd;            		
	
//--------------------------------工作电压与观测器电压转换系数-------------------------------------------/
	Stru_Coff.ModuSig2PU_Vphase_temp = V_DC_BASE * _Q10_VAL / _SQRT_3 / V_PHASE_BASE;   		 //Q10
	Stru_Coff.AD2ModuSig_Bef_temp = HW_ADC_REF*_SQRT_3*4*32768.0/HW_BEMF_DIVIDER/V_DC_BASE;  //Q1
	
}

/*****************************************************************************
* Function Name  : MC_IPD_Para_Init
* Description    : IPD参数初始化
* Function Call  : 上电初始化一次
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/09/18    新建			RMY
******************************************************************************/
void MC_IPD_Para_Init(void)
{
  Stru_IPD_Pulse.SqTime1							= IPD_SQUARE_TIME1;
  Stru_IPD_Pulse.SqTime2							= IPD_SQUARE_TIME2;
	Stru_IPD_Pulse.IPD_Cur_AdMin				= 4096;
}
/*****************************************************************************
* Function Name  : SetPara_init
* Description    : 用户设定参数初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建 		Wj
******************************************************************************/
void SetPara_init()
{
	//============================================================================/
	//-------------------------------运行模式----------------------------------/ 
  Stru_Para.Mode.Dir		= CONFIG_MOTOR_DIR;						//电机转动方向  0->顺时针  1->逆时针
  Stru_Para.Mode.Shunt	= Config_Shunt_Mode;					//单双电阻      Single_Shunt->单电阻  Double_Shunt->双电阻    
  Stru_Para.Mode.SVPWM	= CONFIG_SVPWM_MODE;					//SVPWM模式选择 0->5段式   1->7段式	
  
	//============================================================================/
	//-------------------------------启动参数----------------------------------/ 
	//对齐电流
	Stru_Para.Start.AlignCurMin					= Phy2Pu_Fun_Iphase(ALIGN_CUR_MIN);				//对齐电流初始值
  Stru_Para.Start.AlignCurMax					= Phy2Pu_Fun_Iphase(ALIGN_CUR_MAX);				//对齐电流最终值
  Stru_Para.Start.AlignCurInc					= Phy2Pu_Fun_Iphase(ALIGN_CUR_INC);				//对齐电流增量 
  Stru_Para.Start.AlignCurIncInterval = (ALIGN_CUR_INC_INTERVAL);								//对齐电流增加间隔	
  Stru_Para.Start.AlignTime						= (int32_t)(ALIGN_TIME*EPWM_FREQ);				//对齐时间	
  if(Stru_Para.Start.AlignCurInc<1)
  {
    Stru_Para.Start.AlignCurInc = 1;
  }

  //--------------------------------------------------------------------------/
	//启动电流
  Stru_Para.Start.StartCurMin					= Phy2Pu_Fun_Iphase(START_CUR_MIN);				//启动电流初始值
  Stru_Para.Start.StartCurMax					= Phy2Pu_Fun_Iphase(START_CUR_MAX);				//启动电流最终值
  Stru_Para.Start.StartCurInc					= Phy2Pu_Fun_Iphase(START_CUR_INC);				//启动电流增量 
  Stru_Para.Start.StartCurIncInterval = (int32_t)(START_CUR_INC_INTERVAL);			//启动电流增加间隔	
  if(Stru_Para.Start.StartCurInc<1)
  {
    Stru_Para.Start.StartCurInc = 1;
  }

	//--------------------------------------------------------------------------/
	//启动锁相环PI参数
  Stru_Para.Start.PLLKp			= START_PLLKP;            
  Stru_Para.Start.PLLKi			= START_PLLKI;    

	//--------------------------------------------------------------------------/
	//启动自适应率
  Stru_Para.Start.Adp_Max		= (int32_t)START_ADP_MAX;     
  Stru_Para.Start.Adp_Min		= (int32_t)START_ADP_MIN;    
	
	//--------------------------------------------------------------------------/
	//闭环转速真实值
  Stru_Para.Start.Speed_Close = START_SPEED_CLOSE;      
	//闭环保持时间
  Stru_Para.Start.HoldTime		= START_HOLD_TIME * EPWM_FREQ;          
	//SW状态机保持时间
  Stru_Para.Start.SwitchTime = SWITCH_HOLD_TIME * EPWM_FREQ;

	//============================================================================/
	//-------------------------------运行参数----------------------------------/ 

	// DQ轴电流环PI系数
	#if (CURRENT_MODE == CURRENT_BY_FORMULA)
  Stru_Para.Run.Dkp				= ID_KP_CAL(RUN_IDQ_FC);     
  Stru_Para.Run.Dki				= ID_KI_CAL(RUN_IDQ_FC);
  if(Stru_Para.Run.Dkp > 28000)			Stru_Para.Run.Dkp = 28000;
  if(Stru_Para.Run.Dki > 28000)			Stru_Para.Run.Dki = 28000;
	
	Stru_Para.Run.Qkp				=  IQ_KP_CAL(RUN_IDQ_FC);               
  Stru_Para.Run.Qki				=  IQ_KI_CAL(RUN_IDQ_FC);
	if(Stru_Para.Run.Qkp > 28000)		Stru_Para.Run.Qkp = 28000;
  if(Stru_Para.Run.Qki > 28000)		Stru_Para.Run.Qki = 28000;   
	
	#else
  Stru_Para.Run.Dkp				= RUN_DKP;       
  Stru_Para.Run.Dki				= RUN_DKI;
	Stru_Para.Run.Qkp				= RUN_QKP;       
  Stru_Para.Run.Qki				= RUN_QKI;
	#endif	
	
	// DQ轴输出限幅
  Stru_Para.Run.Dout_Max	= RUN_DOUT_MAX;        
  Stru_Para.Run.Dout_Min	= RUN_DOUT_MIN;      
  Stru_Para.Run.Qout_Max	= RUN_QOUT_MAX;        
	Stru_Para.Run.Qout_Min	= RUN_QOUT_MIN;    

	
	//--------------------------------------------------------------------------/
	//运行锁相环PI参数
  Stru_Para.Run.PLLKp			= RUN_PLLKP;              //锁相环比例系数
  Stru_Para.Run.PLLKi			= RUN_PLLKI;              //锁相环积分系数
	//--------------------------------------------------------------------------/
	//功率环PI参数
  Stru_Para.Run.PwrKp			= POWER_KP;               //功率环比例系数
  Stru_Para.Run.PwrKi			= POWER_KI;               //功率环积分系数
	//速度环PID参数
  Stru_Para.Run.SpdKp			= SPEED_KP;               //速度环比例系数
  Stru_Para.Run.SpdKi			= SPEED_KI;               //速度环积分系数
  Stru_Para.Run.SpdKd			= 0;               //速度环微分系数 
	
	//弱磁参数
	Stru_Para.Run.WeakFluxKp = WEAKFLUX_KP;						//弱磁比例系数
	Stru_Para.Run.WeakFluxKi = WEAKFLUX_Ki;					  //弱磁积分系数
	Stru_Para.Run.WF_AngMax  = MAX_LEADANGLE;					  //弱磁积分系数
}



/*****************************************************************************
* Function Name  : Foc_Para_Init
* Description    : FOC参数初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建					Wj、Lsy
******************************************************************************/
void Foc_Para_Init()
{
	//------------------------------------------------------------------------/	
	//电流初始化
  Stru_Cur_abc.Ia						= 0;  
  Stru_Cur_abc.Ib						= 0;  
  Stru_Cur_abc.Ic						= 0;  
  Stru_Cur_alphabeta.Ialpha	= 0;
  Stru_Cur_alphabeta.Ibeta	= 0;
  Stru_Cur_dqRef.Id					= 0;
  Stru_Cur_dqRef.Iq					= 0;
  Stru_Cur_dq.Id						= 0;
  Stru_Cur_dq.Iq						= 0;
    
	//------------------------------------------------------------------------/	
	//电压初始化    
  Stru_Vol_abc.Ua						= 0;
  Stru_Vol_abc.Ub						= 0;
  Stru_Vol_abc.Uc						= 0;
  Stru_Vol_alphabeta.Ualpha	= 0;
  Stru_Vol_alphabeta.Ubeta	= 0;
  Stru_Vol_dq.Ud						= 0;
  Stru_Vol_dq.Uq						= 0;
  
	//------------------------------------------------------------------------/	
	//FOC参数、角度、极限圆初始化 
  Stru_Foc.EPWM_Period			= EPWM_PERIOD;
  Stru_Foc.OffsetTheta			= FOC_OFFSET_ANGLE*65536/360;
  Stru_Foc.Elec_Omega				= 0;
	Stru_Foc.OmegaPU					= 0;
  Stru_Foc.Elec_Theta				= 0;
	Stru_Foc.Curr_Is_Max			= Phy2Pu_Fun_Iphase(IPHASE_MAX);
	Stru_Foc.Curr_Is_Min			= Phy2Pu_Fun_Iphase(IPHASE_MIN);
  Stru_Sincos.Sin						= 0;
  Stru_Sincos.Cos						= 16384;
	
	#if ((Config_SpeedUp_Mode == Over_Modulate) || (Config_SpeedUp_Mode == OverAndWeaken))
	
  Stru_Foc.Vsmax						= MAX_MODULATE;
	Stru_Foc.VsmaxSquare			= MAX_MODULATE * MAX_MODULATE * 1.3 ;
  Stru_Foc.Q15_VsmaxSquare	= (int32_t)(MAX_MODULATE * MAX_MODULATE * 1.3)>>15;
  Stru_Foc.WeakThreshold		= WEAKENING_THRESHOLD * MAX_MODULATE * 1.15;
	Stru_Foc.WeakVs_Square			= Stru_Foc.WeakThreshold * Stru_Foc.WeakThreshold >> 15;
	
	#else
	
  Stru_Foc.Vsmax						= MAX_MODULATE;
  Stru_Foc.VsmaxSquare			= MAX_MODULATE*MAX_MODULATE ;
	Stru_Foc.Q15_VsmaxSquare	= MAX_MODULATE*MAX_MODULATE >> 15;
  Stru_Foc.WeakThreshold		= WEAKENING_THRESHOLD * MAX_MODULATE;
	Stru_Foc.WeakVs_Square			= Stru_Foc.WeakThreshold * Stru_Foc.WeakThreshold >> 15;
	
	#endif
	//------------------------------------------------------------------------/	
	//SVPWM初始化 
	
	Stru_SVPWM.EPWM_Period		= (uint16_t)EPWM_PERIOD;

  Stru_SVPWM.Ts_Max					= MAX_MODULATE; 	
	Stru_SVPWM.T_TGDLY				= (int32_t)((EPWM_DT+SAMP_STEADY) * EPWM_Tus * _Q15_VAL / EPWM_PERIOD);
	Stru_SVPWM.T_Ahead				= (int32_t)(EPWM_AHEAD *EPWM_Tus* _Q15_VAL / EPWM_PERIOD);
	Stru_SVPWM.TSamp_Window		= (int32_t)((EPWM_DT+SAMP_STEADY +EPWM_AHEAD) * EPWM_Tus * _Q15_VAL / EPWM_PERIOD);	

	Stru_SVPWM.SectorNum			= 1;
  Stru_SVPWM.Last_Sector		= 1;
	Stru_SVPWM.Cnt_TG1st			= Stru_SVPWM.EPWM_Period >> 1;
	Stru_SVPWM.Cnt_TG2nd			= Stru_SVPWM.EPWM_Period >> 2;

	Stru_SVPWM.LN_Ctrl				= 0;
	Stru_SVPWM.LN_State				= 0;
	Stru_SVPWM.LN_StateLast		= 0;
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

        
}

/*****************************************************************************
* Function Name  : LHall_Ctrl_Init
* Description    : 线性霍尔启动控制初始化
* Function Call  : 电机霍尔启动前调用
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1     2025/01/06    新建			Xh
******************************************************************************/
void LHall_Ctrl_Init(void)
{
//学习初始化
	Stru_LHall.Learn.LearnState = SIG_OFFSET;
	Stru_LHall.Learn.LearnSuccessFlag = 0;
	Stru_LHall.Learn.Flag_ErrorNeed_Compen = 0;
	Stru_LHall.Learn.Flag_CompenAngle_Determined = 0;
	Stru_LHall.Learn.thetaerror = 0;
	Stru_LHall.Learn.thetaerrorCompened = 0;
	Stru_LHall.Learn.thetaerrorPre = 0;
	Stru_LHall.Learn.deltaError = 0;

	Stru_LHall.Learn.LPFLHall_OffsetTheta.coff  = 0;
	Stru_LHall.Learn.LPFLHall_AlphaOffset.coff = 0;
	Stru_LHall.Learn.LPFLHall_BetaOffset.coff = 0;
	Stru_LHall.Learn.LPFLHall_OffsetTheta.y  = 0;
	Stru_LHall.Learn.LPFLHall_AlphaOffset.y = 0;
	Stru_LHall.Learn.LPFLHall_BetaOffset.y = 0;
	Stru_LHall.Ualpha = 0;
	Stru_LHall.Ubeta = 0;
	
	Stru_LHall.Pair_Ratio = 1;			//电机极对数与磁环极对数之比
	Stru_LHall.ThetaH = 0;					//霍尔角度
	Stru_LHall.ThetaE = 0;					//电角度
	
	
	// PLL初始化
	Stru_LHall.QPLLQ30.Coff_OmegaPLLQ30_T0_OmegaPUQ15 = (Stru_LHall.Pair_Ratio*EPWM_FREQ*30.0)/(MOTOR_PAIRS*MECH_SPEED_BASE);
	Stru_LHall.QPLLQ30.Omega = 0;
	Stru_LHall.QPLLQ30.OmegaPU = 0;
	Stru_LHall.QPLLQ30.Theta = 0;
	Stru_LHall.QPLLQ30.SinCos= SinCos_Cal((Stru_LHall.QPLLQ30.Theta>>15));
	PIQ30_Init(&Stru_LHall.QPLLQ30.PI,LHALL_PLLKP,LHALL_PLLKI,15,15,_Q30(0.9),_Q30(-0.9),24);
	
	
	
	Stru_LHall.Hall_Seq = LHall_Para[LHALL_ORDER_INDEX];
	Stru_LHall.LHall_OffsetTheta = LHall_Para[LHALL_OFFSET_ANGLE_INDEX];
	Stru_Sample.LHallAlphaOffset = LHall_Para[OFFSET_LHALLALPHA_INDEX];
	Stru_Sample.LHallBetaOffset = LHall_Para[OFFSET_LHALLBETA_INDEX];
	Stru_Foc.LHall_LearnState = LHall_Para[LEARN_STATE_INDEX];
	
	//如果和检验没通过，则说明学习错误或者还未进行学习，则需要重新学习
	if( (LHall_Para[CHECKSUM_INDEX]) != (LHall_Para[LEARN_STATE_INDEX] + LHall_Para[LHALL_ORDER_INDEX]
																			+LHall_Para[OFFSET_LHALLALPHA_INDEX] + LHall_Para[OFFSET_LHALLBETA_INDEX]
																			+LHall_Para[LHALL_OFFSET_ANGLE_INDEX]))
	{
		Stru_Foc.LHall_LearnState =  LEARN_ING;
	}

}

/*****************************************************************************
* Function Name  : AlgorPara_Init
* Description    : 观测器参数初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : 
* Return Value   : none
* note           : 
* Version        : V0.1    2024/06/16    新建			Wj
******************************************************************************/
void AlgorPara_Init()
{
	//时间基值  Q15
  Stru_AlgorPara.TsPu				= (32768.0/EPWM_FREQ)/Stru_BaseValue.Ts;        
	//自适应律
  Stru_AlgorPara.AdpMax			= Stru_Para.Start.Adp_Max;     
  Stru_AlgorPara.AdpMin			= Stru_Para.Start.Adp_Min;  
	//系数Q8,将角速度的锁相环值转化为标幺值的Q15格式 
  Stru_AlgorPara.OmegaPLL2SpdPUQ15	=  (EPWM_FREQ *_PI()*256.0)/Stru_BaseValue.ElecOmega;           
	//电角频率基值
  Stru_AlgorPara.ElecOmegaBase			= Stru_BaseValue.ElecOmega;   

	//观测器1参数
	Stru_AlgorPara.LibOb1.K1 = -25000;
	Stru_AlgorPara.LibOb1.K2 = 25000;
	Stru_AlgorPara.LibOb1.PLLKp = Stru_Para.Start.PLLKp;
	Stru_AlgorPara.LibOb1.PLLKi = Stru_Para.Start.PLLKi;
	Stru_AlgorPara.LibOb1.m = 0;
	Stru_AlgorPara.LibOb1.q = 15; 
	
	//观测器2参数
	Stru_AlgorPara.LibOb2.K1 = -25000;
	Stru_AlgorPara.LibOb2.K2 = 25000;
	Stru_AlgorPara.LibOb2.PLLKp = Stru_Para.Run.PLLKp;
	Stru_AlgorPara.LibOb2.PLLKi = Stru_Para.Run.PLLKi;
	Stru_AlgorPara.LibOb2.m = 0;
	Stru_AlgorPara.LibOb2.q = 15; 
	//观测器3参数
	Stru_AlgorPara.LibOb3.OmegaPu = 0;
	Stru_AlgorPara.LibOb3.PLLKp = Stru_Para.Run.PLLKp;
	Stru_AlgorPara.LibOb3.PLLKi = Stru_Para.Run.PLLKi;
	Stru_AlgorPara.LibOb3.m = 0;
	Stru_AlgorPara.LibOb3.q = 15; 
}

/*****************************************************************************
* Function Name  : MC_LPFCoff_Init
* Description    : 滤波系数计算
* Function Call  : 上电开启中断前调用一次
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			
******************************************************************************/
void MC_LPFCoff_Init()
{
	// 初值赋值
	LPFVbus.y = 0;
	LPFIbus.y = 0;
	LPFPower.y = 0;
	LPFVctrl.y = 0;
	LPFSpeed.y = 0;
	LPFOmegaPU.y = 0;
}

/*****************************************************************************
* Function Name  : TimePara_Init
* Description    : 系统时间初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建					Lsy
******************************************************************************/
void TimePara_Init()
{
	Stru_Time.PowerOn					= 50;					// 上电延时50ms，滤波变量等加载
	Stru_Time.Motor_PowerDown	=	TIME_STOP_HOLD * EPWM_FREQ / 1000;
	Stru_Time.Motor_Brake			= BRAKE_STOP_TIME * EPWM_FREQ / 1000;
}

/*****************************************************************************
* Function Name  : OpenLoop_ParaInit
* Description    : 开环参数初始化
* Function Call  : 上电开启中断前调用一次
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建					Lsy
******************************************************************************/
void OpenLoop_ParaInit(void)
{
	#if (Config_HWTEST_MODE == HWTEST_CURR_DRAG)
		Stru_Test.OpenCurr_Final = Phy2Pu_Fun_Iphase(Openloop_Curr_Max);
	#elif (Config_HWTEST_MODE == HWTEST_VOLT_DRAG)
		Stru_Test.OpenCurr_Final = Openloop_Volt_Max;
	#endif

	Stru_Test.DragCurr = Stru_Test.OpenCurr_Final >> 1;
	
	// 按OmegaE每次+1计算	
	Stru_Test.OmegaEMax			= Openloop_Speed_Max * MOTOR_PAIRS *_Q16_VAL / ( 60 * EPWM_FREQ);
	Stru_Test.DeataCur			= (Stru_Test.OpenCurr_Final >> 1) / Stru_Test.OmegaEMax + 1;
	
	Stru_Test.ACCCycle			= (int32_t)((float)Openloop_AccTime *  (uint32_t)EPWM_FREQ / Stru_Test.OmegaEMax);
	
	if(Stru_Test.ACCCycle < 1)	Stru_Test.ACCCycle = 1;
	
}

/*****************************************************************************
* Function Name  : VbusRippleComp_Para_Init
* Description    : 纹波电压补偿参数初始化
* Function Call  : 上电前调用一次
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建					Lsy
******************************************************************************/
void VbusRippleComp_Para_Init(void)
{

	LPFVbus_Aver.y = 0;
	Stru_VbusRippleComp.CompGain_Q14 	= _Q14_VAL;
	Stru_VbusRippleComp.Vbus_Aver 		= _Q14_VAL;
}

/*****************************************************************************
* Function Name  : WeakenFlux_Init
* Description    : 死区补偿参数初始化
* Function Call  : 上电初始化一次
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2024/07/03    新建			Lsy
******************************************************************************/
void DeadComp_ParaInit(void)
{
	Stru_DeadComp.s32_Comp_Coeff = _Q15_VAL * EPWM_DT * 2 /(float)(1000000.0/EPWM_FREQ);
	Stru_DeadComp.s32_Comp_Angle = 0;
}
/*****************************************************************************
* Function Name  : MC_Para_Init
* Description    : 所有电机参数上电初始化
* Function Call  : 上电硬件初始化后、开启中断前调用一次，
* Input Paragram : none
* Return Value   : none
* note           : 510us
* Version        : V0.1    2024/07/03    新建					Lsy
******************************************************************************/
void MC_Para_Init(void)
{
		// 1 基值参数初始化
		BaseValue_Init();        
		// 2 电机标幺参数初始化
		Motor_Para_Init();           
		// 3 转化系数初始化
		Convert_Para_Init();            
		// 4 设定宏定义参数初始化
		SetPara_init();            
		// 5 Foc参数初始化
		Foc_Para_Init();             
		// 6 算法参数初始化
		AlgorPara_Init();       
	
		// 7 PI参数初始化

		// 9 故障参数初始化
		Fault_ParaInit();        
		
		// 10 用户控制、命令初始化
		UserCtrl_Para_Init();
		
		// 11 滤波系数初始化
		MC_LPFCoff_Init();

		// 12 时间初始化
		TimePara_Init();
		
		//计算查表斜率
		Cal_Slope();
		
		// 测试模式初始化
		#if ((Config_HWTEST_MODE == HWTEST_VOLT_DRAG) || (Config_HWTEST_MODE == HWTEST_CURR_DRAG))
		{
			OpenLoop_ParaInit();
		}
		#endif
		

		// 死区补偿初始化
		#if (CONFIG_DeadComp_MODE == DeadComp_Enable)
		{
			DeadComp_ParaInit();			
		}
		#endif

		// 母线纹波电压补偿初始化
		#if (Config_VbusRipple_Comp == Vbus_Comp_ENABLE)
		{
			VbusRippleComp_Para_Init();			
		}
		#endif

		// IPD定位初始化
		#if (CONFIG_INITIAL_POSITION == POSITION_PULSE_INJECTION)
		{
			MC_IPD_Para_Init();				
		}
		#endif

		// 顺逆风初始化
		#if (Config_Wind_Mode == Start_Wind)
		{
			Wind_ParaInit();
		}
		#endif



		// Final 采样初始化
		HW_Sample_Init();
		
}






/******************************** END OF FILE *******************************/




