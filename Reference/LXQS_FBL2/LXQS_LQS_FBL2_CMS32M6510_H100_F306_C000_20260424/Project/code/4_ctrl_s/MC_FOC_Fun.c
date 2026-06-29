
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    MC_FOC_Fun.c
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   该文件主要存放Foc控制的部分功能、以及用作封库buff
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
struct_RippleComp 				Stru_VbusRippleComp 				= {0};				// 母线纹波电压补偿

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
* Function Name  : VbusRipple_Comp_Cal
* Description    : 母线纹波电流补偿
* Function Call  : FOC任务调用
* Input Paragram : 母线电压 
* Return Value   : 补偿系数
* note           : 
* Version        : V0.1    2024/09/13    新建			Lsy
******************************************************************************/
void VbusRipple_Comp_Cal(int32_t VbusReal)
{
	Stru_VbusRippleComp.Vbus_Aver = LPF_Cal(&LPFVbus_Aver,VbusReal,_Foc_LPF_40Hz);

	Stru_VbusRippleComp.CompGain_Q14 = HWDivider(Stru_VbusRippleComp.Vbus_Aver << 14,VbusReal);
	
	// 限制max2 min0.5
	if(Stru_VbusRippleComp.CompGain_Q14 > 32768)	Stru_VbusRippleComp.CompGain_Q14 = 32768;
	else if(Stru_VbusRippleComp.CompGain_Q14 < 8192)	Stru_VbusRippleComp.CompGain_Q14 = 8192;
}


/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  : Coordinate_Switch
* Description    : 全坐标投切
* Function Call  : 
* Input Paragram :
* Return Value   : 
*-----------------------------------------------------------------------------
******************************************************************************/
void Coordinate_Switch(int16_t Theta_LS , int16_t Theta_HS)
{
	int32_t Angle_Error;
	int32_t tempD;
	int32_t tempQ;
	Struct_Sincos	  	Stru_SinCos = {0};
	//求角度误差
	Angle_Error = Theta_HS - Theta_LS;
	if ( Angle_Error > 50000 ) Angle_Error -= 65535;
	if ( Angle_Error < -50000 ) Angle_Error += 65535;
	Stru_SinCos = SinCos_Cal(Angle_Error);	
	//电压切换
	tempD   = ((  Stru_Vol_dq.Ud  * Stru_SinCos.Cos ) >> 15 )  + (( Stru_Vol_dq.Uq  * Stru_SinCos.Sin ) >> 15 );
	tempQ   = (( -Stru_Vol_dq.Ud  * Stru_SinCos.Sin ) >> 15 )  + (( Stru_Vol_dq.Uq  * Stru_SinCos.Cos ) >> 15 );
	Stru_PI_Id.IntegralTerm = (tempD<<Stru_PI_Id.qKi);
	Stru_PI_Iq.IntegralTerm = (tempQ<<Stru_PI_Iq.qKi);
	//电流切换
	tempD   = ((  Stru_Cur_dq.Id * Stru_SinCos.Cos ) >> 15 )  + (( Stru_Cur_dq.Iq * Stru_SinCos.Sin ) >> 15 );
	tempQ   = (( -Stru_Cur_dq.Id * Stru_SinCos.Sin ) >> 15 )  + (( Stru_Cur_dq.Iq * Stru_SinCos.Cos ) >> 15 );
	Stru_Cur_dqRef.Id = tempD;	
	Stru_Cur_dqRef.Iq = tempQ;	
}








/******************************** END OF FILE *******************************/


