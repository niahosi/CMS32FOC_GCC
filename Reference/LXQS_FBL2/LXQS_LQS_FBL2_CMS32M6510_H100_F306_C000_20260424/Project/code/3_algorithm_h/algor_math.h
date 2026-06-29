//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    main.h
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/

//==========================================================================//



#ifndef __MCLIB_MATH_H
#define __MCLIB_MATH_H
/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>
#include "MC_FOC.h"
#include "MC_PID.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */ 
/*****************************************************************************/

//Q格式定义
#define 	_Q10(A) 												(int32_t)((A)*1024)												// Q10 format
#define 	_Q12(A) 												(int32_t)((A)*4096)												// Q12 format
#define 	_Q15(A) 												(int32_t)((A)*32767)											// Q15 format
#define 	_Q16(A) 												(int32_t)((A)*65536)											// Q15 format
#define 	_Q30(A) 												(int32_t)((A)*1073741824)											// Q15 format
#define 	_Q31(A) 												(int32_t)((A)*2147483647)											// Q15 format


#define 	_ANGLE_DEG2PU(A) 								(int32_t)(65536 * (A)/360)								// 角度的实际值转化为标幺值

 //一阶低通滤波滤波系数计算  Ts：计算周期 单位S  Fc：截止频率 单位HZ
#define   Cal_LPF_Coff(Ts,Fc)							(int32_t)(32768*(Ts*_2PI()*Fc/(1+Ts*_2PI()*Fc)))	 

//Foc中断
#define   _Foc_LPF_1Hz									 Cal_LPF_Coff(1.0/EPWM_FREQ,1.0)			           // 可根据模板自行添加 注：整数也要加小数点
#define   _Foc_LPF_3Hz                   Cal_LPF_Coff(1.0/EPWM_FREQ,3.0)     
#define   _Foc_LPF_10Hz                  Cal_LPF_Coff(1.0/EPWM_FREQ,10.0)      
#define   _Foc_LPF_11Hz                  Cal_LPF_Coff(1.0/EPWM_FREQ,11.0)   
#define   _Foc_LPF_40Hz                  Cal_LPF_Coff(1.0/EPWM_FREQ,40.0)  
#define   _Foc_LPF_400Hz                  Cal_LPF_Coff(1.0/EPWM_FREQ,400.0)  
//1ms中断
#define   _1MS_LPF_5Hz									 Cal_LPF_Coff(0.001,5.0)                        //截止频率为5，周期为1000
#define   _1MS_LPF_10Hz									 Cal_LPF_Coff(0.001,10.0)




//10ms中断
#define   _10MS_LPF_1Hz                  Cal_LPF_Coff(0.01,1.0)     


//角频率标幺值的
#define   _Fc_FOR_OMEGAPU_							(_Foc_LPF_400Hz)



#define 	FIXP_sat(A, V_MAX, V_MIN) ((A) > (V_MAX) ? (V_MAX) : (A) < (V_MIN) ? (V_MIN) : (A))

typedef struct   
{
	int16_t coff;
	int32_t y;
}Struct_FirstOrderLPF;





//正交锁相环 Q30格式
typedef struct   
{
	int32_t Omega;
	int32_t OmegaPU;
	
	int32_t Theta;
	
	Struct_Sincos  SinCos;  
	
	Struct_PIQ30  PI;        		  
	
	int32_t Coff_OmegaPLLQ30_T0_OmegaPUQ15;
}Struct_PLLQ30;



extern Struct_FirstOrderLPF 			LPFVbus;
extern Struct_FirstOrderLPF 			LPFIbus;
extern Struct_FirstOrderLPF 			LPFPower;
extern Struct_FirstOrderLPF 			LPFVctrl;
extern Struct_FirstOrderLPF 			LPFVbus_Aver;
extern Struct_FirstOrderLPF 			LPFSpeed;
extern Struct_FirstOrderLPF 			LPFOmegaPU;
/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/

//extern const uint16_t Q15Sqrt_Table[512];

extern const uint16_t Q15Sqrt_Table[677];
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
int32_t MCM_Sqrt(int32_t input);                            //软件开方
int32_t ArcTan_Cal(int32_t hSin,   int32_t hCos);           //反正切计算
int32_t MC_SoftPower_Calc(void);														// 软件功率计算
/*****************************************************************************
* Function Name  : LPF_Cal
* Description    : 低通滤波器  内部信号的Q格式为Q30
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2025/1/05  
******************************************************************************/
int32_t LPF_Cal(Struct_FirstOrderLPF *LPF,int32_t x, int16_t LPFCoff);
/*****************************************************************************
* Function Name  : LPF_Cal2
* Description    : 低通滤波器  内部信号的Q格式为Q25
* Function Call  : 
* Input Paragram : none
* Return Value   : none
* note           : 
* Version        : V0.1    2025/1/05  
******************************************************************************/
int32_t LPF_Cal2(Struct_FirstOrderLPF *LPF,int32_t x, int16_t LPFCoff);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/
//--------------------------------------------------------------------------/
// 绝对值
#define 	ABSFUN(Value)												((Value)>=0?(Value):(-(Value)))

// 最小值宏定义
#define 	MinFun(var1,var2)										((var1) < (var2)? (var1):(var2))
// 最大值宏定义
#define 	MaxFun(var1,var2)										((var1) > (var2)? (var1):(var2))

// D轴电流环KP计算
#define   ID_KP_CAL(F)								(int32_t)(_SQRT_3 * _Q12_VAL * _2PI()*F * MOTOR_LD * I_PHASE_BASE * 0.001 / V_DC_BASE) 
#define   IQ_KP_CAL(F)								(int32_t)(_SQRT_3 * _Q12_VAL * _2PI()*F * MOTOR_LQ * I_PHASE_BASE * 0.001 / V_DC_BASE) 


#define   ID_KI_CAL(F)								(int32_t)(_SQRT_3 * _Q15_VAL * _2PI()*F * MOTOR_RS * I_PHASE_BASE /V_DC_BASE / EPWM_FREQ);   
#define   IQ_KI_CAL(F)								(int32_t)(_SQRT_3 * _Q15_VAL * _2PI()*F * MOTOR_RS * I_PHASE_BASE /V_DC_BASE / EPWM_FREQ);  





#endif

/******************************** END OF FILE *******************************/

