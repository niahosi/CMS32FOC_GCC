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

#ifndef __ALGOR_SVPWM_H
#define __ALGOR_SVPWM_H


/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>
#include "MC_FOC.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/


/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/

typedef struct
{
	uint8_t			LN_State;								// LN状态			1当前周期LN控制  0 当前周期非LN控制
	uint8_t			LN_StateLast;						// 上个周期LN状态
	uint8_t			LN_Ctrl;								// LN控制使能 	0，不使能 1使能
	
  uint8_t			SectorNum;							// 扇区	
	uint8_t 		Last_Sector;						// 上个周期扇区

	uint16_t		EPWM_Period;						// FOC周期计数器值

	int32_t			TSamp_Window;						// 单电阻采样窗口 Q15	
	int32_t			T_Ahead;								// 采样提前时间
	int32_t			T_TGDLY;								// 单电阻采样延迟	
	int32_t			Ts_Max;									// T1 T2最长时间 Q15		
	
	int32_t			T1;
	int32_t			T2;
	
	int32_t			Tu;											// 双电阻下三相占空比时间 Q15
	int32_t			Tv;
	int32_t			Tw;
	
	int32_t			Tu_up;									// 单电阻左右半周占空比时间时间Q15
	int32_t			Tu_dn;
	int32_t			Tv_up;
	int32_t			Tv_dn;
	int32_t			Tw_up;
	int32_t			Tw_dn;
	
	int32_t			CntU_up;								// 单电阻左右半周触发点寄存器值
	int32_t			CntU_dn;
	int32_t			CntV_up;
	int32_t			CntV_dn;
	int32_t			CntW_up;
	int32_t			CntW_dn;
	
	int32_t			T_TG1st;
	int32_t			T_TG2nd;		
	int32_t			Cnt_TG1st;							// 第一触发点寄存器值
	int32_t			Cnt_TG2nd;							// 第二触发点寄存器值
	
}struct_SVPWM;


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/
extern struct_SVPWM											Stru_SVPWM;


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/

void SVPWM_Para_Init(void);
void SVPWM_Ctrl_Init(void);

void SVPWM_DoubleShunt(struct_SVPWM *SVPWM,int32_t Ualpha,int32_t Ubeta);

void SVPWM_SingleShunt(struct_SVPWM *SVPWM,int32_t Ualpha,int32_t Ubeta);
void SVPWM_LN_SingleShunt(struct_SVPWM *SVPWM,int32_t Ualpha,int32_t Ubeta);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/


#endif


/******************************** END OF FILE *******************************/






