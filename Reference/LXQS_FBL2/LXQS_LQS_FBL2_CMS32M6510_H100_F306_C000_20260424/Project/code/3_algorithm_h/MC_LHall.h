//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    FOC_Hall_Start.h
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/

//==========================================================================//

#ifndef __FOC_Hall_Start_H
#define __FOC_Hall_Start_H

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>
#include "MC_PID.h"
#include "MC_FOC.h"
#include "algor_math.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/

typedef enum
{
	SIG_OFFSET   = 0,
	SIG_ORDER    = 1,
	ANGLE_OFFSET = 2,
}Learn_State;
/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/


typedef struct
{

	Learn_State LearnState;						//学习状态机
	
	uint8_t     LearnSuccessFlag;     //学习成功标志位
	
	uint8_t     Flag_ErrorNeed_Compen;			 //标志位，是否需要补偿
	
	int8_t  		Flag_CompenAngle_Determined;     //0->补偿角度还没确定，   1->补偿360度；-1->补偿负360°
	
	int32_t 		thetaerror;               //HALL拟合值与真实值误差
	
	int32_t 		thetaerrorCompened;       //补偿后的角度差
	
	int32_t 		thetaerrorPre;	          // 前一时刻角度误差
	
	int32_t 		deltaError;               //当前时刻与前一时刻误差之差
	Struct_FirstOrderLPF 			LPFLHall_AlphaOffset;
	Struct_FirstOrderLPF 			LPFLHall_BetaOffset;
	Struct_FirstOrderLPF 			LPFLHall_OffsetTheta;
	
}Struct_LHallLearn;


typedef struct
{
	//霍尔学习结构体
	Struct_LHallLearn	Learn;
	
	//霍尔信号
	int32_t			Ualpha;             
	int32_t			Ubeta;
	
	int16_t   	Hall_Seq;         		// 双线性霍尔安装顺序
	int32_t     LHall_OffsetTheta;		// 线性霍尔补偿角度

	uint8_t 		Pair_Ratio;						//电机极对数与霍尔磁环极对数之比

	Struct_PLLQ30  QPLLQ30;
	
	
	int16_t    ThetaH;								//霍尔角度
	int16_t	   ThetaE;								//电角度
	
}Struct_LHall;


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/
extern Struct_LHall			      Stru_LHall;
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
void LHall_Theta_Cal(Struct_LHall *LHall,int32_t Ualpha, int32_t Ubeta);

void FOC_LHall_LEARN(void);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/

#endif


/******************************** END OF FILE *******************************/

