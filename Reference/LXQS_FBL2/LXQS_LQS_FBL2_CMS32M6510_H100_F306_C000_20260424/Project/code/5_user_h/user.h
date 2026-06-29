
//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    user.h
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/

//==========================================================================//
#ifndef __USER_H
#define __USER_H



/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/


/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/

typedef enum 
{
	SYS_INIT		= 0,
	SYS_RUN			= 1,
	SYS_FAULT		= 2
}SystStates_e;


typedef struct 
{ 	
	uint8_t	SigOnFlag;						// 信号标志  0无用户控制信号 1 有控制信号
	
  uint8_t RunFlag;      				// 启动标志  0关机  1运行
	
	int32_t CtrlSig;							// 控制信号值
	
	int32_t Target_Min;
	int32_t Target_Max;
	
	int32_t Signal_OFF;
	int32_t Signal_ON;
	int32_t Signal_Max;
	
	int32_t TargetKe;
	int32_t TargetB;
	
	int32_t TargetPu;							// 控制目标值
	int32_t TargetPhy;
	
	int32_t	SpeedFG;
	
}Struct_User;


typedef struct 
{ 
	uint8_t		OverFlowFlag;	
	uint8_t		CompleteFlag;
	uint8_t		IO_HighCnt;
	uint8_t		IO_LowCnt;
	
	int32_t		Fre;
	int16_t		Duty;
	
	int32_t		FreTemp;
	int16_t		DutyTemp;	
	
	uint32_t		HighTime;
	uint32_t		LowTime;	
	uint32_t		PeriodTime;
	
	uint32_t		OverFlowTime;

	
}Struct_Capture;


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/

extern SystStates_e             SYSTEM_STATE;
extern Struct_User              Stru_User;              //用户命令

extern Struct_Capture						Stru_Capture;
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
void User_Motor_On(void);
void User_Motor_Off(void);
void System_Control(void);
void Task1ms_Main(void);
void UserCtrl_Para_Init(void);
void User_Ctrl_Motor(void);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/


#endif




/******************************** END OF FILE *******************************/




