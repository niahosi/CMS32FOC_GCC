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

#ifndef __FAULT_H
#define __FAULT_H

/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>

#include "base_define.h"
#include "MC_FOC.h"
#include "algor_math.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/
#define	Fault_Flag								FaultMessag.Source.Flag
#define	Fault_HallError						FaultMessag.Source.Bits.HallError
#define	Fault_MosError						FaultMessag.Source.Bits.Offset
#define	Fault_StartFail						FaultMessag.Source.Bits.StartFail
#define	Fault_OFFSET							FaultMessag.Source.Bits.Offset
#define	Fault_UnderTemp						FaultMessag.Source.Bits.UnderTemp
#define	Fault_OverTemp						FaultMessag.Source.Bits.OverTemp

#define	Fault_PhaseLoss						FaultMessag.Source.Bits.LossPhase
#define	Fault_Block								FaultMessag.Source.Bits.Block
#define	Fault_UnderVoltage				FaultMessag.Source.Bits.UnderVolt
#define	Fault_OverVoltage					FaultMessag.Source.Bits.OverVolt
#define	Fault_OverPhaseCur				FaultMessag.Source.Bits.OverPhaseCurr
#define	Fault_ShortCircuit				FaultMessag.Source.Bits.ShortCircuit

/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/
/*----------------------------------------------------*/
typedef union
{
	uint16_t Flag;
	struct
	{
		uint16_t		ShortCircuit 				: 1;				//硬件过流错误标志位  
		uint16_t		OverPhaseCurr 			: 1;				//软件过流错误标志位  
		uint16_t		OverVolt   					: 1;				//过压错误标志位     
		uint16_t		UnderVolt  					: 1;				//欠压错误标志位       
		uint16_t		Block      					: 1;				//堵转错误标志位      
		uint16_t		LossPhase  					: 1;				//缺相错误标志位       
		uint16_t		OverTemp   					: 1;				//过温错误标志位    
		uint16_t		UnderTemp   				: 1;				//过温错误标志位  
		uint16_t		Offset							: 1;				//偏置错误标志位     
		uint16_t		StartFail      			: 1;				//启动错误标志位       
		uint16_t		MosError        		: 1;				//mos自检测错误标志位       
		uint16_t		HallError      			: 1;				//霍尔缺相标志位        
		uint16_t                 				: 4;				//保留，可根据其他需求来添加错误标志位，注意不要超过16位，否则需要修改u16 R --> u32 R
	} Bits;
} 
Struct_FaultStatus;

typedef enum
{
		NOERROR          	= 0,       //无错误 
		Short_Circuit 		= 1,       //硬件过流 
		Over_PhaseCurr		= 2,       //软件过流
		Over_Volt					= 3,       //母线欠压
		Under_Volt				= 4,       //母线过压 
		Motor_Block				= 5,       //堵转	
		Phase_Loss				= 6,       //缺相保护	
		Over_Temp					= 7,       //过温	
		Under_Temp				= 8,       //欠温	
		Offset_Error			= 9,       //偏置错误 
		Start_Fail				= 10,      //启动失败
		Mos_Error         = 11,      //MOS自检错误 
		Hall_Error        = 12,      //霍尔缺相保护   
} 
Struct_FaultCode;

typedef struct
{
  Struct_FaultStatus		Source;		// 故障源（所有故障）
  Struct_FaultCode			Code;			// 故障码（按优先级仅显示一个）
} 
Struct_Fault;



typedef struct
{
	//-------------------------------------------------------------------//
	uint16_t	Ia_ABS;
	uint16_t	Ib_ABS;
	uint16_t	Ic_ABS;
	
	uint16_t	Ia_Max;
	uint16_t	Ib_Max;
	uint16_t	Ic_Max;	
	
	//-------------------------------------------------------------------//
	// 母线硬件过流、短路保护
	struct
	{
		uint8_t			ReStartNum;				//设置的重启次数
		uint8_t			ReStartTimes;			// 已重启次数
    uint16_t		ReStartGAP;				//设置重启间隔时间    

	} ShortCircuit;
	
	//-------------------------------------------------------------------//
	// 相线软件过流
	struct
	{
		uint8_t			ReStartNum;				//设置的重启次数
		uint8_t			ReStartTimes;			// 已重启次数
    uint16_t		ReStartGap;				//设置重启间隔时间    
		
		uint16_t		OC_Value;					// 保护值
		uint16_t		Protect_Times;		// 设置的保护次数

	} OverPhaseCur;
	
	//-------------------------------------------------------------------//
	// 母线电压保护
	struct
	{
		uint16_t		Over_Volt_Value;		// 过压保护值
		uint16_t		Under_Volt_Value;		// 欠压保护值
		uint16_t		Protect_Time;				// 设置的保护等待时间
		
		uint16_t		OV_Recover_Value;		// 过压恢复值
		uint16_t		UV_Recover_Value;		// 欠压恢复值		
		uint16_t		Recover_Time;				// 设置的恢复间隔时间

	} Vbus;

	//-------------------------------------------------------------------//
	// 温度保护
	struct
	{

		int16_t		Over_Temp_Value;			// 过温保护值
		int16_t		Under_Temp_Value;			// 欠温保护值
		uint16_t	Protect_Time;					// 设置的保护等待时间
		
		int16_t		OT_Recover_Value;			// 过温恢复值
		int16_t		UT_Recover_Value;			// 欠温恢复值		
		uint16_t	Recover_Time;					// 设置的恢复间隔时间

	} Temp_Mos;	
	//-------------------------------------------------------------------//
	// 缺相保护
	struct
	{
		uint8_t			ReStartNum;					//设置的重启次数
		uint8_t			ReStartTimes;				// 已重启次数
    uint16_t		ReStartGAP;					//设置重启间隔时间    
		
		uint16_t		Curr_Min;						// 缺相保护值
		uint16_t		Protect_Time;				// 设置的保护等待时间

	} PhaseLoss;	
	//-------------------------------------------------------------------//
	// 堵转保护
	struct
	{
		uint8_t			ReStartNum;					//设置的重启次数
		uint8_t			ReStartTimes;				// 已重启次数
    uint16_t		ReStartGAP;     		//设置重启间隔时间    
		
		int32_t			BlockSpeed_Max;			// 堵转保护最大值
		int32_t			BlockSpeed_Min;			// 堵转保护最小值
		uint16_t		Protect_Time;				// 设置的保护等待时间

	} Block;	
	//-------------------------------------------------------------------//
	// 启动失败保护
	struct
	{
		uint8_t			ReStartNum;					//设置的重启次数
		uint8_t			ReStartTimes;				// 已重启次数
    uint16_t		ReStartGAP;     		//设置重启间隔时间    
		
		uint16_t		Protect_Time;				// 设置的保护等待时间
	} Start;		

} 
Struct_FaultCheck;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/
extern	Struct_Fault            FaultMessag;						
extern	Struct_FaultCheck				Stru_Fault;            

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
void Fault_Clear_RestartTimes(void);
void Fault_ParaInit(void);
void Fault_Vbus_Check(void);
void Fault_PhaseLoss_Check(void);	

void Fault_Check_FOCTask(void);
void Fault_ReStart_Process(void);
void Fault_Check_1msTask(void);
void Fault_Show_Source(void);
void Fault_OFFSET_Check(void);
/*****************************************************************************/
/* Intrinsic function definition ('extern', definition in C source) */
/*****************************************************************************/

/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/

/*****************************************************************************
*-----------------------------------------------------------------------------
* Function Name  :void Find_MaxCurStruct_Cur_abc *Iabc)
* Description    :查找最大电流
* Function Call  :
* Input Paragram :
* Return Value   :无
*-----------------------------------------------------------------------------
******************************************************************************/
static __inline void Find_MaxCur(Struct_Cur_abc *Iabc)
{

}

/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/


#endif

/******************************** END OF FILE *******************************/

