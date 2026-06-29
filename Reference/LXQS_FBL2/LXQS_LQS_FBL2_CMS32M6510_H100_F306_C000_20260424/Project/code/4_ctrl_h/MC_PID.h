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

#ifndef __PID_H
#define __PID_H

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
typedef struct 
{ 	
    uint8_t qKp;                          //系数Q格式
    uint8_t qKi;                          //系数Q格式
    
    int32_t Kp;                           //比例系数分子             
    int32_t Ki;                           //积分系数分子   
    
    int32_t IntegralTerm;                 //积分项 
    
    int32_t UpperIntegralLimit;           //积分项上限                                     
    int32_t LowerIntegralLimit;           //积分项下限

    int32_t UpperOutputLimit;             //输出值上限 
    int32_t LowerOutputLimit;             //输出值下限
      
    int32_t Error;                        //误差值
		int32_t Output;												//PI输出值
}Struct_PI;

typedef struct 
{ 	
	uint8_t SigFixP;                      //PI控制器内部信号的Q格式
	
	uint8_t qKp;                          //系数Q格式
	uint8_t qKi;                          //系数Q格式
	
	int32_t Kp;                           //比例系数分子             
	int32_t Ki;                           //积分系数分子   
	
	int32_t IntegralTerm;                 //积分项 
	int32_t ProportionTerm;								//比例项

	int32_t UpperIntegralLimit;           //积分项上限                                     
	int32_t LowerIntegralLimit;           //积分项下限

	int32_t UpperOutputLimit;             //输出值上限 
	int32_t LowerOutputLimit;             //输出值下限
		
	int32_t Error;                        //误差值
	int32_t Output;												//PI输出值

}Struct_PIQ30;



/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/
extern Struct_PI                 Stru_PI_Id;                   //d轴电流环
extern Struct_PI                 Stru_PI_Iq;                   //q轴电流环
extern Struct_PI							 	 Stru_PI_OL;
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
int32_t PIQ30_Controller(Struct_PIQ30 *hPI, int32_t error);
void PIQ30_Init(Struct_PIQ30 *hPI,int32_t kp,int32_t ki,uint8_t Qkp,uint8_t Qki,int32_t max,int32_t min,uint8_t FixP);


int32_t PI_Controller(Struct_PI *pi, int32_t error);
void PI_Para_Init(Struct_PI *PI, int32_t kp, int32_t ki, uint8_t Qkp, uint8_t Qki, int16_t max, int16_t min);
void PI_Set_Integrater(Struct_PI *PI, int32_t temp);
void PI_Set_Limit(Struct_PI *PI, int32_t Upper, int32_t Lower);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/

#endif 

/******************************** END OF FILE *******************************/

