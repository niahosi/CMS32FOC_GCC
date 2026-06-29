//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    mcu_port.h
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/

//==========================================================================//


#ifndef _MCU_PORT_H
#define _MCU_PORT_H
/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>
#include "Set_SPEED.h"
#include "mcu_driver.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/
#define ON                      (1) 
#define OFF                     (0)

 //掩码控制，上三管关闭，下三管打开
#define Bridge_Output_Down() 	{EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY; EPWM->MASK = 0x00003F2A;EPWM->LOCK = 0x0;}  
//掩码控制，关闭6个管子
#define Bridge_Output_Off()  	{EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY; EPWM->MASK = 0x00003F00;EPWM->LOCK = 0x0;} 

 //关闭掩码功能，使能PWM控制引脚输出
#define Bridge_Output_On()   	{EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY; EPWM->MASK = 0x00000000;EPWM->LOCK = 0x0;} 

//读取ADC结果
#define Get_ADC_Result(Channel) (ADC->DATA[Channel])


/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/




/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
void UART_View(int16_t view1,int16_t view2,int16_t view3,int16_t view4);

uint32_t Soft_Trig_ADC(int32_t channelNum);         //软件触发AD采样
int32_t HWDivider(int32_t x,int32_t y);             //硬件除法器
int32_t HWSqrt(int32_t x);                          //硬件开方器
void SetEPWMRegister(void);                         //更新EPWM寄存器
void SetEPWMDuty(int32_t dutyA,int32_t dutyB,int32_t dutyC);  //设置三相占空比

void EPWM_ResetFaultBrake(void);
void User_Speed_Out(int32_t SpeedFG);
void User_CapMode2_Handle(void);
void User_PWM_Capture(void);
int16_t ADC_IPD_SoftSAMP(void);
void MC_PWM_Mask(uint16_t ucDriver);
void ADC_IPD_REV(void);
void ADC_IPD_CONFIG(void);
void EPWM_CompareTriger_Reset(void);
void DutyBrake(int32_t Duty);
void Flash_Write_Int32(uint32_t* addr , int32_t value);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/


/*****************************************************************************
Description : 双电阻模式下，EPWM比较寄存器更新
Input       : TrigCnt0：比较触发器0的触发点，TrigCnt1：比较触发器1的触发点
*****************************************************************************/
#define SetTrigerPoint(TrigCnt0, TrigCnt1)	\
{																						\
    EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;		\
    EPWM->CMPTGD[0] = TrigCnt0;							\
    EPWM->CMPTGD[1] = TrigCnt1;							\
    EPWM->LOCK = 0x0;												\
}



#endif

/******************************** END OF FILE *******************************/

