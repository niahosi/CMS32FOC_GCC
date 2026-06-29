//==========================================================================//
/*****************************************************************************
*-----------------------------------------------------------------------------
* @file    mcu_init.h
* @author  CMS_Motor_Control_Team
* @version 第三代电机平台
* @date    2024年6月
* @brief   
*---------------------------------------------------------------------------//
*****************************************************************************/

//==========================================================================//


#ifndef _MCU_INIT_H
#define _MCU_INIT_H

#ifdef __cplusplus
extern "C"
{
#endif
	
/*****************************************************************************/
/* Include files */
/*****************************************************************************/
#include <stdint.h>

#include "mcu_driver.h"
#include "mcu_port.h"

#include "Set_SPEED.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define') */
/*****************************************************************************/
#define	EPWM_ZIFn_Flag				  (0x1UL)
#define	EPWM_ClearZIFn_Flag()		{EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;EPWM->ICLR |= EPWM_ZIFn_Flag;EPWM->LOCK = 0x0;}
#define	EPWM_ClearAllInt_Flag() {EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;EPWM->ICLR |= 0xFFFFFFFF;EPWM->LOCK = 0x0;}

#if (ADC_SCAN_CHA > ADC_SCAN_CHB)
	#define		ADC_INTER_CH                       (ADC_SCAN_CHA)				
#else
	#define		ADC_INTER_CH                       (ADC_SCAN_CHB)	
#endif
#define	ADC_ClearIntFlag_CHA() 	{ ADC->LOCK = ADC_LOCK_WRITE_KEY; ADC->ICLR |= ADC_INTER_CH; ADC->LOCK = 0x00;}

#define	ADC_ClearAllInt_Flag() 	{ ADC->LOCK = ADC_LOCK_WRITE_KEY; ADC->ICLR |= 0xFFFFFFFF; ADC->LOCK = 0x00;}

#define CMD_READ_ANGLE 0x00F5
#define CMD_READ_RD 	 0x4900
/*****************************************************************************/
/* Global type definitions ('typedef') */
/*****************************************************************************/




/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source) */
/*****************************************************************************/


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source) */
/*****************************************************************************/
void System_Init(void);

void Enable_INT(void);
void __DI1(void);
void LowPower_Mode(void);
void Normal_Mode(void);
void f_DelayTime(uint32_t delay);
void DelayTime_ms(uint32_t delay);
void ADC1_IPA_SAMP(void);
void ADC1_ClearAllInt_Flag(void);
void Delay_us(int32_t m);
uint32_t SPI_KTH7801_Read(uint32_t cmd);
/*****************************************************************************/
/* Intrinsic function definition ('__inline', definition in Header File) */
/*****************************************************************************/


/*****************************************************************************/
/* Macro Definition function definition ( definition in Header File) */
/*****************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* __HARDINIT_H__ */

/******************************** END OF FILE *******************************/
