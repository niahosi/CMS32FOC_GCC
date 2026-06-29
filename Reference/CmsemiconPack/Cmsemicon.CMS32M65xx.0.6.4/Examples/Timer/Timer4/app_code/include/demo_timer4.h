#ifndef __TIMER4_DEMO_H__
#define __TIMER4_DEMO_H__
#include "gpio.h"
#include "timer4.h"

void TM4_GetPulseWidth_Init(void);
void TM4_GetPulsePeriod_Init(void);
uint32_t TM4_GetPulsePeriod(void);
void Tim4_IntervalTimer(void);
/***********************************************************************************************************************
* Function Name: RESTPinGpio_Set
* @brief  This function set the P02 to gpio or reset pin.
 ** \param [in] RStat: 
 **								Enable: P02 to gpio
 **								Disable: P02 to reset pin
* @return None
***********************************************************************************************************************/
void RESTPinGpio_Set(uint8_t RStat);

#endif
