#ifndef __GPIO_DEMO_H__
#define __GPIO_DEMO_H__
#include "gpio.h"
#define LED_ON         PORT_ClrBit(PORT2,PIN0)
#define LED_OFF        PORT_SetBit(PORT2,PIN0)
#define LED_Toggle     PORT->P2 ^= (1<<0);
void Led_Init(void);

#endif
