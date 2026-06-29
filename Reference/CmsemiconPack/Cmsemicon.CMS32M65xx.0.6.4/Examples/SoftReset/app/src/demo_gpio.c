#include "demo_gpio.h"
/**
  * @brief  Demo code of operation about gpio.
  * @param  No parameter.
  * @return void
*/
void Led_Init(void)
{		
		GPIO_PinAFOutConfig(P20CFG,IO_OUTCFG_P20_GPIO);	
		GPIO_Init(PORT2,PIN0,OUTPUT);
}

