/*******************************************************************************
*                  COPYRIGHT (C) 2021 CMS Technologies Ltd.                    *
*                                                                              *
********************************************************************************
* FileName      : tim_demo.c                                                  *
* Author        :                                                              *
* Version       : 1.0                                                          *
* Date          : 2021.08.13                                                   *
* Description   :                                                              *
* Function List :                                                              *
********************************************************************************/
#include "demo_timer4.h"
#include "demo_uart.h"


/***********************************************************************************************************************
* Function Name: TM4_GetPulseWidth
* @brief This function measures the high level pulse width of PA00.
* @param
* @return none
***********************************************************************************************************************/
void TM4_GetPulseWidth_Init(void)
{
    TIM_InitTypeDef TIM_InitStructure = {};
    
//		GPIO_PinAFInConfig(TI01CFG,IO_INCFG_P27_TAU1);  //P27 used as TI01 input
//		GPIO_Init(PORT2,PIN7,INPUT);

    TIM_InitStructure.TIM = TIM4;
    TIM_InitStructure.TIM_Channel = TTM_Channel_1;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Trigger     = TIM_Trigger_BothEdge; //bothedge :start trigger and capture trigger
    TIM_InitStructure.TIM_Pulse_Edge  = TIM_Pulse_Both_Width_High;  // Measure the high or low level pulse width of PA00/TI00
    TIM_InitStructure.TIM_Mode        = TIM_Mode_Measure;      // pulse width measure
    TIM_InitStructure.TIM_StartInt    = TIM_StartInt_Disable;
    TIM_InitStructure.TM4_Input      = TM4_CH1_Input_fIL;  // specify the internal low speed clock

    TIM_Init(&TIM_InitStructure);
		
		CGC_LSI_Enable();
}

/***********************************************************************************************************************
* Function Name: TM4_Get_PulsePeriod
* @brief  This function measures the pulse period with polling mode for TM4 module.
* @return None
***********************************************************************************************************************/
void TM4_GetPulsePeriod_Init(void)
{
    TIM_InitTypeDef TIM_InitStructure = {};
    
		GPIO_PinAFInConfig(TI01CFG,IO_INCFG_P27_TAU1);  //P27 used as TI01 input
		GPIO_Init(PORT2,PIN7,INPUT);

    TIM_InitStructure.TIM = TIM4;
    TIM_InitStructure.TIM_Channel     = TTM_Channel_1;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Trigger     = TIM_Trigger_IputEdge; //IputEdge is used to start trigger and capture trigger
    TIM_InitStructure.TIM_Pulse_Edge  = TIM_Pulse_Rising;  // Measure the high or low level pulse width of PA01/TI00  /TIM_Pulse_Both/TIM_Pulse_Rising

    TIM_InitStructure.TIM_Mode        = TIM_Mode_PluseInterval;  // pulse interval measure: use capture function
    TIM_InitStructure.TIM_StartInt    = TIM_StartInt_Disable;
    TIM_InitStructure.TM4_Input      = TM4_CH1_Input_TI01;  // specify the input of TI01
    TIM_Init(&TIM_InitStructure);
			
//		CGC_LSI_Enable();
}

/***********************************************************************************************************************
* Function Name: Tim4_IntervalTimer
* @brief This function interval timer and generate interval interrupt.
* @note   Interval timer period calculation： 1/(fclk/32768) *TIM_Period = timerValue (s)
*         for fclk = 32MHz ,timer frequence division is 32768, TTM_Channel_0 as interval timer， 
*         TIM_Period is 20；   timerValue = 1/1000* 20
* @return none
***********************************************************************************************************************/
void Tim4_IntervalTimer(void)
{
    TIM_InitTypeDef TIM_InitStructure = {};

    TIM_InitStructure.TIM         = TIM4;                   //specify the 8 channels TIM
    TIM_InitStructure.TIM_Channel = TTM_Channel_2;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div32768;  // specify the operation clk=1khz of tim for 32Mhz fclk
    TIM_InitStructure.TIM_Period[2]   = 20;                //specify the  time =20*(1/1000) =20ms
    TIM_InitStructure.TIM_Trigger     = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Mode        = TIM_Mode_Interval;  // external event
    TIM_InitStructure.TIM_StartInt    = TIM_StartInt_Disable;
    TIM_Init(&TIM_InitStructure);

		NVIC_EnableIRQ(TM02_IRQn);		//TIM4 通道2 中断服务注册
		NVIC_SetPriority(TM02_IRQn,3);	
}



