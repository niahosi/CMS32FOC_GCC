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
#include "tim_demo.h"
#include "gpio_demo.h"
#include "uart_demo.h"
extern  unsigned char ledon = 0;
uint32_t g_Int_Taken;

/***********************************************************************************************************************
* Function Name: Tim4_SquareOutput
* @brief  TIM4 output 50% square wave
* @param
* @return none
***********************************************************************************************************************/
void Tim40_SquareOutput()
{
    TIM_InitTypeDef TIM_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_PinAFConfig(GPIO_PORT0, GPIO_Pin_1, GPIO_P01, GROUP_AF_ODEFAULT); //P01 used as TO00 output(P01 default function)

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT0, &GPIO_InitStruct);


    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Channel = TTM_Channel_0;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[0] = 100;                 //specify the number of count clock
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Mode = TIM_Mode_Square;      // square count mode
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_Init(&TIM_InitStructure);
}
/***********************************************************************************************************************
* Function Name: Tim40_DelayCounter
* @brief
* @param
* @return none
***********************************************************************************************************************/
void Tim40_DelayCounter()
{
    TIM_InitTypeDef TIM_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_6, GPIO_P16, GROUP_AF_ODEFAULT); //P16 default used as TO01 output

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);


    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Channel = TTM_Channel_1;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[0] = 100;                 //specify the number of count clock
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_IputEdge; //TImn valid input edge is used as a start or capture trigger
    TIM_InitStructure.TIM4_Input = TIM4_CH1_Input_ELC; // choose ELC event as Timer input
    TIM_InitStructure.TIM_Pulse_Edge = TIM_Pulse_Rising; // Timer valid edge choose
    TIM_InitStructure.TIM_Mode = TIM_Mode_DelayCount;      // DelayCount mode
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_Init(&TIM_InitStructure);
}

/***********************************************************************************************************************
* Function Name: Tim40_PwmOutput1_1
* @brief  TIM4 output square wave which can select different duty
* @param  period - the period of PWM waveform
* @param  duty - the duty factor of PWM waveform
* @return none
***********************************************************************************************************************/
void Tim40_PwmOutput1_1(uint16_t period, uint16_t duty)
{

    TIM_InitTypeDef TIM_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_6, GPIO_P16, GROUP_AF_ODEFAULT); //P16 default used as TO01 output
    //  GPIO_PinAFConfig(GPIO_PORT1,GPIO_Pin_7,GPIO_P17,GROUP_AF_ODEFAULT);  //P17 default used as TO02 output
    //  GPIO_PinAFConfig(GPIO_PORT3,GPIO_Pin_1,GPIO_P31,GROUP_AF_ODEFAULT);  //P31 default used as TO03 output

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Selection_Master = TTM_Channel_0;//when multi-tim combination,it can generate pwm wave
    TIM_InitStructure.TIM_Channel = TTM_Channel_1;  //|TTM_Channel_2|TTM_Channel_3
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[0] = period;                 //specify the number of count clock
    TIM_InitStructure.TIM_Period[1] = duty;                 //specify duty
    //  TIM_InitStructure.TIM_Period[2] = 80;                   //specify duty1
    //  TIM_InitStructure.TIM_Period[3] = 80;                   //specify duty2
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Mode = TIM_Mode_PWM_Master;      // PWM_Master mode
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Enable;   //the relationship between startCount and interrupt setting
    TIM_Init(&TIM_InitStructure);
}
/***********************************************************************************************************************
* Function Name: Tim4_PwmOutput
* @brief  TIM4 output square wave which can select different duty
* @param  period - the period of PWM waveform
* @param  duty - the duty factor of PWM waveform
* @return none
***********************************************************************************************************************/
void Tim40_PwmOutput1_2(uint16_t period, uint16_t duty, uint16_t duty1)
{

    TIM_InitTypeDef TIM_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_6, GPIO_P16, GROUP_AF_ODEFAULT); //P16 default used as TO01 output
    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_7, GPIO_P17, GROUP_AF_ODEFAULT); //P17 default used as TO02 output
    //  GPIO_PinAFConfig(GPIO_PORT3,GPIO_Pin_1,GPIO_P31,GROUP_AF_ODEFAULT);  //P31 default used as TO03 output

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT3, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Selection_Master = TTM_Channel_0;//when multi-tim combination,it can generate pwm wave
    TIM_InitStructure.TIM_Channel = TTM_Channel_1 | TTM_Channel_2; //|TTM_Channel_2|TTM_Channel_3
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[0] = period;                 //specify the number of count clock
    TIM_InitStructure.TIM_Period[1] = duty;                 //specify duty
    TIM_InitStructure.TIM_Period[2] = duty1;                   //specify duty1
    //  TIM_InitStructure.TIM_Period[3] = 80;                   //specify duty2
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Mode = TIM_Mode_PWM_Master;      // PWM_Master mode
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Enable;   //the relationship between startCount and interrupt setting
    TIM_Init(&TIM_InitStructure);
}

void Tim40_PwmOutput1_3(uint16_t period, uint16_t duty, uint16_t duty1, uint16_t duty2)
{

    TIM_InitTypeDef TIM_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_6, GPIO_P16, GROUP_AF_ODEFAULT); //P16 default used as TO01 output
    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_7, GPIO_P17, GROUP_AF_ODEFAULT); //P17 default used as TO02 output
    GPIO_PinAFConfig(GPIO_PORT3, GPIO_Pin_1, GPIO_P31, GROUP_AF_ODEFAULT); //P31 default used as TO03 output

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT3, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Selection_Master = TTM_Channel_0;//when multi-tim combination,it can generate pwm wave
    TIM_InitStructure.TIM_Channel = TTM_Channel_1 | TTM_Channel_2; //|TTM_Channel_2|TTM_Channel_3
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[0] = period;                 //specify the number of count clock
    TIM_InitStructure.TIM_Period[1] = duty;                 //specify duty
    TIM_InitStructure.TIM_Period[2] = duty1;                   //specify duty1
    TIM_InitStructure.TIM_Period[3] = duty2;                   //specify duty2
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Mode = TIM_Mode_PWM_Master;      // PWM_Master mode
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Enable;   //the relationship between startCount and interrupt setting
    TIM_Init(&TIM_InitStructure);
}

void Tim40_PwmOutput2_2(uint16_t period, uint16_t duty, uint16_t period1, uint16_t duty1)
{

    TIM_InitTypeDef TIM_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_6, GPIO_P16, GROUP_AF_ODEFAULT); //P16 default used as TO01 output
    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_7, GPIO_P17, GROUP_AF_ODEFAULT); //P17 default used as TO02 output
    GPIO_PinAFConfig(GPIO_PORT3, GPIO_Pin_1, GPIO_P31, GROUP_AF_ODEFAULT); //P31 default used as TO03 output

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT3, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Selection_Master = TTM_Channel_0 | TTM_Channel_2; //when multi-tim combination,it can generate pwm wave
    TIM_InitStructure.TIM_Channel = TTM_Channel_1 | TTM_Channel_3; //|TTM_Channel_2|TTM_Channel_3
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[0] = period;                 //specify the number of count clock
    TIM_InitStructure.TIM_Period[1] = duty;                 //specify duty
    TIM_InitStructure.TIM_Period[2] = period1;                   //specify duty1
    TIM_InitStructure.TIM_Period[3] = duty1;                   //specify duty2
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Mode = TIM_Mode_PWM_Master;      // PWM_Master mode
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Enable;   //the relationship between startCount and interrupt setting
    TIM_Init(&TIM_InitStructure);
}
/***********************************************************************************************************************
* Function Name: Tim4_EventCount
* @brief  TIM4 specified channel operates as external event counter.
* @param
* @return none
***********************************************************************************************************************/
void Tim40_EventCount()
{
    TIM_InitTypeDef TIM_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    ISR_InitTypeDef_t ISR_InitStructure;

    GPIO_PinAFConfig(GPIO_PORT0, GPIO_Pin_0, GPIO_P00, GROUP_AF_ODEFAULT); //P00 used as TI00 input

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT0, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Channel = TTM_Channel_0;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[0] = 10;                 //specify the count num of event
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Pulse_Edge = TIM_Pulse_Rising;  // TIM effective edge choose
    TIM_InitStructure.TIM_Mode = TIM_Mode_EventCount;      // event count
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_InitStructure.TIM4_Input  = TIM4_CH0_Input_TI00;  // specify the input of TI
    TIM_Init(&TIM_InitStructure);

    ISR_InitStructure.IRQ_Channel = TM00_IRQn;
    ISR_InitStructure.IRQ[ISR_InitStructure.IRQ_Channel].intHandle = tim40_channel0_interrupt;//中断服务路径注册
    ISR_Init(&ISR_InitStructure);

    INTC_EnableIRQ(TM00_IRQn);
}

/***********************************************************************************************************************
* Function Name: Tim4_GetPulseWidth
* @brief This function measures the high level pulse width of P00.
* @param
* @return none
***********************************************************************************************************************/
uint32_t Tim40_GetPulseWidth()
{
    uint32_t width;

    TIM_InitTypeDef TIM_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;
    ISR_InitTypeDef_t ISR_InitStructure;

    GPIO_PinAFConfig(GPIO_PORT0, GPIO_Pin_0, GPIO_P00, GROUP_AF_ODEFAULT); //P00 used as TI00 input

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT0, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Channel = TTM_Channel_0;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_BothEdge; //bothedge :start trigger and capture trigger
    TIM_InitStructure.TIM_Pulse_Edge = TIM_Pulse_Both_Width_High;  // Measure the high or low level pulse width of P00/TI00
    TIM_InitStructure.TIM_Mode = TIM_Mode_Measure;      // pulse width measure
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;

    TIM_Init(&TIM_InitStructure);

    ISR_InitStructure.IRQ_Channel = TM00_IRQn;
    ISR_InitStructure.IRQ[ISR_InitStructure.IRQ_Channel].intHandle = tim40_channel0_interrupt;//中断服务路径注册
    ISR_Init(&ISR_InitStructure);

    INTC_EnableIRQ(TM00_IRQn);

    while (g_Int_Taken < 1);

    width = pData.len;

	TIM_Stop(TIM40,TTM_Channel_0);

    return (width);
}

/***********************************************************************************************************************
* Function Name: TM40_Get_PulsePeriod
* @brief  This function measures the pulse period with polling mode for TM4 module.
* @return period - return the pulse period at specified edge
***********************************************************************************************************************/
uint32_t Tim40_GetPulsePeriod()
{
    uint32_t width;

    TIM_InitTypeDef TIM_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;
    ISR_InitTypeDef_t ISR_InitStructure;

    GPIO_PinAFConfig(GPIO_PORT0, GPIO_Pin_0, GPIO_P00, GROUP_AF_ODEFAULT); //P00 used as TI00 input

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT0, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM40;
    TIM_InitStructure.TIM_Channel = TTM_Channel_0;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div1;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_IputEdge; //IputEdge is used to start trigger and capture trigger
    TIM_InitStructure.TIM_Pulse_Edge = TIM_Pulse_Rising;  // Measure the high or low level pulse width of P00/TI00
    TIM_InitStructure.TIM_Mode = TIM_Mode_PluseInterval;  // pulse interval measure: use capture function
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_InitStructure.TIM4_Input  = TIM4_CH0_Input_TI00;  // specify the input of TI
    TIM_Init(&TIM_InitStructure);

    ISR_InitStructure.IRQ_Channel = TM00_IRQn;
    ISR_InitStructure.IRQ[ISR_InitStructure.IRQ_Channel].intHandle = tim40_channel0_interrupt;//中断服务路径注册
    ISR_Init(&ISR_InitStructure);

    INTC_EnableIRQ(TM00_IRQn);

    while (g_Int_Taken < 2);

    width = pData.len;

	TIM_Stop(TIM40,TTM_Channel_0);
    return (width);
}


/***********************************************************************************************************************
* Function Name: tim40_channel0_interrupt
* @brief  TIM4 Channel interrupt service routine
* @param  msg
* @return None
***********************************************************************************************************************/
void tim40_channel0_interrupt(void *msg)
{
    ATE_FRAME_t *pFrame = (ATE_FRAME_t *)msg;

    INTC_ClearPendingIRQ(TM00_IRQn);    // clear INTTM00 interrupt flag
    g_Int_Taken++;
    pFrame->flag = INT_RX;              //interrupt is generated

    if (1U == (TM40->TSR00 & _0001_TM4_OVERFLOW_OCCURS))
    {
        pFrame->len = (uint32_t)(TM40->TDR00 + 1UL) + 0x10000UL;
    }
    else
    {
        pFrame->len = (uint32_t)(TM40->TDR00 + 1UL);
    }
}

/***********************************************************************************************************************
* Function Name: Tim41_IntervalTimer
* @brief This function interval timer and generate interval interrupt.
* @param
* @return none
***********************************************************************************************************************/
void Tim41_IntervalTimer()
{
    TIM_InitTypeDef TIM_InitStructure;
    ISR_InitTypeDef_t ISR_InitStructure;


    TIM_InitStructure.TIM = TIM41;                   //specify the 8 channels TIM
    TIM_InitStructure.TIM_Channel = TTM_Channel_0;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div32768;// specify the operation clk=1khz of tim for 32Mhz fclk
    TIM_InitStructure.TIM_Period[0] = 20;                //specify the  time =20*(1/1000) =20ms
    TIM_InitStructure.TIM_Mode = TIM_Mode_Interval; // external event
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_Init(&TIM_InitStructure);

    ISR_InitStructure.IRQ_Channel = TM10_IRQn;
    ISR_InitStructure.IRQ[ISR_InitStructure.IRQ_Channel].intHandle = tim41_channel0_interrupt;//中断服务路径注册
    ISR_Init(&ISR_InitStructure);

    INTC_EnableIRQ(TM10_IRQn);
}

/***********************************************************************************************************************
* Function Name: tim41_channel0_interrupt
* @brief  TIM41 Channel interrupt service routine
* @param  msg
* @return None
***********************************************************************************************************************/

void tim41_channel0_interrupt(void *msg)
{
    INTC_ClearPendingIRQ(TM10_IRQn);    // clear INTTM00 interrupt flag

    UART2_RX_STA |= 1 << 15;                //强制标记接收完成

    TIM_Cmd(TIM41, TTM_Channel_0, Disable);
    //  led_onoff(ledon);
    //  ledon = !ledon;
}

/***********************************************************************************************************************
* Function Name: Tim41_SquareOutput
* @brief  TIM41 output 50% square wave
* @param
* @return none
***********************************************************************************************************************/
void Tim41_SquareOutput()
{
    TIM_InitTypeDef TIM_InitStructure;
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_5, GPIO_P15, GROUP_AF_TO11); //P15 used as TO11 output

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Level  = GPIO_Level_LOW;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);


    TIM_InitStructure.TIM = TIM41;
    TIM_InitStructure.TIM_Channel = TTM_Channel_1;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div32768; //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[1] = 2000;                  //specify the count
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Mode = TIM_Mode_Square;         // interval count mode
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_Init(&TIM_InitStructure);
}

/***********************************************************************************************************************
* Function Name: Tim41_EventCount
* @brief  TIM41 specified channel operates as external event counter.
* @param
* @return none
***********************************************************************************************************************/
void Tim41_EventCount()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_InitTypeDef TIM_InitStructure;
    ISR_InitTypeDef_t ISR_InitStructure;

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_1, GPIO_P11, GROUP_AF_TI11); // TI11 can be used to any disired pins

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM41;
    TIM_InitStructure.TIM_Channel = TTM_Channel_1;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div32768;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Period[1] = 30;                //specify the count num of event
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_Software; //specify the software trigger
    TIM_InitStructure.TIM_Pulse_Edge = TIM_Pulse_Rising;  //specify the input edge of Tim
    TIM_InitStructure.TIM_Mode = TIM_Mode_EventCount;   // external event
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_InitStructure.TIM4_Input  = TIM4_CH1_Input_TI01;  // specify the input of TI
    TIM_Init(&TIM_InitStructure);

    ISR_InitStructure.IRQ_Channel = TM11_IRQn;
    ISR_InitStructure.IRQ[ISR_InitStructure.IRQ_Channel].intHandle = tim41_channel1_interrupt;//中断服务路径注册
    ISR_Init(&ISR_InitStructure);

    INTC_EnableIRQ(TM11_IRQn);
}

/***********************************************************************************************************************
* Function Name: tim8_channel1_interrupt
* @brief  TIM8 Channel interrupt service routine
* @param  msg
* @return None
***********************************************************************************************************************/

void tim41_channel1_interrupt(void *msg)
{
    INTC_ClearPendingIRQ(TM11_IRQn);    // clear INTTM00 interrupt flag
    g_Int_Taken++;
    //  led_onoff(ledon);
    //  ledon = !ledon;
}

/***********************************************************************************************************************
* Function Name: Tim8_EventCount
* @brief  TIM8 specified channel operates as external event counter.
* @param
* @return none
***********************************************************************************************************************/
uint32_t Tim41_Get_PulseWidth(TIM_Pulse_t PulseEdge)
{
    uint32_t width;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_InitTypeDef TIM_InitStructure;
    ISR_InitTypeDef_t ISR_InitStructure;

    GPIO_PinAFConfig(GPIO_PORT1, GPIO_Pin_5, GPIO_P15, GROUP_AF_TI12); //P15 used as TI12 input

    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Ctrl   = GPIO_Control_DIG;
    GPIO_Init(GPIO_PORT1, &GPIO_InitStruct);

    TIM_InitStructure.TIM = TIM41;
    TIM_InitStructure.TIM_Channel = TTM_Channel_2;
    TIM_InitStructure.TIM_ClkDivision = TIM_CLK0_Div32768;   //specify the operation clk of tim
    TIM_InitStructure.TIM_Trigger = TIM_Trigger_BothEdge;  // TIM effective edge choose
    TIM_InitStructure.TIM_Pulse_Edge = PulseEdge;  //specify the input edge of Tim
    TIM_InitStructure.TIM_Mode = TIM_Mode_Measure;  // measure event
    TIM_InitStructure.TIM_StartInt = TIM_StartInt_Disable;
    TIM_Init(&TIM_InitStructure);

    ISR_InitStructure.IRQ_Channel = TM12_IRQn;
    ISR_InitStructure.IRQ[ISR_InitStructure.IRQ_Channel].intHandle = tim41_channel2_interrupt;//中断服务路径注册
    ISR_Init(&ISR_InitStructure);

    INTC_EnableIRQ(TM12_IRQn);

    pData.flag = INT_IDLE;

    while (pData.flag < INT_RX);

    width = pData.len;

	TIM_Stop(TIM41,TTM_Channel_2);

    return (width);
}
/***********************************************************************************************************************
* Function Name: tim8_channel2_interrupt
* @brief  TIM8 Channel interrupt service routine
* @param  msg
* @return None
***********************************************************************************************************************/
void tim41_channel2_interrupt(void *msg)
{
    ATE_FRAME_t *pFrame = (ATE_FRAME_t *)msg;

    INTC_ClearPendingIRQ(TM12_IRQn);    // clear INTTM00 interrupt flag
    g_Int_Taken++;
    pFrame->flag = INT_RX;              //interrupt is generated

    if (1U == (TM41->TSR12 & 0x0001))
    {
        pFrame->len = (uint32_t)(TM41->TDR12 + 1UL) + 0x10000UL;
    }
    else
    {
        pFrame->len = (uint32_t)(TM41->TDR12 + 1UL);
    }
}
