#include "Board_PWM.h"
#include "Board_Analog.h"
#include "cgc.h"
#include "common.h"
#include "epwm.h"
#include "gpio.h"

/**
 * @file Board_PWM.c
 * @brief 板级三相 EPWM 输出实现。
 * @details 配置互补 PWM、死区、ADC 触发点和 P16 驱动使能安全态。
 */

#define PWM_MAIN_CH_MASK (EPWM_CH_0_MSK | EPWM_CH_2_MSK | EPWM_CH_4_MSK)
#define PWM_ALL_CH_MASK                                                                            \
    (EPWM_CH_0_MSK | EPWM_CH_1_MSK | EPWM_CH_2_MSK | EPWM_CH_3_MSK | EPWM_CH_4_MSK | EPWM_CH_5_MSK)

static uint16_t s_duty_u;
static uint16_t s_duty_v;
static uint16_t s_duty_w;
static uint16_t s_adc_trigger_tick = PWM_ADC_TRIGGER_TICK_DEFAULT;
static uint16_t s_adc_trigger_tick_a = PWM_ADC_TRIGGER_TICK_DEFAULT;
static uint16_t s_adc_trigger_tick_b = PWM_ADC_TRIGGER_TICK_DEFAULT;

static uint8_t s_output_on;
static uint8_t s_brake_on;

static void SetDriverEnable(uint8_t enable);
static uint16_t ClampDuty(uint16_t duty);
static void ConfigPwmPins(void);
static void ConfigAdcTrigger(void);
static void UpdateAdcTriggerTicks(void);
static void ResetPwmState(void);

void Board_InitPwm(void)
{
    /*
     * 三相主通道使用 EPWM0/2/4，互补输出由硬件波形发生器生成。
     * 初始化阶段先保持 50% 占空比和软件刹车，最后再启动计数器。
     */
    CGC_PER11PeriphClockCmd(CGC_PER11Periph_EPWM, ENABLE);
    EPWM_ConfigRunMode(EPWM_COUNT_UP_DOWN | EPWM_OCU_SYMMETRIC | EPWM_WFG_COMPLEMENTARYK |
                       EPWM_OC_INDEPENDENT);

    EPWM_ConfigChannelClk(EPWM0, EPWM_CLK_DIV_1);
    EPWM_ConfigChannelClk(EPWM2, EPWM_CLK_DIV_1);
    EPWM_ConfigChannelClk(EPWM4, EPWM_CLK_DIV_1);

    EPWM_ConfigChannelPeriod(EPWM0, PWM_PERIOD);
    EPWM_ConfigChannelPeriod(EPWM2, PWM_PERIOD);
    EPWM_ConfigChannelPeriod(EPWM4, PWM_PERIOD);

    EPWM_ConfigChannelSymDuty(EPWM0, PWM_DUTY_50);
    EPWM_ConfigChannelSymDuty(EPWM2, PWM_DUTY_50);
    EPWM_ConfigChannelSymDuty(EPWM4, PWM_DUTY_50);

    EPWM_DisableReverseOutput(EPWM_CH_0_MSK | EPWM_CH_1_MSK | EPWM_CH_2_MSK | EPWM_CH_3_MSK |
                              EPWM_CH_4_MSK | EPWM_CH_5_MSK);
    EPWM_EnableDeadZone(PWM_ALL_CH_MASK, PWM_DEADTIME_TICKS);
    EPWM_EnableAutoLoadMode(PWM_MAIN_CH_MASK);
    UpdateAdcTriggerTicks();
    ConfigAdcTrigger();

    EPWM_EnableSoftwareBrake();
    ConfigPwmPins();
    EPWM_DisableOutput(PWM_ALL_CH_MASK);
    ResetPwmState();

    EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
    EPWM->POREMAP = 0xAA543210;
    EPWM->LOCK = 0x0;

    EPWM_ConfigLoadAndIntMode(EPWM0, EPWM_EACH_ZERO);
    EPWM_ConfigLoadAndIntMode(EPWM2, EPWM_EACH_ZERO);
    EPWM_ConfigLoadAndIntMode(EPWM4, EPWM_EACH_ZERO);
    EPWM_Start(PWM_MAIN_CH_MASK);
    Board_ForcePwmOff();
}

void Board_SetPwmDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w)
{
    /* 只写主通道比较值；互补通道由 EPWM 根据死区配置自动生成。 */
    s_duty_u = ClampDuty(duty_u);
    s_duty_v = ClampDuty(duty_v);
    s_duty_w = ClampDuty(duty_w);

    EPWM_ConfigChannelSymDuty(EPWM0, s_duty_u);
    EPWM_ConfigChannelSymDuty(EPWM2, s_duty_v);
    EPWM_ConfigChannelSymDuty(EPWM4, s_duty_w);
    Board_UpdateCurrentSampleTiming();
}

void Board_SetAdcTriggerTick(uint16_t tick)
{
    if (tick > PWM_PERIOD)
    {
        tick = PWM_PERIOD;
    }

    s_adc_trigger_tick = tick;
    UpdateAdcTriggerTicks();
    ConfigAdcTrigger();
}

uint16_t Board_GetAdcTriggerTick(void)
{
    return s_adc_trigger_tick;
}

uint16_t Board_GetAdcTriggerTickA(void)
{
    return s_adc_trigger_tick_a;
}

uint16_t Board_GetAdcTriggerTickB(void)
{
    return s_adc_trigger_tick_b;
}

void Board_SetAdcTriggerTicks(uint16_t tick_a, uint16_t tick_b)
{
    if (tick_a > PWM_PERIOD)
    {
        tick_a = PWM_PERIOD;
    }
    if (tick_b > PWM_PERIOD)
    {
        tick_b = PWM_PERIOD;
    }

    s_adc_trigger_tick_a = tick_a;
    s_adc_trigger_tick_b = tick_b;
    ConfigAdcTrigger();
}

void Board_ForcePwmOff(void)
{
    /*
     * 关断顺序优先关闭驱动使能，再打开软件刹车并关闭 EPWM 输出。
     * 占空比回到 50%，避免下一次使能时残留上一次控制量。
     */
    SetDriverEnable(0);
    EPWM_EnableSoftwareBrake();
    EPWM_DisableOutput(PWM_ALL_CH_MASK);

    Board_SetPwmDuty(PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50);

    s_output_on = 0;
    s_brake_on = 1;
}

uint8_t Board_EnablePwmOutput(uint8_t enable)
{
    if (enable != 0)
    {
        /*
         * 重新输出前必须清除 mask/brake 状态，再打开 EPWM 输出和驱动使能。
         * 调用者必须已经完成零漂、采样同步和状态机安全检查。
         */
        EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
        EPWM->MASK = 0x00000000;
        EPWM->LOCK = 0x0;

        EPWM_ClearBrakeIntFlag();
        EPWM_ClearBrake();
        EPWM_EnableOutput(PWM_ALL_CH_MASK);
        EPWM_DisableSoftwareBrake();
        SetDriverEnable(1);

        s_output_on = 1;
        s_brake_on = 0;
        return 1;
    }

    Board_ForcePwmOff();
    return 0;
}

void Board_GetPwmDebug(volatile uint16_t* duty_u, volatile uint16_t* duty_v,
                       volatile uint16_t* duty_w, volatile uint8_t* output_on,
                       volatile uint8_t* brake_on)
{
    *duty_u = s_duty_u;
    *duty_v = s_duty_v;
    *duty_w = s_duty_w;

    *output_on = s_output_on;
    *brake_on = s_brake_on;
}

uint8_t Board_IsPwmOffSafe(void)
{
    return (uint8_t)((s_brake_on != 0) && (s_output_on == 0));
}

uint8_t Board_IsPwmRunOk(void)
{
    return (uint8_t)((s_brake_on == 0) && (s_output_on != 0));
}

uint8_t Board_IsPwmSafe(void)
{
    return Board_IsPwmOffSafe();
}

static void SetDriverEnable(uint8_t enable)
{
    /* P16 为板级 3P3N 驱动使能脚，高电平允许功率级输出。 */
    if (enable != 0)
    {
        PORT_SetBit(PORT1, PIN6);
    }
    else
    {
        PORT_ClrBit(PORT1, PIN6);
    }
}

static uint16_t ClampDuty(uint16_t duty)
{
    if (duty < PWM_DUTY_MIN)
    {
        return PWM_DUTY_MIN;
    }
    if (duty > PWM_DUTY_MAX)
    {
        return PWM_DUTY_MAX;
    }
    return duty;
}

static void ConfigPwmPins(void)
{
    /* P10~P15 对应 U/NU/V/NV/W/NW 六路 EPWM 输出。 */
    GPIO_PinAFOutConfig(P10CFG, IO_OUTCFG_P10_EPWM0);
    GPIO_PinAFOutConfig(P11CFG, IO_OUTCFG_P11_EPWM1);
    GPIO_PinAFOutConfig(P12CFG, IO_OUTCFG_P12_EPWM2);
    GPIO_PinAFOutConfig(P13CFG, IO_OUTCFG_P13_EPWM3);
    GPIO_PinAFOutConfig(P14CFG, IO_OUTCFG_P14_EPWM4);
    GPIO_PinAFOutConfig(P15CFG, IO_OUTCFG_P15_EPWM5);

    GPIO_Init(PORT1, PIN0, OUTPUT);
    GPIO_Init(PORT1, PIN1, OUTPUT);
    GPIO_Init(PORT1, PIN2, OUTPUT);
    GPIO_Init(PORT1, PIN3, OUTPUT);
    GPIO_Init(PORT1, PIN4, OUTPUT);
    GPIO_Init(PORT1, PIN5, OUTPUT);
}

static void ConfigAdcTrigger(void)
{
#if (CURRENT_SAMPLE_MULTI_ENABLE != 0U)
    /* 双点诊断模式：两个比较点都沿用 EPWM0 下降计数段。 */
    EPWM_ConfigCompareTriger(EPWM_CMPTG_0, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0,
                             s_adc_trigger_tick_a);
    EPWM_ConfigCompareTriger(EPWM_CMPTG_1, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0,
                             s_adc_trigger_tick_b);
#else
    /* 单点模式保持当前行为。 */
    EPWM_ConfigCompareTriger(EPWM_CMPTG_0, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0,
                             s_adc_trigger_tick);
#endif
}

static void UpdateAdcTriggerTicks(void)
{
#if (CURRENT_SAMPLE_MULTI_ENABLE != 0U)
    uint16_t tick_a;
    uint16_t tick_b;

    if (s_adc_trigger_tick > CURRENT_SAMPLE_MULTI_DELTA_TICK)
    {
        tick_a = (uint16_t)(s_adc_trigger_tick - CURRENT_SAMPLE_MULTI_DELTA_TICK);
    }
    else
    {
        tick_a = 0U;
    }

    tick_b = (uint16_t)(s_adc_trigger_tick + CURRENT_SAMPLE_MULTI_DELTA_TICK);
    if (tick_b > PWM_PERIOD)
    {
        tick_b = PWM_PERIOD;
    }

    s_adc_trigger_tick_a = tick_b;
    s_adc_trigger_tick_b = tick_a;
#else
    s_adc_trigger_tick_a = s_adc_trigger_tick;
    s_adc_trigger_tick_b = s_adc_trigger_tick;
#endif
}

static void ResetPwmState(void)
{
    s_duty_u = PWM_DUTY_50;
    s_duty_v = PWM_DUTY_50;
    s_duty_w = PWM_DUTY_50;
    s_output_on = 0;
    s_brake_on = 1;
}
