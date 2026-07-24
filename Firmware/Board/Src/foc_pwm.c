#include "foc_pwm.h"
#include "BoardConfig.h"
#include "CMS32M6510.h"
#include "TuneConfig.h"
#include "foc_curr.h"
#include "cgc.h"
#include "common.h"
#include "epwm.h"
#include "gpio.h"
#include <stdint.h>

/**
 * @file foc_pwm.c
 * @brief 三相 EPWM 输出和功率级安全态实现。
 */

#define PWM_MAIN_MASK (EPWM_CH_0_MSK | EPWM_CH_2_MSK | EPWM_CH_4_MSK)
#define PWM_ALL_MASK                                                                               \
    (EPWM_CH_0_MSK | EPWM_CH_1_MSK | EPWM_CH_2_MSK | EPWM_CH_3_MSK | EPWM_CH_4_MSK | EPWM_CH_5_MSK)

/** @brief PWM 三相 duty 缓存。 */
typedef struct
{
    uint16_t u;
    uint16_t v;
    uint16_t w;
} PwmDuty;

/** @brief PWM 输出、安全态和 ADC 触发状态缓存。 */
typedef struct
{
    PwmDuty duty;
    uint16_t trig;
    uint16_t trig_a;
    uint16_t trig_b;
    uint8_t out;
    uint8_t brake;
} PwmState;

static PwmState s_pwm = {
    .duty = {PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50},
    .trig = PWM_ADC_TRIGGER_TICK_DEFAULT,
    .trig_a = PWM_ADC_TRIGGER_TICK_DEFAULT,
    .trig_b = PWM_ADC_TRIGGER_TICK_DEFAULT,
    .out = 0U,
    .brake = 1U,
};

static void driver_en(uint8_t en);
static uint16_t clamp_duty(uint16_t duty);
static void map_pwm_duty(uint16_t u_in, uint16_t v_in, uint16_t w_in, PwmDuty *out);
static void pins_init(void);
static void adc_trigger_apply(void);
static void adc_trigger_update(void);
static void state_reset(void);

/** @brief 初始化 EPWM 主通道、互补输出、死区和软件刹车。 */
void pwm_init(void)
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
    EPWM_EnableDeadZone(PWM_ALL_MASK, PWM_DEADTIME_TICKS);
    EPWM_EnableAutoLoadMode(PWM_MAIN_MASK);
    adc_trigger_update();
    adc_trigger_apply();

    EPWM_EnableSoftwareBrake();
    pins_init();
    EPWM_DisableOutput(PWM_ALL_MASK);
    state_reset();

    EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
    EPWM->POREMAP = 0xAA543210;
    EPWM->LOCK = 0x0;

    EPWM_ConfigLoadAndIntMode(EPWM0, EPWM_EACH_ZERO);
    EPWM_ConfigLoadAndIntMode(EPWM2, EPWM_EACH_ZERO);
    EPWM_ConfigLoadAndIntMode(EPWM4, EPWM_EACH_ZERO);
    EPWM_Start(PWM_MAIN_MASK);
    pwm_off();
}

/** @brief 写入三相 duty 并通知电流采样层重选下一拍采样窗口。 */
void pwm_set_duty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w)
{
    /* 只写主通道比较值；互补通道由 EPWM 根据死区配置自动生成。 */
    map_pwm_duty(duty_u, duty_v, duty_w, &s_pwm.duty);

    EPWM_ConfigChannelSymDuty(EPWM0, s_pwm.duty.u);
    EPWM_ConfigChannelSymDuty(EPWM2, s_pwm.duty.v);
    EPWM_ConfigChannelSymDuty(EPWM4, s_pwm.duty.w);
    curr_sync_timing();
}

/** @brief 按 TuneConfig 相序映射三相 duty，并执行 duty 安全限幅。 */
static void map_pwm_duty(uint16_t u_in, uint16_t v_in, uint16_t w_in, PwmDuty *out)
{
#if (MOT_PWM_PHASE_MAP == MOT_PHASE_MAP_UWV)
    out->u = clamp_duty(u_in);
    out->v = clamp_duty(w_in);
    out->w = clamp_duty(v_in);
#elif (MOT_PWM_PHASE_MAP == MOT_PHASE_MAP_VUW)
    out->u = clamp_duty(v_in);
    out->v = clamp_duty(u_in);
    out->w = clamp_duty(w_in);
#elif (MOT_PWM_PHASE_MAP == MOT_PHASE_MAP_VWU)
    out->u = clamp_duty(v_in);
    out->v = clamp_duty(w_in);
    out->w = clamp_duty(u_in);
#elif (MOT_PWM_PHASE_MAP == MOT_PHASE_MAP_WUV)
    out->u = clamp_duty(w_in);
    out->v = clamp_duty(u_in);
    out->w = clamp_duty(v_in);
#elif (MOT_PWM_PHASE_MAP == MOT_PHASE_MAP_WVU)
    out->u = clamp_duty(w_in);
    out->v = clamp_duty(v_in);
    out->w = clamp_duty(u_in);
#else
    out->u = clamp_duty(u_in);
    out->v = clamp_duty(v_in);
    out->w = clamp_duty(w_in);
#endif
}

/** @brief 设置单中心 ADC 触发点，并派生双点触发默认值。 */
void pwm_set_adc_trigger(uint16_t tick)
{
    if (tick > PWM_PERIOD)
    {
        tick = PWM_PERIOD;
    }

    s_pwm.trig = tick;
    adc_trigger_update();
    adc_trigger_apply();
}

/** @brief 返回单中心 ADC 触发点。 */
uint16_t pwm_adc_trigger(void)
{
    return s_pwm.trig;
}

/** @brief 返回双点采样 A 触发点。 */
uint16_t pwm_adc_trigger_a(void)
{
    return s_pwm.trig_a;
}

/** @brief 返回双点采样 B 触发点。 */
uint16_t pwm_adc_trigger_b(void)
{
    return s_pwm.trig_b;
}

/** @brief 直接设置双点 ADC 触发点。 */
void pwm_set_adc_triggers(uint16_t tick_a, uint16_t tick_b)
{
    if (tick_a > PWM_PERIOD)
    {
        tick_a = PWM_PERIOD;
    }
    if (tick_b > PWM_PERIOD)
    {
        tick_b = PWM_PERIOD;
    }

    s_pwm.trig_a = tick_a;
    s_pwm.trig_b = tick_b;
    adc_trigger_apply();
}

/** @brief 关闭驱动输出、打开软件刹车并将 duty 回到 50%。 */
void pwm_off(void)
{
    /*
     * 关断顺序优先关闭驱动使能，再打开软件刹车并关闭 EPWM 输出。
     * 占空比回到 50%，避免下一次使能时残留上一次控制量。
     */
    driver_en(0);
    EPWM_EnableSoftwareBrake();
    EPWM_DisableOutput(PWM_ALL_MASK);

    pwm_set_duty(PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50);

    s_pwm.out = 0U;
    s_pwm.brake = 1U;
}

/** @brief 根据状态机命令打开或关闭功率级输出。 */
uint8_t pwm_enable(uint8_t enable)
{
    if (enable != 0)
    {
        /* 已经运行时不重复写 brake、输出 mask 和驱动使能寄存器。 */
        if ((s_pwm.out != 0U) && (s_pwm.brake == 0U))
        {
            return 1U;
        }

        /*
         * 重新输出前必须清除 mask/brake 状态，再打开 EPWM 输出和驱动使能。
         * 调用者必须已经完成零漂、采样同步和状态机安全检查。
         */
        EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
        EPWM->MASK = 0x00000000;
        EPWM->LOCK = 0x0;

        EPWM_ClearBrakeIntFlag();
        EPWM_ClearBrake();
        EPWM_EnableOutput(PWM_ALL_MASK);
        EPWM_DisableSoftwareBrake();
        driver_en(1);

        s_pwm.out = 1U;
        s_pwm.brake = 0U;
        return 1;
    }

    pwm_off();
    return 0;
}

/** @brief 将 PWM 当前状态复制到 watch/诊断变量。 */
void pwm_snapshot(volatile uint16_t *duty_u,
                  volatile uint16_t *duty_v,
                  volatile uint16_t *duty_w,
                  volatile uint8_t *output_on,
                  volatile uint8_t *brake_on)
{
    *duty_u = s_pwm.duty.u;
    *duty_v = s_pwm.duty.v;
    *duty_w = s_pwm.duty.w;

    *output_on = s_pwm.out;
    *brake_on = s_pwm.brake;
}

/** @brief 判断软件刹车打开且输出关闭。 */
uint8_t pwm_is_off_safe(void)
{
    return (uint8_t)((s_pwm.brake != 0U) && (s_pwm.out == 0U));
}

/** @brief 判断软件刹车释放且输出已打开。 */
uint8_t pwm_is_running(void)
{
    return (uint8_t)((s_pwm.brake == 0U) && (s_pwm.out != 0U));
}

/** @brief 控制板级 P16 驱动使能脚。 */
static void driver_en(uint8_t en)
{
    /* P16 为板级 3P3N 驱动使能脚，高电平允许功率级输出。 */
    if (en != 0U)
    {
        PORT_SetBit(PORT1, PIN6);
    }
    else
    {
        PORT_ClrBit(PORT1, PIN6);
    }
}

/** @brief 将 duty 限制到死区和保护裕量内。 */
static uint16_t clamp_duty(uint16_t duty)
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

/** @brief 配置六路 EPWM 输出引脚。 */
static void pins_init(void)
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

/** @brief 将缓存中的 ADC 触发点写入 EPWM compare trigger。 */
static void adc_trigger_apply(void)
{
#if (CS_MULTI_EN != 0U)
    EPWM_ConfigCompareTriger(EPWM_CMPTG_0, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0, s_pwm.trig_a);
    EPWM_ConfigCompareTriger(EPWM_CMPTG_1, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0, s_pwm.trig_b);
#else
    EPWM_ConfigCompareTriger(EPWM_CMPTG_0, EPWM_CMPTG_FALLING, EPWM_CMPTG_EPWM0, s_pwm.trig);
#endif
}

/** @brief 根据中心触发点生成双点采样 A/B 触发点。 */
static void adc_trigger_update(void)
{
#if (CS_MULTI_EN != 0U)
    uint16_t tick_a;
    uint16_t tick_b;

    if (s_pwm.trig > CS_MULTI_DELTA_TICK)
    {
        tick_a = (uint16_t)(s_pwm.trig - CS_MULTI_DELTA_TICK);
    }
    else
    {
        tick_a = 0U;
    }

    tick_b = (uint16_t)(s_pwm.trig + CS_MULTI_DELTA_TICK);
    if (tick_b > PWM_PERIOD)
    {
        tick_b = PWM_PERIOD;
    }

    s_pwm.trig_a = tick_b;
    s_pwm.trig_b = tick_a;
#else
    s_pwm.trig_a = s_pwm.trig;
    s_pwm.trig_b = s_pwm.trig;
#endif
}

/** @brief 复位 PWM 软件缓存为安全关闭状态。 */
static void state_reset(void)
{
    s_pwm.duty.u = PWM_DUTY_50;
    s_pwm.duty.v = PWM_DUTY_50;
    s_pwm.duty.w = PWM_DUTY_50;
    s_pwm.out = 0U;
    s_pwm.brake = 1U;
}
