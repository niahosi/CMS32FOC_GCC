#include "foc_curr.h"
#include "foc_pwm.h"
#include "adc.h"
#include "adcldo.h"
#include "cgc.h"
#include "gpio.h"
#include "pga.h"

/**
 * @file foc_curr.c
 * @brief 三相电流采样、零漂和 PWM/ADC 同步实现。
 */

#define CURR_PAIR_NONE 255U

typedef struct
{
    int16_t u;
    int16_t v;
    int16_t w;
} Curr3;

typedef struct
{
    uint8_t pair;
    uint32_t channel_mask;
    uint32_t last_channel;
    uint32_t last_channel_mask;
} CurrPairCfg;

typedef struct
{
    volatile uint16_t u;
    volatile uint16_t v;
    volatile uint16_t w;
} CurrRawAdc;

typedef struct
{
    uint16_t u;
    uint16_t v;
    uint16_t w;
} CurrZero;

typedef struct
{
    volatile int16_t u;
    volatile int16_t v;
    volatile int16_t w;
    volatile int16_t sum;
} CurrCache;

typedef struct
{
    volatile uint32_t count;
    volatile uint8_t started;
    uint32_t last;
    uint32_t last_mask;
} CurrSync;

typedef struct
{
    volatile uint8_t multi;
    volatile uint8_t dynamic;
} CurrMode;

typedef struct
{
    volatile uint8_t pair;
    volatile uint8_t hold;
    volatile uint8_t three_shunt;
    volatile uint16_t pair_hold_left;
    volatile uint16_t hold_count;
    volatile uint16_t center;
    volatile uint16_t tick_a;
    volatile uint16_t tick_b;
    volatile uint16_t common;
    volatile uint16_t u;
    volatile uint16_t v;
    volatile uint16_t w;
    volatile uint32_t switch_count;
    volatile uint32_t fallback_count;
} CurrWindow;

typedef struct
{
    volatile uint8_t stage;
    volatile uint8_t count;
    volatile int16_t a0;
    volatile int16_t a1;
    volatile int16_t b0;
    volatile int16_t b1;
    volatile int16_t spread0;
    volatile int16_t spread1;
} CurrDiag;

typedef struct
{
    volatile uint8_t v_valid;
    volatile uint8_t w_valid;
    volatile int16_t prev_v;
    volatile int16_t prev_w;
    volatile uint32_t iv_spike_count;
    volatile uint32_t iw_spike_count;
    volatile uint16_t iv_max_step;
    volatile uint16_t iw_max_step;
} CurrQuality;

typedef struct
{
    Curr3 a;
    Curr3 b;
} CurrPairSample;

static CurrRawAdc s_raw;
static CurrZero s_zero;
static CurrCache s_phys;
static CurrCache s_logic;
static CurrSync s_sync;
static CurrMode s_mode;
static CurrWindow s_win;
static CurrDiag s_diag;
static CurrQuality s_quality;
static CurrPairSample s_sample;

static void pins_init(void);
static void ldo_init(void);
static void pga_init(void);
static void adc_init(void);
static uint16_t read_one(uint32_t ch, uint32_t ch_msk);
static void irq_clear(void);
static void sync_reset(void);
static void diag_clear(void);
static void trigger_update(void);
static void trigger_pair(uint8_t pair);
static uint8_t pair_cfg(uint8_t pair, CurrPairCfg* cfg);
#if ((CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U) || (CURRENT_SAMPLE_MODE != CURRENT_SAMPLE_MODE_FIXED_PAIR))
static void window_select(void);
static void window_apply_pair(uint8_t pair, uint16_t common_width, uint16_t center);
static void window_hold(void);
#if (CURRENT_SAMPLE_MODE == CURRENT_SAMPLE_MODE_VW_PREFERRED_DYNAMIC_PAIR)
static uint8_t pair_window_valid(uint8_t pair, uint16_t* common_width);
static void window_select_vw_preferred(void);
static uint8_t fallback_pair_select(uint8_t* pair, uint16_t* common_width);
#else
static uint8_t pair_select(uint16_t wu, uint16_t wv, uint16_t ww, uint8_t* pair,
                              uint16_t* common_width);
#endif
static void pair_try(uint8_t pair, uint16_t wa, uint16_t wb, uint8_t* best_pair,
                    uint16_t* best_common, uint16_t* best_sum);
#endif
static void sample_pair(Curr3* sample);
static void sample_resolve(void);
static void reconstruct(uint8_t pair, const Curr3* sample,
                                       Curr3* physical);
static void apply_phys(const Curr3* physical);
static void map_logic(const Curr3* physical);
static void quality_clear(void);
static void quality_update(const Curr3* physical);
static void quality_update_phase(int16_t value, volatile uint8_t* valid,
                                 volatile int16_t* previous, volatile uint32_t* spike_count,
                                 volatile uint16_t* max_step);
static uint8_t pair_samples_phys_v(uint8_t pair);
static uint8_t pair_samples_phys_w(uint8_t pair);
#if ((CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U) || (CURRENT_SAMPLE_MODE != CURRENT_SAMPLE_MODE_FIXED_PAIR))
static uint16_t clamp_tick(uint16_t tick);
static uint16_t clamp_sample_tick(uint16_t tick);
static uint16_t min_u16(uint16_t a, uint16_t b);
#endif
static uint16_t abs_diff_i16(int16_t a, int16_t b);
static int16_t clamp_spread(int32_t value);
static uint16_t cnt_to_adc(int16_t cnt, uint16_t offset);

void curr_init(void)
{
    s_mode.multi = (uint8_t)(CURRENT_SAMPLE_MULTI_ENABLE != 0U);
    s_mode.dynamic = (uint8_t)(CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U);

    pins_init();
    ldo_init();
    pga_init();
    adc_init();
    diag_clear();
}

void curr_sample_raw(void)
{
    s_raw.u = read_one(ADC_CH_0, ADC_CH_0_MSK);
    s_raw.v = read_one(ADC_CH_2, ADC_CH_2_MSK);
    s_raw.w = read_one(ADC_CH_3, ADC_CH_3_MSK);
}

void curr_calib(uint16_t samples)
{
    uint32_t sum_u = 0;
    uint32_t sum_v = 0;
    uint32_t sum_w = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        curr_sample_raw();

        sum_u += s_raw.u;
        sum_v += s_raw.v;
        sum_w += s_raw.w;
    }

    s_zero.u = (uint16_t)(sum_u / samples);
    s_zero.v = (uint16_t)(sum_v / samples);
    s_zero.w = (uint16_t)(sum_w / samples);
}

void curr_calib_pwm(uint16_t samples)
{
    uint32_t sum_u = 0;
    uint32_t sum_v = 0;
    uint32_t sum_w = 0;

    NVIC_DisableIRQ(ADC_IRQn);
    ADC_DisableHardwareTrigger(ADC_TG_EPWM_CMP0);
    ADC_DisableHardwareTrigger(ADC_TG_EPWM_CMP1);
    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_SINGLE, ADC_CLK_DIV_1, 25);

    for (uint16_t i = 0; i < samples; i++)
    {
        curr_sample_raw();

        sum_u += s_raw.u;
        sum_v += s_raw.v;
        sum_w += s_raw.w;
    }

    s_zero.u = (uint16_t)(sum_u / samples);
    s_zero.v = (uint16_t)(sum_v / samples);
    s_zero.w = (uint16_t)(sum_w / samples);

    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_CONTINUOUS, ADC_CLK_DIV_1, 25);
    trigger_update();
    irq_clear();
    ADC_DisableChannelInt(ADC_CH_0_MSK | ADC_CH_2_MSK | ADC_CH_3_MSK);
    ADC_EnableChannelInt(s_sync.last_mask);
    NVIC_EnableIRQ(ADC_IRQn);
    ADC_Go();
    
}

void curr_update(void)
{
    Curr3 physical;

    physical.u = (int16_t)((int16_t)s_raw.u - (int16_t)s_zero.u);
    physical.v = (int16_t)((int16_t)s_raw.v - (int16_t)s_zero.v);
    physical.w = (int16_t)((int16_t)s_raw.w - (int16_t)s_zero.w);
    apply_phys(&physical);
}

void curr_sync_init(void)
{
    sync_reset();
    s_sync.started = 1U;

    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_CONTINUOUS, ADC_CLK_DIV_1, 25);
    trigger_update();
    irq_clear();
    NVIC_EnableIRQ(ADC_IRQn);
    ADC_Go();
}

void curr_sync_timing(void)
{
    if (s_sync.started == 0U)
    {
        return;
    }

    trigger_update();
}

uint8_t curr_irq(void)
{
    if (ADC_GetChannelIntFlag(s_sync.last) == 0U)
    {
        return 0U;
    }

    ADC_ClearChannelIntFlag(s_sync.last);

    if (s_mode.multi != 0U)
    {
        if (s_diag.stage == 0U)
        {
            sample_pair(&s_sample.a);
            s_diag.stage = 1U;
            s_diag.count = 1U;
            return 0U;
        }

        sample_pair(&s_sample.b);
        s_diag.stage = 0U;
        s_diag.count = 2U;
        sample_resolve();
    }
    else
    {
        sample_pair(&s_sample.a);
        s_sample.b = s_sample.a;
        s_diag.count = 1U;
        sample_resolve();
    }

    s_sync.count++;
    return 1U;
}

int16_t curr_u(void)
{
    return s_logic.u;
}

int16_t curr_v(void)
{
    return s_logic.v;
}

int16_t curr_w(void)
{
    return s_logic.w;
}

int16_t curr_sum(void)
{
    return s_logic.sum;
}

int16_t curr_raw_u(void)
{
    return s_phys.u;
}

int16_t curr_raw_v(void)
{
    return s_phys.v;
}

int16_t curr_raw_w(void)
{
    return s_phys.w;
}

int16_t curr_raw_sum(void)
{
    return s_phys.sum;
}

uint16_t curr_raw_adc_u(void)
{
    return s_raw.u;
}

uint16_t curr_raw_adc_v(void)
{
    return s_raw.v;
}

uint16_t curr_raw_adc_w(void)
{
    return s_raw.w;
}

uint32_t curr_sync_count(void)
{
    return s_sync.count;
}

uint8_t curr_multi_enabled(void)
{
    return s_mode.multi;
}

uint8_t curr_dynamic_enabled(void)
{
    return s_mode.dynamic;
}

uint8_t curr_pair(void)
{
    return s_win.pair;
}

uint8_t curr_is_hold(void)
{
    return s_win.hold;
}

uint16_t curr_hold_count(void)
{
    return s_win.hold_count;
}

uint16_t curr_center_tick(void)
{
    return s_win.center;
}

uint16_t curr_window_u(void)
{
    return s_win.u;
}

uint16_t curr_window_v(void)
{
    return s_win.v;
}

uint16_t curr_window_w(void)
{
    return s_win.w;
}

uint8_t curr_three_shunt_active(void)
{
    return s_win.three_shunt;
}

uint16_t curr_window_common(void)
{
    return s_win.common;
}

uint32_t curr_sample_switch_count(void)
{
    return s_win.switch_count;
}

uint32_t curr_sample_fallback_count(void)
{
    return s_win.fallback_count;
}

uint16_t curr_sample_pair_hold_left(void)
{
    return s_win.pair_hold_left;
}

uint32_t curr_iv_spike_count(void)
{
    return s_quality.iv_spike_count;
}

uint32_t curr_iw_spike_count(void)
{
    return s_quality.iw_spike_count;
}

uint16_t curr_iv_max_step(void)
{
    return s_quality.iv_max_step;
}

uint16_t curr_iw_max_step(void)
{
    return s_quality.iw_max_step;
}

uint8_t curr_diag_stage(void)
{
    return s_diag.stage;
}

uint8_t curr_diag_count(void)
{
    return s_diag.count;
}

int16_t curr_diag_a0(void)
{
    return s_diag.a0;
}

int16_t curr_diag_a1(void)
{
    return s_diag.a1;
}

int16_t curr_diag_b0(void)
{
    return s_diag.b0;
}

int16_t curr_diag_b1(void)
{
    return s_diag.b1;
}

int16_t curr_diag_spread0(void)
{
    return s_diag.spread0;
}

int16_t curr_diag_spread1(void)
{
    return s_diag.spread1;
}

static void pins_init(void)
{
    GPIO_Init(PORT0, PIN0, ANALOG_INPUT);
    GPIO_Init(PORT0, PIN1, ANALOG_INPUT);

    GPIO_Init(PORT2, PIN4, ANALOG_INPUT);
    GPIO_Init(PORT2, PIN5, ANALOG_INPUT);

    GPIO_Init(PORT2, PIN6, ANALOG_INPUT);
    GPIO_Init(PORT2, PIN7, ANALOG_INPUT);

    GPIO_Init(PORT2, PIN0, ANALOG_INPUT);
}

static void ldo_init(void)
{
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCLDO, ENABLE);

    ADCLDO_OutVlotageSel(ADCLDO_OutV_3d6);
    ADCLDO_Enable();
}

static void pga_init(void)
{
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_PGA0EN, ENABLE);
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_PGA1EN, ENABLE);
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_PGA2EN, ENABLE);

    PGA_ModeSet(PGA0x, PgaDiffer);
    PGA_ModeSet(PGA1x, PgaDiffer);
    PGA_ModeSet(PGA2x, PgaDiffer);

    PGA_VrefCtrl(PGA0x, VrefHalf);
    PGA_VrefCtrl(PGA1x, VrefHalf);
    PGA_VrefCtrl(PGA2x, VrefHalf);

    PGA_ConfigGain(PGA0x, PGA_GAIN_2);
    PGA_ConfigGain(PGA1x, PGA_GAIN_2);
    PGA_ConfigGain(PGA2x, PGA_GAIN_2);

    PGA_EnableOutput(PGA0x);
    PGA_EnableOutput(PGA1x);
    PGA_EnableOutput(PGA2x);

    PGA_Start(PGA0x);
    PGA_Start(PGA1x);
    PGA_Start(PGA2x);
}

static void adc_init(void)
{
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCEN, ENABLE);

    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_SINGLE, ADC_CLK_DIV_1, 25);
    ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);
    ADC_DisableChargeAndDischarge();
    ADC_ConfigVREF(ADC_VREFP_AVREFP);
    ADC_Start();
}

static uint16_t read_one(uint32_t ch, uint32_t ch_msk)
{
    ADC_DisableScanChannel(0xFFFFFFFFUL);
    ADC_EnableScanChannel(ch_msk);

    ADC_Go();

    while (ADC_IS_BUSY())
    {
    }

    return (uint16_t)ADC_GetResult(ch);
}

static void irq_clear(void)
{
    ADC_ClearChannelIntFlag(ADC_CH_0);
    ADC_ClearChannelIntFlag(ADC_CH_2);
    ADC_ClearChannelIntFlag(ADC_CH_3);
    NVIC_ClearPendingIRQ(ADC_IRQn);
}

static void sync_reset(void)
{
    s_sync.count = 0;
    s_sync.started = 0U;
    diag_clear();
}

static void diag_clear(void)
{
    s_diag.stage = 0U;
    s_diag.count = 0U;
    s_win.pair = CURR_PAIR_NONE;
    s_win.hold = 0U;
    s_win.three_shunt = 0U;
    s_win.pair_hold_left = 0U;
    s_win.hold_count = 0U;
    s_win.center = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.tick_a = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.tick_b = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.common = 0U;
    s_win.u = 0U;
    s_win.v = 0U;
    s_win.w = 0U;
    s_win.switch_count = 0U;
    s_win.fallback_count = 0U;
    s_diag.a0 = 0;
    s_diag.a1 = 0;
    s_diag.b0 = 0;
    s_diag.b1 = 0;
    s_diag.spread0 = 0;
    s_diag.spread1 = 0;
    s_sample.a.u = 0;
    s_sample.a.v = 0;
    s_sample.a.w = 0;
    s_sample.b.u = 0;
    s_sample.b.v = 0;
    s_sample.b.w = 0;
    quality_clear();
}

static void trigger_update(void)
{
#if ((CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U) || (CURRENT_SAMPLE_MODE != CURRENT_SAMPLE_MODE_FIXED_PAIR))
    window_select();
#else
    s_win.pair = CURRENT_SAMPLE_PAIR;
    s_win.three_shunt = 0U;
    s_win.common = 0U;
    s_win.center = pwm_adc_trigger();
    pwm_set_adc_trigger(s_win.center);
    trigger_pair(s_win.pair);
#endif
}

static void trigger_pair(uint8_t pair)
{
    CurrPairCfg cfg;

    if (pair_cfg(pair, &cfg) == 0U)
    {
        return;
    }

    ADC_DisableHardwareTrigger(ADC_TG_EPWM_CMP0);
    ADC_DisableHardwareTrigger(ADC_TG_EPWM_CMP1);
    ADC_DisableHardwareTrigger(ADC_TG_EPWM0_PERIOD);
    ADC_DisableHardwareTrigger(ADC_TG_EPWM0_ZERO);

    ADC_DisableEPWMTriggerChannel(0xFFFFFFFFUL);
    ADC_DisableEPWMCmp0TriggerChannel(0xFFFFFFFFUL);
    ADC_DisableEPWMCmp1TriggerChannel(0xFFFFFFFFUL);

    ADC_DisableScanChannel(0xFFFFFFFFUL);
    ADC_EnableScanChannel(cfg.channel_mask);

    s_diag.stage = 0U;
    s_diag.count = 0U;

    ADC_DisableChannelInt(ADC_CH_0_MSK | ADC_CH_2_MSK | ADC_CH_3_MSK);
    irq_clear();
    ADC_EnableChannelInt(cfg.last_channel_mask);

    ADC_EnableEPWMCmp0TriggerChannel(cfg.channel_mask);
    ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP0);
    if (s_mode.multi != 0U)
    {
        ADC_EnableEPWMCmp1TriggerChannel(cfg.channel_mask);
        ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP1);
    }

    s_sync.last = cfg.last_channel;
    s_sync.last_mask = cfg.last_channel_mask;
}

static uint8_t pair_cfg(uint8_t pair, CurrPairCfg* cfg)
{
    if (cfg == 0)
    {
        return 0U;
    }

    cfg->pair = pair;
    switch (pair)
    {
        case CURRENT_SAMPLE_UV:
            cfg->channel_mask = ADC_CH_0_MSK | ADC_CH_2_MSK;
            cfg->last_channel = ADC_CH_2;
            cfg->last_channel_mask = ADC_CH_2_MSK;
            return 1U;

        case CURRENT_SAMPLE_UW:
            cfg->channel_mask = ADC_CH_0_MSK | ADC_CH_3_MSK;
            cfg->last_channel = ADC_CH_3;
            cfg->last_channel_mask = ADC_CH_3_MSK;
            return 1U;

        case CURRENT_SAMPLE_UVW:
        case CURRENT_SAMPLE_UVW_DIAG:
            cfg->channel_mask = ADC_CH_0_MSK | ADC_CH_2_MSK | ADC_CH_3_MSK;
            cfg->last_channel = ADC_CH_3;
            cfg->last_channel_mask = ADC_CH_3_MSK;
            return 1U;

        case CURRENT_SAMPLE_VW:
        default:
            cfg->channel_mask = ADC_CH_2_MSK | ADC_CH_3_MSK;
            cfg->last_channel = ADC_CH_3;
            cfg->last_channel_mask = ADC_CH_3_MSK;
            return 1U;
    }
}

#if ((CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U) || (CURRENT_SAMPLE_MODE != CURRENT_SAMPLE_MODE_FIXED_PAIR))
static void window_select(void)
{
    volatile uint16_t duty_u;
    volatile uint16_t duty_v;
    volatile uint16_t duty_w;
    volatile uint8_t output_on;
    volatile uint8_t brake_on;

    pwm_snapshot(&duty_u, &duty_v, &duty_w, &output_on, &brake_on);
    (void)output_on;
    (void)brake_on;

    s_win.u = clamp_tick((uint16_t)duty_u);
    s_win.v = clamp_tick((uint16_t)duty_v);
    s_win.w = clamp_tick((uint16_t)duty_w);

#if (CURRENT_SAMPLE_MODE == CURRENT_SAMPLE_MODE_VW_PREFERRED_DYNAMIC_PAIR)
    window_select_vw_preferred();
    return;
#else
#if (CURRENT_SAMPLE_MODE == CURRENT_SAMPLE_MODE_DYNAMIC_3SHUNT)
    uint16_t three_common;

    three_common = min_u16(s_win.u, min_u16(s_win.v, s_win.w));
    if (three_common >= CURRENT_SAMPLE_MIN_WINDOW_TICK)
    {
        window_apply_pair(CURRENT_SAMPLE_UVW, three_common, (uint16_t)(three_common / 2U));
        return;
    }
#endif

    uint8_t pair;
    uint16_t common_width;

    if (pair_select(s_win.u, s_win.v,
                       s_win.w, &pair, &common_width) == 0U)
    {
        window_hold();
        return;
    }

    window_apply_pair(pair, common_width, (uint16_t)(common_width / 2U));
#endif
}

static void window_apply_pair(uint8_t pair, uint16_t common_width, uint16_t center)
{
    uint16_t delta;

    if ((s_win.pair != pair) && (s_win.pair != CURR_PAIR_NONE))
    {
        if (s_win.switch_count < 0xFFFFFFFFUL)
        {
            s_win.switch_count++;
        }
        s_win.pair_hold_left = CURRENT_SAMPLE_PAIR_MIN_HOLD_PWM;
    }
    else if (s_win.pair_hold_left > 0U)
    {
        s_win.pair_hold_left--;
    }

    center = clamp_sample_tick(center);
    delta = (uint16_t)(common_width / 4U);
    if (delta > CURRENT_SAMPLE_MULTI_DELTA_TICK)
    {
        delta = CURRENT_SAMPLE_MULTI_DELTA_TICK;
    }
    if (delta == 0U)
    {
        delta = 1U;
    }

    s_win.hold = 0U;
    s_win.three_shunt =
        (uint8_t)((pair == CURRENT_SAMPLE_UVW) || (pair == CURRENT_SAMPLE_UVW_DIAG));
    s_win.pair = pair;
    s_win.common = common_width;
    s_win.center = center;
    s_win.tick_a = clamp_tick((uint16_t)(center + delta));
    s_win.tick_b = (center > delta) ? (uint16_t)(center - delta) : 0U;

    if (s_mode.multi != 0U)
    {
        pwm_set_adc_triggers(s_win.tick_a, s_win.tick_b);
    }
    else
    {
        pwm_set_adc_trigger(center);
    }
    trigger_pair(pair);
}

static void window_hold(void)
{
    s_win.hold = 1U;
    if (s_win.hold_count < 65535U)
    {
        s_win.hold_count++;
    }
    if (s_win.pair_hold_left > 0U)
    {
        s_win.pair_hold_left--;
    }
    if (s_win.pair == CURR_PAIR_NONE)
    {
        s_win.pair = CURRENT_SAMPLE_PAIR;
        s_win.three_shunt = 0U;
        s_win.common = 0U;
        s_win.center = CURRENT_SAMPLE_VW_TRIGGER_TICK;
        s_win.tick_a = CURRENT_SAMPLE_VW_TRIGGER_TICK;
        s_win.tick_b = CURRENT_SAMPLE_VW_TRIGGER_TICK;
        pwm_set_adc_trigger(CURRENT_SAMPLE_VW_TRIGGER_TICK);
        trigger_pair(s_win.pair);
    }
}

#if (CURRENT_SAMPLE_MODE == CURRENT_SAMPLE_MODE_VW_PREFERRED_DYNAMIC_PAIR)
static uint8_t pair_window_valid(uint8_t pair, uint16_t* common_width)
{
    uint16_t common;

    switch (pair)
    {
        case CURRENT_SAMPLE_UV:
            if ((s_win.u < CURRENT_SAMPLE_MIN_WINDOW_TICK) ||
                (s_win.v < CURRENT_SAMPLE_MIN_WINDOW_TICK))
            {
                return 0U;
            }
            common = min_u16(s_win.u, s_win.v);
            break;

        case CURRENT_SAMPLE_UW:
            if ((s_win.u < CURRENT_SAMPLE_MIN_WINDOW_TICK) ||
                (s_win.w < CURRENT_SAMPLE_MIN_WINDOW_TICK))
            {
                return 0U;
            }
            common = min_u16(s_win.u, s_win.w);
            break;

        case CURRENT_SAMPLE_VW:
            if ((s_win.v < CURRENT_SAMPLE_MIN_WINDOW_TICK) ||
                (s_win.w < CURRENT_SAMPLE_MIN_WINDOW_TICK))
            {
                return 0U;
            }
            common = min_u16(s_win.v, s_win.w);
            break;

        default:
            return 0U;
    }

    if (common_width != 0)
    {
        *common_width = common;
    }
    return 1U;
}

static void window_select_vw_preferred(void)
{
    uint8_t pair;
    uint16_t common_width;
    uint16_t vw_common;
    uint8_t vw_valid = 0U;

    if (pair_window_valid(CURRENT_SAMPLE_VW, &vw_common) != 0U)
    {
        vw_valid = 1U;
    }

    if ((s_win.pair != CURRENT_SAMPLE_VW) && (s_win.pair != CURR_PAIR_NONE) &&
        (s_win.pair_hold_left > 0U) && (pair_window_valid(s_win.pair, &common_width) != 0U))
    {
        if (s_win.fallback_count < 0xFFFFFFFFUL)
        {
            s_win.fallback_count++;
        }
        window_apply_pair(s_win.pair, common_width, (uint16_t)(common_width / 2U));
        return;
    }

    if (vw_valid != 0U)
    {
        if ((s_win.pair == CURRENT_SAMPLE_VW) ||
            (vw_common >= (uint16_t)(CURRENT_SAMPLE_MIN_WINDOW_TICK +
                                     CURRENT_SAMPLE_SWITCH_HYST_TICK)) ||
            (s_win.pair == CURR_PAIR_NONE))
        {
            window_apply_pair(CURRENT_SAMPLE_VW, vw_common, CURRENT_SAMPLE_VW_TRIGGER_TICK);
            return;
        }
    }

    if (fallback_pair_select(&pair, &common_width) == 0U)
    {
        if (vw_valid != 0U)
        {
            window_apply_pair(CURRENT_SAMPLE_VW, vw_common, CURRENT_SAMPLE_VW_TRIGGER_TICK);
            return;
        }

        window_hold();
        return;
    }

    if (s_win.fallback_count < 0xFFFFFFFFUL)
    {
        s_win.fallback_count++;
    }
    window_apply_pair(pair, common_width, (uint16_t)(common_width / 2U));
}

static uint8_t fallback_pair_select(uint8_t* pair, uint16_t* common_width)
{
    uint8_t best_pair = CURR_PAIR_NONE;
    uint16_t best_common = 0U;
    uint16_t best_sum = 0U;

    pair_try(CURRENT_SAMPLE_UW, s_win.u, s_win.w, &best_pair, &best_common, &best_sum);
    pair_try(CURRENT_SAMPLE_UV, s_win.u, s_win.v, &best_pair, &best_common, &best_sum);

    if (best_pair == CURR_PAIR_NONE)
    {
        return 0U;
    }

    *pair = best_pair;
    *common_width = best_common;
    return 1U;
}
#else
static uint8_t pair_select(uint16_t wu, uint16_t wv, uint16_t ww, uint8_t* pair,
                              uint16_t* common_width)
{
    uint8_t best_pair = CURR_PAIR_NONE;
    uint16_t best_common = 0U;
    uint16_t best_sum = 0U;

    pair_try(CURRENT_SAMPLE_UV, wu, wv, &best_pair, &best_common, &best_sum);
    pair_try(CURRENT_SAMPLE_UW, wu, ww, &best_pair, &best_common, &best_sum);
    pair_try(CURRENT_SAMPLE_VW, wv, ww, &best_pair, &best_common, &best_sum);

    if (best_pair == CURR_PAIR_NONE)
    {
        return 0U;
    }

    *pair = best_pair;
    *common_width = best_common;
    return 1U;
}
#endif

static void pair_try(uint8_t pair, uint16_t wa, uint16_t wb, uint8_t* best_pair,
                    uint16_t* best_common, uint16_t* best_sum)
{
    uint16_t common;
    uint16_t sum;

    if ((wa < CURRENT_SAMPLE_MIN_WINDOW_TICK) || (wb < CURRENT_SAMPLE_MIN_WINDOW_TICK))
    {
        return;
    }

    common = min_u16(wa, wb);
    sum = (uint16_t)(wa + wb);
    if ((common > *best_common) || ((common == *best_common) && (sum > *best_sum)))
    {
        *best_pair = pair;
        *best_common = common;
        *best_sum = sum;
    }
}
#endif

static void sample_pair(Curr3* sample)
{
    int16_t first = 0;
    int16_t second = 0;

    sample->u = 0;
    sample->v = 0;
    sample->w = 0;

    switch (s_win.pair)
    {
        case CURRENT_SAMPLE_UVW:
        case CURRENT_SAMPLE_UVW_DIAG:
            sample->u = (int16_t)((int16_t)ADC_GetResult(ADC_CH_0) - (int16_t)s_zero.u);
            sample->v = (int16_t)((int16_t)ADC_GetResult(ADC_CH_2) - (int16_t)s_zero.v);
            sample->w = (int16_t)((int16_t)ADC_GetResult(ADC_CH_3) - (int16_t)s_zero.w);
            first = sample->u;
            second = sample->v;
            break;

        case CURRENT_SAMPLE_UV:
            sample->u = (int16_t)((int16_t)ADC_GetResult(ADC_CH_0) - (int16_t)s_zero.u);
            sample->v = (int16_t)((int16_t)ADC_GetResult(ADC_CH_2) - (int16_t)s_zero.v);
            first = sample->u;
            second = sample->v;
            break;

        case CURRENT_SAMPLE_UW:
            sample->u = (int16_t)((int16_t)ADC_GetResult(ADC_CH_0) - (int16_t)s_zero.u);
            sample->w = (int16_t)((int16_t)ADC_GetResult(ADC_CH_3) - (int16_t)s_zero.w);
            first = sample->u;
            second = sample->w;
            break;

        case CURRENT_SAMPLE_VW:
        default:
            sample->v = (int16_t)((int16_t)ADC_GetResult(ADC_CH_2) - (int16_t)s_zero.v);
            sample->w = (int16_t)((int16_t)ADC_GetResult(ADC_CH_3) - (int16_t)s_zero.w);
            first = sample->v;
            second = sample->w;
            break;
    }

    if (s_diag.stage == 0U)
    {
        s_diag.a0 = first;
        s_diag.a1 = second;
    }
    else
    {
        s_diag.b0 = first;
        s_diag.b1 = second;
    }
}

static void sample_resolve(void)
{
    Curr3 avg;
    Curr3 physical;

    if (s_win.hold != 0U)
    {
        return;
    }

    avg.u = (int16_t)(((int32_t)s_sample.a.u + (int32_t)s_sample.b.u) / 2);
    avg.v = (int16_t)(((int32_t)s_sample.a.v + (int32_t)s_sample.b.v) / 2);
    avg.w = (int16_t)(((int32_t)s_sample.a.w + (int32_t)s_sample.b.w) / 2);

    s_diag.spread0 =
        clamp_spread((int32_t)s_diag.b0 -
                                 (int32_t)s_diag.a0);
    s_diag.spread1 =
        clamp_spread((int32_t)s_diag.b1 -
                                 (int32_t)s_diag.a1);
    reconstruct(s_win.pair, &avg, &physical);
    apply_phys(&physical);
}

static void reconstruct(uint8_t pair, const Curr3* sample,
                                       Curr3* physical)
{
    physical->u = sample->u;
    physical->v = sample->v;
    physical->w = sample->w;

    switch (pair)
    {
        case CURRENT_SAMPLE_UVW:
        case CURRENT_SAMPLE_UVW_DIAG:
            break;

        case CURRENT_SAMPLE_UV:
            physical->w = (int16_t)(-physical->u - physical->v);
            break;

        case CURRENT_SAMPLE_UW:
            physical->v = (int16_t)(-physical->u - physical->w);
            break;

        case CURRENT_SAMPLE_VW:
        default:
            physical->u = (int16_t)(-physical->v - physical->w);
            break;
    }
}

static void apply_phys(const Curr3* physical)
{
    s_phys.u = physical->u;
    s_phys.v = physical->v;
    s_phys.w = physical->w;
    s_phys.sum = (int16_t)(physical->u + physical->v + physical->w);

    s_raw.u = cnt_to_adc(physical->u, s_zero.u);
    s_raw.v = cnt_to_adc(physical->v, s_zero.v);
    s_raw.w = cnt_to_adc(physical->w, s_zero.w);

    map_logic(physical);
    quality_update(physical);
}

static void map_logic(const Curr3* physical)
{
    /*
     * 保持现有 FOC 坐标：logic U/V/W = physical V/W/U。
     * 这样动态切 UV/UW/VW 采样时，不需要重新改电角零点和相序。
     */
    s_logic.u = physical->v;
    s_logic.v = physical->w;
    s_logic.w = physical->u;
    s_logic.sum = (int16_t)(s_logic.u + s_logic.v + s_logic.w);
}

static void quality_clear(void)
{
    s_quality.v_valid = 0U;
    s_quality.w_valid = 0U;
    s_quality.prev_v = 0;
    s_quality.prev_w = 0;
    s_quality.iv_spike_count = 0U;
    s_quality.iw_spike_count = 0U;
    s_quality.iv_max_step = 0U;
    s_quality.iw_max_step = 0U;
}

static void quality_update(const Curr3* physical)
{
    if (pair_samples_phys_v(s_win.pair) != 0U)
    {
        quality_update_phase(physical->v, &s_quality.v_valid, &s_quality.prev_v,
                             &s_quality.iv_spike_count, &s_quality.iv_max_step);
    }
    else
    {
        s_quality.v_valid = 0U;
    }

    if (pair_samples_phys_w(s_win.pair) != 0U)
    {
        quality_update_phase(physical->w, &s_quality.w_valid, &s_quality.prev_w,
                             &s_quality.iw_spike_count, &s_quality.iw_max_step);
    }
    else
    {
        s_quality.w_valid = 0U;
    }
}

static void quality_update_phase(int16_t value, volatile uint8_t* valid,
                                 volatile int16_t* previous, volatile uint32_t* spike_count,
                                 volatile uint16_t* max_step)
{
    uint16_t step;

    if (*valid == 0U)
    {
        *previous = value;
        *valid = 1U;
        return;
    }

    step = abs_diff_i16(value, *previous);
    if (step > *max_step)
    {
        *max_step = step;
    }
    if (step > (uint16_t)CURRENT_SAMPLE_SPIKE_LIMIT_CNT)
    {
        if (*spike_count < 0xFFFFFFFFUL)
        {
            (*spike_count)++;
        }
    }
    *previous = value;
}

static uint8_t pair_samples_phys_v(uint8_t pair)
{
    return (uint8_t)((pair == CURRENT_SAMPLE_UV) || (pair == CURRENT_SAMPLE_VW) ||
                     (pair == CURRENT_SAMPLE_UVW) || (pair == CURRENT_SAMPLE_UVW_DIAG));
}

static uint8_t pair_samples_phys_w(uint8_t pair)
{
    return (uint8_t)((pair == CURRENT_SAMPLE_UW) || (pair == CURRENT_SAMPLE_VW) ||
                     (pair == CURRENT_SAMPLE_UVW) || (pair == CURRENT_SAMPLE_UVW_DIAG));
}

#if ((CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U) || (CURRENT_SAMPLE_MODE != CURRENT_SAMPLE_MODE_FIXED_PAIR))
static uint16_t clamp_tick(uint16_t tick)
{
    if (tick > PWM_PERIOD)
    {
        return PWM_PERIOD;
    }
    return tick;
}

static uint16_t clamp_sample_tick(uint16_t tick)
{
    const uint16_t max_tick = (PWM_PERIOD > PWM_DEADTIME_TICKS)
                                  ? (uint16_t)(PWM_PERIOD - PWM_DEADTIME_TICKS)
                                  : PWM_PERIOD;

    tick = clamp_tick(tick);
    if (tick < PWM_DEADTIME_TICKS)
    {
        return PWM_DEADTIME_TICKS;
    }
    if (tick > max_tick)
    {
        return max_tick;
    }
    return tick;
}

static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}
#endif

static uint16_t abs_diff_i16(int16_t a, int16_t b)
{
    int32_t diff = (int32_t)a - (int32_t)b;

    if (diff < 0)
    {
        diff = -diff;
    }
    if (diff > 65535)
    {
        return 65535U;
    }
    return (uint16_t)diff;
}

static int16_t clamp_spread(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

static uint16_t cnt_to_adc(int16_t cnt, uint16_t offset)
{
    int32_t adc = (int32_t)cnt + (int32_t)offset;

    if (adc < 0)
    {
        return 0U;
    }
    if (adc > 4095)
    {
        return 4095U;
    }
    return (uint16_t)adc;
}
