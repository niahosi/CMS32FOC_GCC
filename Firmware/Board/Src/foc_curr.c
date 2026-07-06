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
 *
 * 当前按 TI 三电阻 T1/T2/T3 思路选择采样点，但映射到 CMS32 中心对齐
 * FALLING 段的“下降计数经过 duty 后低边打开”半周，尽量采在低边刚稳定后。
 */

#define CURR_PAIR_NONE 255U
#define CURR_PAIR_ALL 254U
#define CURR_PHASE_U 0U
#define CURR_PHASE_V 1U
#define CURR_PHASE_W 2U
#define CURR_VALID_U 0x01U
#define CURR_VALID_V 0x02U
#define CURR_VALID_W 0x04U
#define CURR_RECON_PAIR 0U
#define CURR_RECON_ALL 1U
#define CURR_RECON_FILTER 2U

typedef struct
{
    int16_t u;
    int16_t v;
    int16_t w;
} Curr3;

typedef struct
{
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
    volatile uint8_t pair;
    volatile uint8_t hold;
    volatile uint8_t blank_count;
    volatile uint8_t valid_mask;
    volatile uint8_t recon_mode;
    volatile uint8_t region;
    volatile uint8_t bad_count;
    volatile uint8_t fault;
    volatile uint16_t pair_hold_count;
    volatile uint16_t hold_count;
    volatile uint16_t center;
    volatile uint16_t tick_a;
    volatile uint16_t tick_b;
    volatile int16_t center_bias;
    volatile uint8_t single_point;
    volatile uint16_t common;
    volatile uint16_t u;
    volatile uint16_t v;
    volatile uint16_t w;
    volatile uint16_t t1;
    volatile uint16_t t2;
    volatile uint16_t t3;
    volatile uint32_t switch_count;
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

typedef struct
{
    int32_t u;
    int32_t v;
    int32_t w;
    uint8_t seeded;
} CurrFilter;

typedef struct
{
    uint8_t phase;
    uint16_t duty;
} CurrDutyItem;

static CurrRawAdc s_raw;
static CurrZero s_zero;
static CurrCache s_phys;
static CurrCache s_logic;
static CurrSync s_sync;
static CurrWindow s_win;
static CurrDiag s_diag;
static CurrQuality s_quality;
static CurrPairSample s_sample;
static CurrFilter s_filter;
static volatile uint16_t s_vf_voltage_abs;

static void pins_init(void);
static void ldo_init(void);
static void pga_init(void);
static void adc_init(void);
static uint16_t adc_result12(uint32_t ch);
static uint16_t read_one(uint32_t ch, uint32_t ch_msk);
static void irq_clear(void);
static void sync_reset(void);
static void diag_clear(void);
static void trigger_update(void);
static void trigger_pair(uint8_t pair);
static uint8_t pair_cfg(uint8_t pair, CurrPairCfg* cfg);
static uint8_t high_vf_single_active(void);
static void window_select(void);
static uint8_t ti_window_select(uint8_t* pair, uint16_t* center, uint16_t* width,
                                uint8_t* valid_mask, uint8_t* recon_mode,
                                uint8_t* region);
static void sort_duty_desc(CurrDutyItem* a, CurrDutyItem* b, CurrDutyItem* c);
static uint8_t pair_from_phases(uint8_t a, uint8_t b);
static uint8_t valid_bit(uint8_t phase);
static void window_apply_pair(uint8_t pair, uint16_t common_width, int16_t center_bias);
static void window_hold(void);
static void reject_bad_sample(void);
static int16_t pair_bias_tick(uint8_t pair);
static void sample_pair(Curr3* sample);
static void sample_resolve(void);
static uint8_t pair_sample_in_range(uint8_t pair, const Curr3* sample);
static uint8_t physical_in_hard_range(const Curr3* physical);
static uint8_t physical_step_in_range(const Curr3* physical);
static uint8_t current_in_hard_range(int16_t value);
static void reconstruct(uint8_t pair, const Curr3* sample, Curr3* physical);
static void apply_phys(const Curr3* physical);
static void map_logic(const Curr3* physical);
static int16_t map_current_sign(int16_t value);
static void quality_clear(void);
static void quality_update(const Curr3* physical);
static void quality_update_phase(int16_t value, volatile uint8_t* valid,
                                 volatile int16_t* previous,
                                 volatile uint32_t* spike_count,
                                 volatile uint16_t* max_step);
static uint8_t pair_samples_phys_v(uint8_t pair);
static uint8_t pair_samples_phys_w(uint8_t pair);
static void filter_clear(void);
static void filter_update(const Curr3* physical, uint8_t valid_mask);
static int16_t filter_value(int32_t value);
static uint16_t clamp_tick(uint16_t tick);
static uint16_t clamp_sample_tick(uint16_t tick);
static uint16_t abs_diff_i16(int16_t a, int16_t b);
static int16_t clamp_spread(int32_t value);
static uint16_t cnt_to_adc(int16_t cnt, uint16_t offset);

void curr_init(void)
{
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

void curr_set_vf_voltage(int16_t vf_voltage)
{
    uint16_t abs_voltage;

    if (vf_voltage < 0)
    {
        abs_voltage = (vf_voltage == INT16_MIN) ? 32768U : (uint16_t)(-vf_voltage);
    }
    else
    {
        abs_voltage = (uint16_t)vf_voltage;
    }

    if (s_vf_voltage_abs == abs_voltage)
    {
        return;
    }

    s_vf_voltage_abs = abs_voltage;
    if (s_sync.started != 0U)
    {
        NVIC_DisableIRQ(ADC_IRQn);
        trigger_update();
        irq_clear();
        NVIC_EnableIRQ(ADC_IRQn);
    }
}

uint8_t curr_irq(void)
{
    if (ADC_GetChannelIntFlag(s_sync.last) == 0U)
    {
        return 0U;
    }

    ADC_ClearChannelIntFlag(s_sync.last);

#if (CS_MULTI_EN != 0U)
    if (s_win.single_point != 0U)
    {
        sample_pair(&s_sample.a);
        s_sample.b = s_sample.a;
        s_diag.stage = 0U;
        s_diag.count = 1U;
        sample_resolve();
    }
    else
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
#else
    sample_pair(&s_sample.a);
    s_sample.b = s_sample.a;
    s_diag.count = 1U;
    sample_resolve();
#endif

    s_sync.count++;
    return 1U;
}

int16_t curr_u(void) { return s_logic.u; }
int16_t curr_v(void) { return s_logic.v; }
int16_t curr_w(void) { return s_logic.w; }
int16_t curr_sum(void) { return s_logic.sum; }
int16_t curr_raw_u(void) { return s_phys.u; }
int16_t curr_raw_v(void) { return s_phys.v; }
int16_t curr_raw_w(void) { return s_phys.w; }
int16_t curr_raw_sum(void) { return s_phys.sum; }
uint16_t curr_raw_adc_u(void) { return s_raw.u; }
uint16_t curr_raw_adc_v(void) { return s_raw.v; }
uint16_t curr_raw_adc_w(void) { return s_raw.w; }
uint32_t curr_sync_count(void) { return s_sync.count; }
uint8_t curr_multi_enabled(void) { return (uint8_t)(CS_MULTI_EN != 0U); }
uint8_t curr_dynamic_enabled(void) { return 1U; }
uint8_t curr_pair(void) { return s_win.pair; }
uint8_t curr_is_hold(void) { return (uint8_t)((s_win.hold != 0U) || (s_win.blank_count != 0U)); }
uint16_t curr_hold_count(void) { return s_win.hold_count; }
uint16_t curr_center_tick(void) { return s_win.center; }
uint16_t curr_window_u(void) { return s_win.u; }
uint16_t curr_window_v(void) { return s_win.v; }
uint16_t curr_window_w(void) { return s_win.w; }
uint16_t curr_window_common(void) { return s_win.common; }
uint32_t curr_sample_switch_count(void) { return s_win.switch_count; }
int16_t curr_sample_center_bias(void) { return s_win.center_bias; }
uint16_t curr_sample_tick_a(void) { return s_win.tick_a; }
uint16_t curr_sample_tick_b(void) { return s_win.tick_b; }
uint8_t curr_sample_region(void) { return s_win.region; }
uint8_t curr_sample_recon_mode(void) { return s_win.recon_mode; }
uint8_t curr_sample_valid_mask(void) { return s_win.valid_mask; }
uint8_t curr_sample_bad_count(void) { return s_win.bad_count; }
uint8_t curr_sample_fault(void) { return s_win.fault; }
uint16_t curr_sample_t1(void) { return s_win.t1; }
uint16_t curr_sample_t2(void) { return s_win.t2; }
uint16_t curr_sample_t3(void) { return s_win.t3; }
uint32_t curr_iv_spike_count(void) { return s_quality.iv_spike_count; }
uint32_t curr_iw_spike_count(void) { return s_quality.iw_spike_count; }
uint16_t curr_iv_max_step(void) { return s_quality.iv_max_step; }
uint16_t curr_iw_max_step(void) { return s_quality.iw_max_step; }
uint8_t curr_diag_stage(void) { return s_diag.stage; }
uint8_t curr_diag_count(void) { return s_diag.count; }
int16_t curr_diag_a0(void) { return s_diag.a0; }
int16_t curr_diag_a1(void) { return s_diag.a1; }
int16_t curr_diag_b0(void) { return s_diag.b0; }
int16_t curr_diag_b1(void) { return s_diag.b1; }
int16_t curr_diag_spread0(void) { return s_diag.spread0; }
int16_t curr_diag_spread1(void) { return s_diag.spread1; }
uint8_t curr_sample_single_point(void) { return s_win.single_point; }

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
    return adc_result12(ch);
}

static uint16_t adc_result12(uint32_t ch)
{
    return (uint16_t)(ADC_GetResult(ch) & 0x0FFFU);
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
    s_win.pair = CURR_PAIR_NONE;
    s_win.hold = 0U;
    s_win.blank_count = 0U;
    s_win.bad_count = 0U;
    s_win.fault = 0U;
    s_win.pair_hold_count = 0U;
    s_win.hold_count = 0U;
    s_win.center = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.tick_a = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.tick_b = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.center_bias = 0;
    s_win.single_point = 0U;
    s_win.common = 0U;
    s_win.u = 0U;
    s_win.v = 0U;
    s_win.w = 0U;
    s_win.switch_count = 0U;

    s_diag.stage = 0U;
    s_diag.count = 0U;
    s_diag.a0 = 0;
    s_diag.a1 = 0;
    s_diag.b0 = 0;
    s_diag.b1 = 0;
    s_diag.spread0 = 0;
    s_diag.spread1 = 0;

    s_sample.a = (Curr3){0, 0, 0};
    s_sample.b = (Curr3){0, 0, 0};
    quality_clear();
    filter_clear();
}

static void trigger_update(void)
{
    window_select();
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
#if (CS_MULTI_EN != 0U)
    if (s_win.single_point == 0U)
    {
        ADC_EnableEPWMCmp1TriggerChannel(cfg.channel_mask);
        ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP1);
    }
#endif

    s_sync.last = cfg.last_channel;
    s_sync.last_mask = cfg.last_channel_mask;
}

static uint8_t high_vf_single_active(void)
{
#if ((CS_MULTI_EN != 0U) && (CS_HIGH_VF_SINGLE_EN != 0U))
    return (uint8_t)(s_vf_voltage_abs >= (uint16_t)CS_HIGH_VF_SINGLE_VOLTAGE);
#else
    return 0U;
#endif
}

static uint8_t pair_cfg(uint8_t pair, CurrPairCfg* cfg)
{
    if (cfg == 0)
    {
        return 0U;
    }

    switch (pair)
    {
        case CURR_PAIR_ALL:
            cfg->channel_mask = ADC_CH_0_MSK | ADC_CH_2_MSK | ADC_CH_3_MSK;
            cfg->last_channel = ADC_CH_3;
            cfg->last_channel_mask = ADC_CH_3_MSK;
            return 1U;

        case CS_PAIR_UV:
            cfg->channel_mask = ADC_CH_0_MSK | ADC_CH_2_MSK;
            cfg->last_channel = ADC_CH_2;
            cfg->last_channel_mask = ADC_CH_2_MSK;
            return 1U;

        case CS_PAIR_UW:
            cfg->channel_mask = ADC_CH_0_MSK | ADC_CH_3_MSK;
            cfg->last_channel = ADC_CH_3;
            cfg->last_channel_mask = ADC_CH_3_MSK;
            return 1U;

        case CS_PAIR_VW:
        default:
            cfg->channel_mask = ADC_CH_2_MSK | ADC_CH_3_MSK;
            cfg->last_channel = ADC_CH_3;
            cfg->last_channel_mask = ADC_CH_3_MSK;
            return 1U;
    }
}

static void window_select(void)
{
    uint8_t pair;
    uint8_t valid_mask;
    uint8_t recon_mode;
    uint8_t region;
    uint16_t common_width;
    uint16_t center;

    if (ti_window_select(&pair, &center, &common_width, &valid_mask, &recon_mode, &region) == 0U)
    {
        window_hold();
        return;
    }

    s_win.valid_mask = valid_mask;
    s_win.recon_mode = recon_mode;
    s_win.region = region;
    window_apply_pair(pair, common_width, (int16_t)center);
}

static uint8_t ti_window_select(uint8_t* pair, uint16_t* center, uint16_t* width,
                                uint8_t* valid_mask, uint8_t* recon_mode,
                                uint8_t* region)
{
    volatile uint16_t duty_u;
    volatile uint16_t duty_v;
    volatile uint16_t duty_w;
    volatile uint8_t output_on;
    volatile uint8_t brake_on;
    CurrDutyItem maxp;
    CurrDutyItem midp;
    CurrDutyItem minp;
    uint16_t reserve;
    uint16_t min_width;
    uint16_t case1_width;
    uint16_t open_delay;

    pwm_snapshot(&duty_u, &duty_v, &duty_w, &output_on, &brake_on);
    (void)output_on;
    (void)brake_on;

    maxp = (CurrDutyItem){CURR_PHASE_U, clamp_tick((uint16_t)duty_u)};
    midp = (CurrDutyItem){CURR_PHASE_V, clamp_tick((uint16_t)duty_v)};
    minp = (CurrDutyItem){CURR_PHASE_W, clamp_tick((uint16_t)duty_w)};
    sort_duty_desc(&maxp, &midp, &minp);

    s_win.u = (uint16_t)duty_u;
    s_win.v = (uint16_t)duty_v;
    s_win.w = (uint16_t)duty_w;
    s_win.t1 = minp.duty;
    s_win.t2 = (uint16_t)(midp.duty - minp.duty);
    s_win.t3 = (uint16_t)(maxp.duty - midp.duty);

    reserve = ((CS_MULTI_EN != 0U) && (high_vf_single_active() == 0U)) ?
              (uint16_t)CS_MULTI_DELTA_TICK : 0U;
    open_delay = (uint16_t)(CS_OPEN_SETTLE_TICK + reserve);
    min_width = (uint16_t)(open_delay + (uint16_t)CS_TAIL_MARGIN_TICK + reserve);
    case1_width = (uint16_t)(min_width + (uint16_t)CS_REGION1_ENTER_MARGIN_TICK);

    if (s_win.t1 >= case1_width)
    {
        *pair = CURR_PAIR_ALL;
        *center = (uint16_t)(minp.duty - open_delay);
        *width = s_win.t1;
        *valid_mask = (uint8_t)(CURR_VALID_U | CURR_VALID_V | CURR_VALID_W);
        *recon_mode = CURR_RECON_ALL;
        *region = 1U;
        return 1U;
    }

    if (s_win.t2 >= min_width)
    {
        *pair = pair_from_phases(maxp.phase, midp.phase);
        if (*pair == CURR_PAIR_NONE)
        {
            return 0U;
        }
        *center = (uint16_t)(midp.duty - open_delay);
        *width = s_win.t2;
        *valid_mask = (uint8_t)(valid_bit(maxp.phase) | valid_bit(midp.phase));
        *recon_mode = CURR_RECON_PAIR;
        *region = 2U;
        return 1U;
    }

    if (s_win.t3 >= min_width)
    {
        *pair = pair_from_phases(maxp.phase, midp.phase);
        if (*pair == CURR_PAIR_NONE)
        {
            return 0U;
        }
        *center = (uint16_t)(maxp.duty - open_delay);
        *width = s_win.t3;
        *valid_mask = valid_bit(maxp.phase);
        *recon_mode = CURR_RECON_FILTER;
        *region = 3U;
        return 1U;
    }

    return 0U;
}

static void sort_duty_desc(CurrDutyItem* a, CurrDutyItem* b, CurrDutyItem* c)
{
    CurrDutyItem t;

    if (a->duty < b->duty)
    {
        t = *a;
        *a = *b;
        *b = t;
    }
    if (b->duty < c->duty)
    {
        t = *b;
        *b = *c;
        *c = t;
    }
    if (a->duty < b->duty)
    {
        t = *a;
        *a = *b;
        *b = t;
    }
}

static uint8_t pair_from_phases(uint8_t a, uint8_t b)
{
    if (((a == CURR_PHASE_U) && (b == CURR_PHASE_V)) ||
        ((a == CURR_PHASE_V) && (b == CURR_PHASE_U)))
    {
        return CS_PAIR_UV;
    }
    if (((a == CURR_PHASE_U) && (b == CURR_PHASE_W)) ||
        ((a == CURR_PHASE_W) && (b == CURR_PHASE_U)))
    {
        return CS_PAIR_UW;
    }
    if (((a == CURR_PHASE_V) && (b == CURR_PHASE_W)) ||
        ((a == CURR_PHASE_W) && (b == CURR_PHASE_V)))
    {
        return CS_PAIR_VW;
    }
    return CURR_PAIR_NONE;
}

static uint8_t valid_bit(uint8_t phase)
{
    switch (phase)
    {
        case CURR_PHASE_U:
            return CURR_VALID_U;

        case CURR_PHASE_V:
            return CURR_VALID_V;

        case CURR_PHASE_W:
            return CURR_VALID_W;

        default:
            return 0U;
    }
}

static void window_apply_pair(uint8_t pair, uint16_t common_width, int16_t center_bias)
{
    uint16_t center;
    int16_t actual_bias;
    uint16_t delta;
    uint16_t tick_a;
    uint16_t tick_b;
    uint8_t single_point;
    uint8_t reconfigure;

    single_point = high_vf_single_active();
    delta = (uint16_t)(common_width / 4U);
    if (delta > CS_MULTI_DELTA_TICK)
    {
        delta = CS_MULTI_DELTA_TICK;
    }
    if (delta == 0U)
    {
        delta = 1U;
    }
    center_bias = (int16_t)(center_bias + pair_bias_tick(pair));
#if (CS_FIXED_SAMPLE_TICK_EN != 0U)
    (void)center_bias;
    center = clamp_sample_tick((uint16_t)CS_FIXED_SAMPLE_TICK);
#else
    (void)pair;
    center = clamp_sample_tick((uint16_t)center_bias);
#endif
    actual_bias = (int16_t)((int32_t)center - (int32_t)(common_width / 2U));

    if (single_point != 0U)
    {
        tick_a = center;
        tick_b = center;
    }
    else
    {
        tick_a = clamp_tick((uint16_t)(center + delta));
        tick_b = (center > delta) ? (uint16_t)(center - delta) : 0U;
    }
    reconfigure = (uint8_t)((s_win.pair != pair) || (s_win.center != center) ||
                            (s_win.tick_a != tick_a) || (s_win.tick_b != tick_b) ||
                            (s_win.center_bias != actual_bias) ||
                            (s_win.single_point != single_point) ||
                            (s_win.hold != 0U));

    if ((s_win.pair != pair) && (s_win.pair != CURR_PAIR_NONE))
    {
        if (s_win.switch_count < 0xFFFFFFFFUL)
        {
            s_win.switch_count++;
        }
        s_win.pair_hold_count = 0U;
        s_win.blank_count = CS_PAIR_SWITCH_BLANK_PWM;
    }

    s_win.hold = 0U;
    s_win.pair = pair;
    s_win.common = common_width;
    s_win.center = center;
    s_win.tick_a = tick_a;
    s_win.tick_b = tick_b;
    s_win.center_bias = actual_bias;
    s_win.single_point = single_point;

    if (reconfigure == 0U)
    {
        return;
    }

    if (single_point != 0U)
    {
        pwm_set_adc_trigger(center);
    }
    else
    {
#if (CS_MULTI_EN != 0U)
        pwm_set_adc_triggers(tick_a, tick_b);
#else
        pwm_set_adc_trigger(center);
#endif
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
    if (s_win.pair == CURR_PAIR_NONE)
    {
        s_win.pair = CS_PAIR_VW;
        s_win.center = PWM_ADC_TRIGGER_TICK_DEFAULT;
        s_win.tick_a = PWM_ADC_TRIGGER_TICK_DEFAULT;
        s_win.tick_b = PWM_ADC_TRIGGER_TICK_DEFAULT;
        s_win.center_bias = 0;
        s_win.single_point = 0U;
        pwm_set_adc_trigger(PWM_ADC_TRIGGER_TICK_DEFAULT);
        trigger_pair(s_win.pair);
    }
}

static void reject_bad_sample(void)
{
    if (s_win.hold_count < 65535U)
    {
        s_win.hold_count++;
    }
    if (s_win.bad_count < 255U)
    {
        s_win.bad_count++;
    }
    if (s_win.bad_count >= (uint8_t)CS_BAD_SAMPLE_SHUTDOWN_COUNT)
    {
        s_win.fault = 1U;
        s_win.hold = 1U;
    }
}

static int16_t pair_bias_tick(uint8_t pair)
{
    switch (pair)
    {
        case CS_PAIR_UV:
            return (int16_t)CS_PAIR_BIAS_UV_TICK;

        case CS_PAIR_UW:
            return (int16_t)CS_PAIR_BIAS_UW_TICK;

        case CS_PAIR_VW:
        default:
            return (int16_t)CS_PAIR_BIAS_VW_TICK;
    }
}

static void sample_pair(Curr3* sample)
{
    int16_t first = 0;
    int16_t second = 0;

    sample->u = 0;
    sample->v = 0;
    sample->w = 0;

    switch (s_win.pair)
    {
        case CURR_PAIR_ALL:
            sample->u = (int16_t)((int16_t)adc_result12(ADC_CH_0) - (int16_t)s_zero.u);
            sample->v = (int16_t)((int16_t)adc_result12(ADC_CH_2) - (int16_t)s_zero.v);
            sample->w = (int16_t)((int16_t)adc_result12(ADC_CH_3) - (int16_t)s_zero.w);
            first = sample->u;
            second = sample->v;
            break;

        case CS_PAIR_UV:
            sample->u = (int16_t)((int16_t)adc_result12(ADC_CH_0) - (int16_t)s_zero.u);
            sample->v = (int16_t)((int16_t)adc_result12(ADC_CH_2) - (int16_t)s_zero.v);
            first = sample->u;
            second = sample->v;
            break;

        case CS_PAIR_UW:
            sample->u = (int16_t)((int16_t)adc_result12(ADC_CH_0) - (int16_t)s_zero.u);
            sample->w = (int16_t)((int16_t)adc_result12(ADC_CH_3) - (int16_t)s_zero.w);
            first = sample->u;
            second = sample->w;
            break;

        case CS_PAIR_VW:
        default:
            sample->v = (int16_t)((int16_t)adc_result12(ADC_CH_2) - (int16_t)s_zero.v);
            sample->w = (int16_t)((int16_t)adc_result12(ADC_CH_3) - (int16_t)s_zero.w);
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
    uint8_t use_avg = 1U;

    if (s_win.hold != 0U)
    {
        return;
    }

    if (s_win.blank_count != 0U)
    {
        s_win.blank_count--;
        if (s_win.hold_count < 65535U)
        {
            s_win.hold_count++;
        }
        return;
    }

    s_diag.spread0 = clamp_spread((int32_t)s_diag.b0 - (int32_t)s_diag.a0);
    s_diag.spread1 = clamp_spread((int32_t)s_diag.b1 - (int32_t)s_diag.a1);

#if (CS_MULTI_EN != 0U)
    if (s_win.single_point != 0U)
    {
        s_diag.spread0 = 0;
        s_diag.spread1 = 0;
    }
    else if ((abs_diff_i16(s_diag.b0, s_diag.a0) > (uint16_t)CS_MULTI_SPREAD_LIMIT_CNT) ||
             (abs_diff_i16(s_diag.b1, s_diag.a1) > (uint16_t)CS_MULTI_SPREAD_LIMIT_CNT))
    {
#if (CS_MULTI_SPREAD_REJECT_EN != 0U)
        reject_bad_sample();
        return;
#else
        use_avg = 0U;
#endif
    }
#endif

    if (use_avg != 0U)
    {
        avg.u = (int16_t)(((int32_t)s_sample.a.u + (int32_t)s_sample.b.u) / 2);
        avg.v = (int16_t)(((int32_t)s_sample.a.v + (int32_t)s_sample.b.v) / 2);
        avg.w = (int16_t)(((int32_t)s_sample.a.w + (int32_t)s_sample.b.w) / 2);
    }
    else
    {
        avg = s_sample.a;
    }

    if (pair_sample_in_range(s_win.pair, &avg) == 0U)
    {
        reject_bad_sample();
        return;
    }

    reconstruct(s_win.pair, &avg, &physical);
    if (physical_in_hard_range(&physical) == 0U)
    {
        reject_bad_sample();
        return;
    }

    if (physical_step_in_range(&physical) == 0U)
    {
        reject_bad_sample();
        return;
    }

    s_win.bad_count = 0U;
    filter_update(&physical, s_win.valid_mask);
    apply_phys(&physical);
}

static uint8_t pair_sample_in_range(uint8_t pair, const Curr3* sample)
{
    if (s_win.recon_mode == CURR_RECON_FILTER)
    {
        uint8_t ok = 1U;
        if ((s_win.valid_mask & CURR_VALID_U) != 0U)
        {
            ok = (uint8_t)(ok && current_in_hard_range(sample->u));
        }
        if ((s_win.valid_mask & CURR_VALID_V) != 0U)
        {
            ok = (uint8_t)(ok && current_in_hard_range(sample->v));
        }
        if ((s_win.valid_mask & CURR_VALID_W) != 0U)
        {
            ok = (uint8_t)(ok && current_in_hard_range(sample->w));
        }
        return ok;
    }

    switch (pair)
    {
        case CURR_PAIR_ALL:
            return (uint8_t)(current_in_hard_range(sample->u) &&
                             current_in_hard_range(sample->v) &&
                             current_in_hard_range(sample->w));

        case CS_PAIR_UV:
            return (uint8_t)(current_in_hard_range(sample->u) &&
                             current_in_hard_range(sample->v));

        case CS_PAIR_UW:
            return (uint8_t)(current_in_hard_range(sample->u) &&
                             current_in_hard_range(sample->w));

        case CS_PAIR_VW:
        default:
            return (uint8_t)(current_in_hard_range(sample->v) &&
                             current_in_hard_range(sample->w));
    }
}

static uint8_t physical_in_hard_range(const Curr3* physical)
{
    return (uint8_t)(current_in_hard_range(physical->u) &&
                     current_in_hard_range(physical->v) &&
                     current_in_hard_range(physical->w));
}

static uint8_t physical_step_in_range(const Curr3* physical)
{
#if (CS_SPIKE_REJECT_EN != 0U)
    if (s_sync.count < 4U)
    {
        return 1U;
    }
    return (uint8_t)((abs_diff_i16(physical->u, s_phys.u) <= (uint16_t)CS_SPIKE_LIMIT_CNT) &&
                     (abs_diff_i16(physical->v, s_phys.v) <= (uint16_t)CS_SPIKE_LIMIT_CNT) &&
                     (abs_diff_i16(physical->w, s_phys.w) <= (uint16_t)CS_SPIKE_LIMIT_CNT));
#else
    (void)physical;
    return 1U;
#endif
}

static uint8_t current_in_hard_range(int16_t value)
{
    return (uint8_t)((value > (int16_t)-CS_SAMPLE_ABS_HARD_LIMIT_CNT) &&
                     (value < (int16_t)CS_SAMPLE_ABS_HARD_LIMIT_CNT));
}

static void reconstruct(uint8_t pair, const Curr3* sample, Curr3* physical)
{
    physical->u = sample->u;
    physical->v = sample->v;
    physical->w = sample->w;

    if (s_win.recon_mode == CURR_RECON_ALL)
    {
        return;
    }

    if (s_win.recon_mode == CURR_RECON_FILTER)
    {
        if ((s_win.valid_mask & CURR_VALID_U) == 0U)
        {
            physical->u = filter_value(s_filter.u);
        }
        if ((s_win.valid_mask & CURR_VALID_V) == 0U)
        {
            physical->v = filter_value(s_filter.v);
        }
        if ((s_win.valid_mask & CURR_VALID_W) == 0U)
        {
            physical->w = filter_value(s_filter.w);
        }
        return;
    }

    switch (pair)
    {
        case CS_PAIR_UV:
            physical->w = (int16_t)(-physical->u - physical->v);
            break;

        case CS_PAIR_UW:
            physical->v = (int16_t)(-physical->u - physical->w);
            break;

        case CS_PAIR_VW:
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
    s_phys.sum = (int16_t)((int32_t)physical->u + (int32_t)physical->v + (int32_t)physical->w);

    s_raw.u = cnt_to_adc(physical->u, s_zero.u);
    s_raw.v = cnt_to_adc(physical->v, s_zero.v);
    s_raw.w = cnt_to_adc(physical->w, s_zero.w);

    map_logic(physical);
    quality_update(physical);
}

static void map_logic(const Curr3* physical)
{
#if (MOT_CURR_PHASE_MAP == MOT_PHASE_MAP_UWV)
    s_logic.u = map_current_sign(physical->u);
    s_logic.v = map_current_sign(physical->w);
    s_logic.w = map_current_sign(physical->v);
#elif (MOT_CURR_PHASE_MAP == MOT_PHASE_MAP_VUW)
    s_logic.u = map_current_sign(physical->v);
    s_logic.v = map_current_sign(physical->u);
    s_logic.w = map_current_sign(physical->w);
#elif (MOT_CURR_PHASE_MAP == MOT_PHASE_MAP_VWU)
    s_logic.u = map_current_sign(physical->v);
    s_logic.v = map_current_sign(physical->w);
    s_logic.w = map_current_sign(physical->u);
#elif (MOT_CURR_PHASE_MAP == MOT_PHASE_MAP_WUV)
    s_logic.u = map_current_sign(physical->w);
    s_logic.v = map_current_sign(physical->u);
    s_logic.w = map_current_sign(physical->v);
#elif (MOT_CURR_PHASE_MAP == MOT_PHASE_MAP_WVU)
    s_logic.u = map_current_sign(physical->w);
    s_logic.v = map_current_sign(physical->v);
    s_logic.w = map_current_sign(physical->u);
#else
    s_logic.u = map_current_sign(physical->u);
    s_logic.v = map_current_sign(physical->v);
    s_logic.w = map_current_sign(physical->w);
#endif
    s_logic.sum = (int16_t)((int32_t)s_logic.u + (int32_t)s_logic.v + (int32_t)s_logic.w);
}

static int16_t map_current_sign(int16_t value)
{
#if (MOT_CURR_SIGN < 0)
    return (int16_t)-value;
#else
    return value;
#endif
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

static void filter_clear(void)
{
    s_filter.u = 0;
    s_filter.v = 0;
    s_filter.w = 0;
    s_filter.seeded = 0U;
}

static void filter_update(const Curr3* physical, uint8_t valid_mask)
{
    if (s_filter.seeded == 0U)
    {
        s_filter.u = (int32_t)physical->u;
        s_filter.v = (int32_t)physical->v;
        s_filter.w = (int32_t)physical->w;
        s_filter.seeded = 1U;
        return;
    }

    if ((valid_mask & CURR_VALID_U) != 0U)
    {
        s_filter.u += ((int32_t)physical->u - s_filter.u) >> CS_FILTER_SHIFT;
    }
    if ((valid_mask & CURR_VALID_V) != 0U)
    {
        s_filter.v += ((int32_t)physical->v - s_filter.v) >> CS_FILTER_SHIFT;
    }
    if ((valid_mask & CURR_VALID_W) != 0U)
    {
        s_filter.w += ((int32_t)physical->w - s_filter.w) >> CS_FILTER_SHIFT;
    }
}

static int16_t filter_value(int32_t value)
{
    if (s_filter.seeded == 0U)
    {
        return 0;
    }
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32767)
    {
        return -32767;
    }
    return (int16_t)value;
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
                                 volatile int16_t* previous,
                                 volatile uint32_t* spike_count,
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
    if (step > (uint16_t)CS_SPIKE_LIMIT_CNT)
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
    return (uint8_t)(((s_win.valid_mask & CURR_VALID_V) != 0U) ||
                     (pair == CURR_PAIR_ALL) || (pair == CS_PAIR_UV) || (pair == CS_PAIR_VW));
}

static uint8_t pair_samples_phys_w(uint8_t pair)
{
    return (uint8_t)(((s_win.valid_mask & CURR_VALID_W) != 0U) ||
                     (pair == CURR_PAIR_ALL) || (pair == CS_PAIR_UW) || (pair == CS_PAIR_VW));
}

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
    if (value < -32767)
    {
        return -32767;
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
