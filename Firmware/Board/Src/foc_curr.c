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

/** @brief 三相电流或 ADC 扣零后的物理相电流，单位 ADC count。 */
typedef struct
{
    int16_t u;
    int16_t v;
    int16_t w;
} Curr3;

/** @brief 当前采样 pair 对应的 ADC 通道和最后完成通道。 */
typedef struct
{
    uint32_t channel_mask;
    uint32_t last_channel;
    uint32_t last_channel_mask;
} CurrPairCfg;

/** @brief 未扣零漂的三相 ADC 原始码值。 */
typedef struct
{
    volatile uint16_t u;
    volatile uint16_t v;
    volatile uint16_t w;
} CurrRawAdc;

/** @brief 三相静态零漂。 */
typedef struct
{
    uint16_t u;
    uint16_t v;
    uint16_t w;
} CurrZero;

/** @brief PWM/ADC 同步采样状态。 */
typedef struct
{
    volatile int16_t u;
    volatile int16_t v;
    volatile int16_t w;
    volatile int16_t sum;
} CurrCache;

/** @brief 当前 PWM 周期选出的采样窗口和触发点。 */
typedef struct
{
    volatile uint32_t count;
    volatile uint8_t started;
    uint32_t last;
    uint32_t last_mask;
} CurrSync;

/** @brief 双点采样两次触发之间的差值诊断缓存。 */
typedef struct
{
    volatile uint8_t pair;
    volatile uint8_t hold;
    volatile uint8_t blank_count;
    volatile uint8_t valid_mask;
    volatile uint8_t recon_mode;
    volatile uint16_t center;
    volatile uint16_t tick_a;
    volatile uint16_t tick_b;
    volatile uint8_t single_point;
} CurrWindow;

/** @brief 一个 PWM 周期内 A/B 两次采样。 */
typedef struct
{
    volatile uint8_t stage;
    volatile int16_t a0;
    volatile int16_t a1;
    volatile int16_t b0;
    volatile int16_t b1;
} CurrSampleDelta;

/** @brief 无法重构全部三相时用于保持缺失相的低通缓存。 */
typedef struct
{
    Curr3 a;
    Curr3 b;
} CurrPairSample;

/** @brief duty 排序项，用于按 T1/T2/T3 选择采样窗口。 */
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
static CurrSampleDelta s_delta;
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
static void state_clear(void);
static void trigger_update(void);
static void trigger_pair(uint8_t pair);
static uint8_t pair_cfg(uint8_t pair, CurrPairCfg* cfg);
static uint8_t high_vf_single_active(void);
static void window_select(void);
static uint8_t ti_window_select(uint8_t* pair, uint16_t* center, uint16_t* width,
                                uint8_t* valid_mask, uint8_t* recon_mode);
static void sort_duty_desc(CurrDutyItem* a, CurrDutyItem* b, CurrDutyItem* c);
static uint8_t pair_from_phases(uint8_t a, uint8_t b);
static uint8_t valid_bit(uint8_t phase);
static void window_apply_pair(uint8_t pair, uint16_t common_width, int16_t center_bias);
static void window_hold(void);
static void sample_pair(Curr3* sample);
static void sample_resolve(void);
static uint8_t pair_sample_in_range(uint8_t pair, const Curr3* sample);
static uint8_t physical_in_hard_range(const Curr3* physical);
static uint8_t current_in_hard_range(int16_t value);
static void reconstruct(uint8_t pair, const Curr3* sample, Curr3* physical);
static void apply_phys(const Curr3* physical);
static void map_logic(const Curr3* physical);
static int16_t map_current_sign(int16_t value);
static void filter_clear(void);
static void filter_update(const Curr3* physical, uint8_t valid_mask);
static int16_t filter_value(int32_t value);
static uint16_t clamp_tick(uint16_t tick);
static uint16_t clamp_sample_tick(uint16_t tick);
static uint16_t abs_diff_i16(int16_t a, int16_t b);
static uint16_t cnt_to_adc(int16_t cnt, uint16_t offset);

/** @brief 初始化电流采样硬件和内部缓存。 */
void curr_init(void)
{
    pins_init();
    ldo_init();
    pga_init();
    adc_init();
    state_clear();
}

/** @brief 软件轮询读取一次三相原始 ADC。 */
void curr_sample_raw(void)
{
    s_raw.u = read_one(ADC_CH_0, ADC_CH_0_MSK);
    s_raw.v = read_one(ADC_CH_2, ADC_CH_2_MSK);
    s_raw.w = read_one(ADC_CH_3, ADC_CH_3_MSK);
}

/** @brief PWM 关闭时采集三相零漂平均值。 */
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

/** @brief 临时关闭 PWM 触发采样，用软件采样重新估计零漂。 */
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

/** @brief 将当前 raw ADC 扣零漂并刷新物理/逻辑电流缓存。 */
void curr_update(void)
{
    Curr3 physical;

    physical.u = (int16_t)((int16_t)s_raw.u - (int16_t)s_zero.u);
    physical.v = (int16_t)((int16_t)s_raw.v - (int16_t)s_zero.v);
    physical.w = (int16_t)((int16_t)s_raw.w - (int16_t)s_zero.w);
    apply_phys(&physical);
}

/** @brief 启动 PWM 触发 ADC 的连续同步采样。 */
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

/** @brief PWM duty 更新后重新选择下一拍采样窗口。 */
void curr_sync_timing(void)
{
    if (s_sync.started == 0U)
    {
        return;
    }

    trigger_update();
}

/** @brief 更新 VF 电压幅值，用于高调制区单点采样策略切换。 */
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

/** @brief ADC IRQ 中解析单点或双点采样，成功后更新三相电流。 */
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
        s_delta.stage = 0U;
        sample_resolve();
    }
    else
    {
        if (s_delta.stage == 0U)
        {
            sample_pair(&s_sample.a);
            s_delta.stage = 1U;
            return 0U;
        }

        sample_pair(&s_sample.b);
        s_delta.stage = 0U;
        sample_resolve();
    }
#else
    sample_pair(&s_sample.a);
    s_sample.b = s_sample.a;
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

/** @brief 配置三相电流 ADC/PGA 相关引脚为模拟输入。 */
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

/** @brief 启动 ADC LDO 并选择输出电压。 */
static void ldo_init(void)
{
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCLDO, ENABLE);
    ADCLDO_OutVlotageSel(ADCLDO_OutV_3d6);
    ADCLDO_Enable();
}

/** @brief 配置三路差分 PGA、半电源参考和增益。 */
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

/** @brief 初始化 ADC 基本运行模式，默认先用于软件单次采样。 */
static void adc_init(void)
{
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCEN, ENABLE);
    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_SINGLE, ADC_CLK_DIV_1, 25);
    ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);
    ADC_DisableChargeAndDischarge();
    ADC_ConfigVREF(ADC_VREFP_AVREFP);
    ADC_Start();
}

/** @brief 软件触发读取单个 ADC 通道。 */
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

/** @brief 读取 ADC 结果并截取 12-bit 有效码值。 */
static uint16_t adc_result12(uint32_t ch)
{
    return (uint16_t)(ADC_GetResult(ch) & 0x0FFFU);
}

/** @brief 清除当前使用到的 ADC 通道中断和 NVIC pending。 */
static void irq_clear(void)
{
    ADC_ClearChannelIntFlag(ADC_CH_0);
    ADC_ClearChannelIntFlag(ADC_CH_2);
    ADC_ClearChannelIntFlag(ADC_CH_3);
    NVIC_ClearPendingIRQ(ADC_IRQn);
}

/** @brief 重置同步采样状态，保留硬件配置由后续 trigger_update 设置。 */
static void sync_reset(void)
{
    s_sync.count = 0;
    s_sync.started = 0U;
    state_clear();
}

/** @brief 清空采样窗口、双点差值和滤波缓存。 */
static void state_clear(void)
{
    s_win.pair = CURR_PAIR_NONE;
    s_win.hold = 0U;
    s_win.blank_count = 0U;
    s_win.center = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.tick_a = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.tick_b = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_win.single_point = 0U;

    s_delta.stage = 0U;
    s_delta.a0 = 0;
    s_delta.a1 = 0;
    s_delta.b0 = 0;
    s_delta.b1 = 0;

    s_sample.a = (Curr3){0, 0, 0};
    s_sample.b = (Curr3){0, 0, 0};
    filter_clear();
}

/** @brief 重新选择采样窗口并配置 PWM/ADC 触发。 */
static void trigger_update(void)
{
    window_select();
}

/** @brief 按当前 pair 配置 ADC 扫描通道和 EPWM CMP 触发源。 */
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

    s_delta.stage = 0U;

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

/** @brief 判断 VF 高电压诊断时是否切到单点采样。 */
static uint8_t high_vf_single_active(void)
{
#if ((CS_MULTI_EN != 0U) && (CS_HIGH_VF_SINGLE_EN != 0U))
    return (uint8_t)(s_vf_voltage_abs >= (uint16_t)CS_HIGH_VF_SINGLE_VOLTAGE);
#else
    return 0U;
#endif
}

/** @brief 将逻辑采样 pair 映射到 ADC 通道掩码。 */
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

/** @brief 根据当前 duty 选择有效采样窗口；无窗口时保持上一配置。 */
static void window_select(void)
{
    uint8_t pair;
    uint8_t valid_mask;
    uint8_t recon_mode;
    uint16_t common_width;
    uint16_t center;

    if (ti_window_select(&pair, &center, &common_width, &valid_mask, &recon_mode) == 0U)
    {
        window_hold();
        return;
    }

    s_win.valid_mask = valid_mask;
    s_win.recon_mode = recon_mode;
    window_apply_pair(pair, common_width, (int16_t)center);
}

/** @brief 按 T1/T2/T3 窗口宽度选择可采样 pair 和重构模式。 */
static uint8_t ti_window_select(uint8_t* pair, uint16_t* center, uint16_t* width,
                                uint8_t* valid_mask, uint8_t* recon_mode)
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
    uint16_t t1;
    uint16_t t2;
    uint16_t t3;

    pwm_snapshot(&duty_u, &duty_v, &duty_w, &output_on, &brake_on);
    (void)output_on;
    (void)brake_on;

    maxp = (CurrDutyItem){CURR_PHASE_U, clamp_tick((uint16_t)duty_u)};
    midp = (CurrDutyItem){CURR_PHASE_V, clamp_tick((uint16_t)duty_v)};
    minp = (CurrDutyItem){CURR_PHASE_W, clamp_tick((uint16_t)duty_w)};
    sort_duty_desc(&maxp, &midp, &minp);

    t1 = minp.duty;
    t2 = (uint16_t)(midp.duty - minp.duty);
    t3 = (uint16_t)(maxp.duty - midp.duty);

    reserve = ((CS_MULTI_EN != 0U) && (high_vf_single_active() == 0U)) ?
              (uint16_t)CS_MULTI_DELTA_TICK : 0U;
    open_delay = (uint16_t)(CS_OPEN_SETTLE_TICK + reserve);
    min_width = (uint16_t)(open_delay + (uint16_t)CS_TAIL_MARGIN_TICK + reserve);
    case1_width = min_width;

    if (t1 >= case1_width)
    {
#if (CS_USE_2PHASE_IN_ALL_WINDOW != 0U)
        *pair = CS_ALL_WINDOW_PAIR;
        if (*pair > CS_PAIR_VW)
        {
            return 0U;
        }
        *recon_mode = CURR_RECON_PAIR;
#else
        *pair = CURR_PAIR_ALL;
        *recon_mode = CURR_RECON_ALL;
#endif
        *center = (uint16_t)(minp.duty - open_delay);
        *width = t1;
        *valid_mask = (uint8_t)(CURR_VALID_U | CURR_VALID_V | CURR_VALID_W);
        return 1U;
    }

    if (t2 >= min_width)
    {
        *pair = pair_from_phases(maxp.phase, midp.phase);
        if (*pair == CURR_PAIR_NONE)
        {
            return 0U;
        }
        *center = (uint16_t)(midp.duty - open_delay);
        *width = t2;
        *valid_mask = (uint8_t)(valid_bit(maxp.phase) | valid_bit(midp.phase));
        *recon_mode = CURR_RECON_PAIR;
        return 1U;
    }

    if (t3 >= min_width)
    {
        *pair = pair_from_phases(maxp.phase, midp.phase);
        if (*pair == CURR_PAIR_NONE)
        {
            return 0U;
        }
        *center = (uint16_t)(maxp.duty - open_delay);
        *width = t3;
        *valid_mask = valid_bit(maxp.phase);
        *recon_mode = CURR_RECON_FILTER;
        return 1U;
    }

    return 0U;
}

/** @brief 将三相 duty 从大到小排序，用于计算 T1/T2/T3。 */
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

/** @brief 将两相物理相编号转换为采样 pair 编号。 */
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

/** @brief 返回某一相在 valid_mask 中的 bit。 */
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

/** @brief 应用采样 pair 和中心点，必要时重配 PWM/ADC 触发。 */
static void window_apply_pair(uint8_t pair, uint16_t common_width, int16_t center_bias)
{
    uint16_t center;
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
    center = clamp_sample_tick((uint16_t)center_bias);

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
                            (s_win.single_point != single_point) ||
                            (s_win.hold != 0U));

    if ((s_win.pair != pair) && (s_win.pair != CURR_PAIR_NONE))
    {
        s_win.blank_count = CS_PAIR_SWITCH_BLANK_PWM;
    }

    s_win.hold = 0U;
    s_win.pair = pair;
    s_win.center = center;
    s_win.tick_a = tick_a;
    s_win.tick_b = tick_b;
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

/** @brief 没有足够采样窗口时保持上一 pair，首次进入时配置保底 pair。 */
static void window_hold(void)
{
    s_win.hold = 1U;
    if (s_win.pair == CURR_PAIR_NONE)
    {
        s_win.pair = CS_PAIR_VW;
        s_win.center = PWM_ADC_TRIGGER_TICK_DEFAULT;
        s_win.tick_a = PWM_ADC_TRIGGER_TICK_DEFAULT;
        s_win.tick_b = PWM_ADC_TRIGGER_TICK_DEFAULT;
        s_win.single_point = 0U;
        pwm_set_adc_trigger(PWM_ADC_TRIGGER_TICK_DEFAULT);
        trigger_pair(s_win.pair);
    }
}

/** @brief 按当前 pair 从 ADC 结果中取样并扣除零漂。 */
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

    if (s_delta.stage == 0U)
    {
        s_delta.a0 = first;
        s_delta.a1 = second;
    }
    else
    {
        s_delta.b0 = first;
        s_delta.b1 = second;
    }
}

/** @brief 解析单点/双点样本，完成范围检查、重构、滤波和缓存更新。 */
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
        return;
    }

#if (CS_MULTI_EN != 0U)
    if ((s_win.single_point == 0U) &&
        ((abs_diff_i16(s_delta.b0, s_delta.a0) > (uint16_t)CS_MULTI_SPREAD_LIMIT_CNT) ||
         (abs_diff_i16(s_delta.b1, s_delta.a1) > (uint16_t)CS_MULTI_SPREAD_LIMIT_CNT)))
    {
        use_avg = 0U;
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
        return;
    }

    reconstruct(s_win.pair, &avg, &physical);
    if (physical_in_hard_range(&physical) == 0U)
    {
        return;
    }

    filter_update(&physical, s_win.valid_mask);
    apply_phys(&physical);
}

/** @brief 检查当前 pair 的已采样相是否在硬限范围内。 */
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

/** @brief 检查重构后的三相物理电流是否在硬限范围内。 */
static uint8_t physical_in_hard_range(const Curr3* physical)
{
    return (uint8_t)(current_in_hard_range(physical->u) &&
                     current_in_hard_range(physical->v) &&
                     current_in_hard_range(physical->w));
}

/** @brief 检查单相电流 count 是否在采样硬限内。 */
static uint8_t current_in_hard_range(int16_t value)
{
    return (uint8_t)((value > (int16_t)-CS_SAMPLE_ABS_HARD_LIMIT_CNT) &&
                     (value < (int16_t)CS_SAMPLE_ABS_HARD_LIMIT_CNT));
}

/** @brief 根据采样 pair 用 KCL 或滤波保持重构三相物理电流。 */
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

/** @brief 写入物理电流缓存，并同步更新 raw 观察值和逻辑相序。 */
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
}

/** @brief 按 TuneConfig 中相序/符号把物理相映射为控制逻辑 U/V/W。 */
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

/** @brief 按 MOT_CURR_SIGN 统一电流方向。 */
static int16_t map_current_sign(int16_t value)
{
#if (MOT_CURR_SIGN < 0)
    return (int16_t)-value;
#else
    return value;
#endif
}

/** @brief 清空缺失相保持滤波器。 */
static void filter_clear(void)
{
    s_filter.u = 0;
    s_filter.v = 0;
    s_filter.w = 0;
    s_filter.seeded = 0U;
}

/** @brief 用有效采样相更新缺失相保持滤波器。 */
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

/** @brief 将滤波器内部 int32 值夹紧为 int16 电流。 */
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

/** @brief 将 PWM tick 限制到周期范围内。 */
static uint16_t clamp_tick(uint16_t tick)
{
    if (tick > PWM_PERIOD)
    {
        return PWM_PERIOD;
    }
    return tick;
}

/** @brief 将采样 tick 限制到避开死区的安全范围。 */
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

/** @brief 计算两个 int16 的绝对差，饱和到 uint16。 */
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

/** @brief 将扣零漂后的 count 还原为 12-bit ADC 观察值。 */
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
