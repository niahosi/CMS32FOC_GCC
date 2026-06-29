#include "Board_Analog.h"
#include "Board_PWM.h"
#include "adc.h"
#include "adcldo.h"
#include "cgc.h"
#include "gpio.h"
#include "pga.h"

/**
 * @file Board_Analog.c
 * @brief 板级电流采样和 ADC/PGA 配置实现。
 * @details 运行采样按当前 PWM duty 动态选择低边窗口最大的两相，第三相由 KCL 重构。
 */

#define CURRENT_SAMPLE_PAIR_NONE 255U

typedef struct
{
    int16_t u;
    int16_t v;
    int16_t w;
} CurrentPhysicalSample_t;

typedef struct
{
    uint8_t pair;
    uint32_t channel_mask;
    uint32_t last_channel;
    uint32_t last_channel_mask;
} CurrentPairConfig_t;

static volatile uint16_t s_iu_raw;
static volatile uint16_t s_iv_raw;
static volatile uint16_t s_iw_raw;

static uint16_t s_iu_offset;
static uint16_t s_iv_offset;
static uint16_t s_iw_offset;

static volatile int16_t s_iu_raw_cnt;
static volatile int16_t s_iv_raw_cnt;
static volatile int16_t s_iw_raw_cnt;
static volatile int16_t s_iuvw_raw_sum;

static volatile int16_t s_iu_cnt;
static volatile int16_t s_iv_cnt;
static volatile int16_t s_iw_cnt;
static volatile int16_t s_iuvw_sum;

static volatile uint32_t s_sync_count;
static volatile uint8_t s_adc_sync_started;
static volatile uint8_t s_current_sample_multi_enabled;
static volatile uint8_t s_current_sample_dynamic_enabled;
static volatile uint8_t s_current_sample_stage;
static volatile uint8_t s_current_sample_count;
static volatile uint8_t s_current_sample_pair;
static volatile uint8_t s_current_sample_hold;
static volatile uint16_t s_current_sample_hold_count;
static volatile uint16_t s_current_sample_center_tick;
static volatile uint16_t s_current_sample_tick_a;
static volatile uint16_t s_current_sample_tick_b;
static volatile uint16_t s_current_sample_window_u;
static volatile uint16_t s_current_sample_window_v;
static volatile uint16_t s_current_sample_window_w;
static volatile int16_t s_current_sample_a_first;
static volatile int16_t s_current_sample_a_second;
static volatile int16_t s_current_sample_b_first;
static volatile int16_t s_current_sample_b_second;
static volatile int16_t s_current_sample_spread_first;
static volatile int16_t s_current_sample_spread_second;

static uint32_t s_current_adc_last_ch;
static uint32_t s_current_adc_last_ch_mask;
static CurrentPhysicalSample_t s_sample_a;
static CurrentPhysicalSample_t s_sample_b;

static void Board_InitAdcPins(void);
static void Board_InitAdcLdo(void);
static void Board_InitPga(void);
static void Board_AnalogInitAdc(void);
static uint16_t ReadAdcSingle(uint32_t ch, uint32_t ch_msk);
static void ClearAdcSyncFlags(void);
static void ResetAdcSyncState(void);
static void ClearCurrentSampleDiag(void);
static void ConfigPwmCurrentTrigger(void);
static void ConfigCurrentTrigger(uint8_t pair);
static uint8_t GetPairConfig(uint8_t pair, CurrentPairConfig_t* cfg);
static void SelectCurrentSampleWindow(void);
static uint8_t SelectBestPair(uint16_t wu, uint16_t wv, uint16_t ww, uint8_t* pair,
                              uint16_t* common_width);
static void TryPair(uint8_t pair, uint16_t wa, uint16_t wb, uint8_t* best_pair,
                    uint16_t* best_common, uint16_t* best_sum);
static void CaptureSelectedPairRaw(CurrentPhysicalSample_t* sample);
static void ResolveSelectedPairRaw(void);
static void ReconstructPhysicalCurrent(uint8_t pair, const CurrentPhysicalSample_t* sample,
                                       CurrentPhysicalSample_t* physical);
static void ApplyPhysicalCurrent(const CurrentPhysicalSample_t* physical);
static void MapPhysicalToLogicCurrent(const CurrentPhysicalSample_t* physical);
static uint16_t ClampU16ToPwm(uint16_t tick);
static uint16_t MinU16(uint16_t a, uint16_t b);
static int16_t ClampCurrentSampleSpread(int32_t value);
static uint16_t CurrentSampleCntToAdc(int16_t cnt, uint16_t offset);

void Board_InitAdc(void)
{
    s_current_sample_multi_enabled = (uint8_t)(CURRENT_SAMPLE_MULTI_ENABLE != 0U);
    s_current_sample_dynamic_enabled = (uint8_t)(CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U);

    Board_InitAdcPins();
    Board_InitAdcLdo();
    Board_InitPga();
    Board_AnalogInitAdc();
    ClearCurrentSampleDiag();
}

void Board_SampleAdc(void)
{
    s_iu_raw = ReadAdcSingle(ADC_CH_0, ADC_CH_0_MSK);
    s_iv_raw = ReadAdcSingle(ADC_CH_2, ADC_CH_2_MSK);
    s_iw_raw = ReadAdcSingle(ADC_CH_3, ADC_CH_3_MSK);
}

void Board_CalibCurrentOffset(uint16_t samples)
{
    uint32_t sum_u = 0;
    uint32_t sum_v = 0;
    uint32_t sum_w = 0;

    for (uint16_t i = 0; i < samples; i++)
    {
        Board_SampleAdc();

        sum_u += s_iu_raw;
        sum_v += s_iv_raw;
        sum_w += s_iw_raw;
    }

    s_iu_offset = (uint16_t)(sum_u / samples);
    s_iv_offset = (uint16_t)(sum_v / samples);
    s_iw_offset = (uint16_t)(sum_w / samples);
}

void Board_CalibCurrentOffsetPwm(uint16_t samples)
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
        Board_SampleAdc();

        sum_u += s_iu_raw;
        sum_v += s_iv_raw;
        sum_w += s_iw_raw;
    }

    s_iu_offset = (uint16_t)(sum_u / samples);
    s_iv_offset = (uint16_t)(sum_v / samples);
    s_iw_offset = (uint16_t)(sum_w / samples);

    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_CONTINUOUS, ADC_CLK_DIV_1, 25);
    ConfigPwmCurrentTrigger();
    ClearAdcSyncFlags();
    ADC_DisableChannelInt(ADC_CH_0_MSK | ADC_CH_2_MSK | ADC_CH_3_MSK);
    ADC_EnableChannelInt(s_current_adc_last_ch_mask);
    NVIC_EnableIRQ(ADC_IRQn);
    ADC_Go();
}

void Board_UpdateCurrent(void)
{
    CurrentPhysicalSample_t physical;

    physical.u = (int16_t)((int16_t)s_iu_raw - (int16_t)s_iu_offset);
    physical.v = (int16_t)((int16_t)s_iv_raw - (int16_t)s_iv_offset);
    physical.w = (int16_t)((int16_t)s_iw_raw - (int16_t)s_iw_offset);
    ApplyPhysicalCurrent(&physical);
}

void Board_InitPwmAdcSync(void)
{
    ResetAdcSyncState();
    s_adc_sync_started = 1U;

    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_CONTINUOUS, ADC_CLK_DIV_1, 25);
    ConfigPwmCurrentTrigger();
    ClearAdcSyncFlags();
    NVIC_EnableIRQ(ADC_IRQn);
    ADC_Go();
}

void Board_UpdateCurrentSampleTiming(void)
{
    if (s_adc_sync_started == 0U)
    {
        return;
    }

    ConfigPwmCurrentTrigger();
}

uint8_t Board_AdcIrqHandler(void)
{
    if (ADC_GetChannelIntFlag(s_current_adc_last_ch) == 0U)
    {
        return 0U;
    }

    ADC_ClearChannelIntFlag(s_current_adc_last_ch);

    if (s_current_sample_multi_enabled != 0U)
    {
        if (s_current_sample_stage == 0U)
        {
            CaptureSelectedPairRaw(&s_sample_a);
            s_current_sample_stage = 1U;
            s_current_sample_count = 1U;
            return 0U;
        }

        CaptureSelectedPairRaw(&s_sample_b);
        s_current_sample_stage = 0U;
        s_current_sample_count = 2U;
        ResolveSelectedPairRaw();
    }
    else
    {
        CaptureSelectedPairRaw(&s_sample_a);
        s_sample_b = s_sample_a;
        s_current_sample_count = 1U;
        ResolveSelectedPairRaw();
    }

    s_sync_count++;
    return 1U;
}

int16_t Board_GetIuCnt(void)
{
    return s_iu_cnt;
}

int16_t Board_GetIvCnt(void)
{
    return s_iv_cnt;
}

int16_t Board_GetIwCnt(void)
{
    return s_iw_cnt;
}

int16_t Board_GetIuvwSum(void)
{
    return s_iuvw_sum;
}

int16_t Board_GetIuRawCnt(void)
{
    return s_iu_raw_cnt;
}

int16_t Board_GetIvRawCnt(void)
{
    return s_iv_raw_cnt;
}

int16_t Board_GetIwRawCnt(void)
{
    return s_iw_raw_cnt;
}

int16_t Board_GetIuvwRawSum(void)
{
    return s_iuvw_raw_sum;
}

uint16_t Board_GetIuRawAdc(void)
{
    return s_iu_raw;
}

uint16_t Board_GetIvRawAdc(void)
{
    return s_iv_raw;
}

uint16_t Board_GetIwRawAdc(void)
{
    return s_iw_raw;
}

uint32_t Board_GetAdcSyncCount(void)
{
    return s_sync_count;
}

uint8_t Board_IsCurrentSampleMultiEnabled(void)
{
    return s_current_sample_multi_enabled;
}

uint8_t Board_IsCurrentSampleDynamicEnabled(void)
{
    return s_current_sample_dynamic_enabled;
}

uint8_t Board_GetCurrentSamplePair(void)
{
    return s_current_sample_pair;
}

uint8_t Board_GetCurrentSampleHold(void)
{
    return s_current_sample_hold;
}

uint16_t Board_GetCurrentSampleHoldCount(void)
{
    return s_current_sample_hold_count;
}

uint16_t Board_GetCurrentSampleCenterTick(void)
{
    return s_current_sample_center_tick;
}

uint16_t Board_GetCurrentSampleWindowU(void)
{
    return s_current_sample_window_u;
}

uint16_t Board_GetCurrentSampleWindowV(void)
{
    return s_current_sample_window_v;
}

uint16_t Board_GetCurrentSampleWindowW(void)
{
    return s_current_sample_window_w;
}

uint8_t Board_GetCurrentSampleDiagStage(void)
{
    return s_current_sample_stage;
}

uint8_t Board_GetCurrentSampleDiagCount(void)
{
    return s_current_sample_count;
}

int16_t Board_GetCurrentSampleAFirst(void)
{
    return s_current_sample_a_first;
}

int16_t Board_GetCurrentSampleASecond(void)
{
    return s_current_sample_a_second;
}

int16_t Board_GetCurrentSampleBFirst(void)
{
    return s_current_sample_b_first;
}

int16_t Board_GetCurrentSampleBSecond(void)
{
    return s_current_sample_b_second;
}

int16_t Board_GetCurrentSampleSpreadFirst(void)
{
    return s_current_sample_spread_first;
}

int16_t Board_GetCurrentSampleSpreadSecond(void)
{
    return s_current_sample_spread_second;
}

static void Board_InitAdcPins(void)
{
    GPIO_Init(PORT0, PIN0, ANALOG_INPUT);
    GPIO_Init(PORT0, PIN1, ANALOG_INPUT);

    GPIO_Init(PORT2, PIN4, ANALOG_INPUT);
    GPIO_Init(PORT2, PIN5, ANALOG_INPUT);

    GPIO_Init(PORT2, PIN6, ANALOG_INPUT);
    GPIO_Init(PORT2, PIN7, ANALOG_INPUT);

    GPIO_Init(PORT2, PIN0, ANALOG_INPUT);
}

static void Board_InitAdcLdo(void)
{
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCLDO, ENABLE);

    ADCLDO_OutVlotageSel(ADCLDO_OutV_3d6);
    ADCLDO_Enable();
}

static void Board_InitPga(void)
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

static void Board_AnalogInitAdc(void)
{
    CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCEN, ENABLE);

    ADC_ConfigRunMode(ADC_MODE_HIGH, ADC_CONVERT_SINGLE, ADC_CLK_DIV_1, 25);
    ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);
    ADC_DisableChargeAndDischarge();
    ADC_ConfigVREF(ADC_VREFP_AVREFP);
    ADC_Start();
}

static uint16_t ReadAdcSingle(uint32_t ch, uint32_t ch_msk)
{
    ADC_DisableScanChannel(0xFFFFFFFFUL);
    ADC_EnableScanChannel(ch_msk);

    ADC_Go();

    while (ADC_IS_BUSY())
    {
    }

    return (uint16_t)ADC_GetResult(ch);
}

static void ClearAdcSyncFlags(void)
{
    ADC_ClearChannelIntFlag(ADC_CH_0);
    ADC_ClearChannelIntFlag(ADC_CH_2);
    ADC_ClearChannelIntFlag(ADC_CH_3);
    NVIC_ClearPendingIRQ(ADC_IRQn);
}

static void ResetAdcSyncState(void)
{
    s_sync_count = 0;
    s_adc_sync_started = 0U;
    ClearCurrentSampleDiag();
}

static void ClearCurrentSampleDiag(void)
{
    s_current_sample_stage = 0U;
    s_current_sample_count = 0U;
    s_current_sample_pair = CURRENT_SAMPLE_PAIR_NONE;
    s_current_sample_hold = 0U;
    s_current_sample_hold_count = 0U;
    s_current_sample_center_tick = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_current_sample_tick_a = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_current_sample_tick_b = PWM_ADC_TRIGGER_TICK_DEFAULT;
    s_current_sample_window_u = 0U;
    s_current_sample_window_v = 0U;
    s_current_sample_window_w = 0U;
    s_current_sample_a_first = 0;
    s_current_sample_a_second = 0;
    s_current_sample_b_first = 0;
    s_current_sample_b_second = 0;
    s_current_sample_spread_first = 0;
    s_current_sample_spread_second = 0;
    s_sample_a.u = 0;
    s_sample_a.v = 0;
    s_sample_a.w = 0;
    s_sample_b.u = 0;
    s_sample_b.v = 0;
    s_sample_b.w = 0;
}

static void ConfigPwmCurrentTrigger(void)
{
#if (CURRENT_SAMPLE_DYNAMIC_ENABLE != 0U)
    SelectCurrentSampleWindow();
#else
    s_current_sample_pair = CURRENT_SAMPLE_PAIR;
    s_current_sample_center_tick = Board_GetAdcTriggerTick();
    Board_SetAdcTriggerTick(s_current_sample_center_tick);
    ConfigCurrentTrigger(s_current_sample_pair);
#endif
}

static void ConfigCurrentTrigger(uint8_t pair)
{
    CurrentPairConfig_t cfg;

    if (GetPairConfig(pair, &cfg) == 0U)
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

    s_current_sample_stage = 0U;
    s_current_sample_count = 0U;

    ADC_DisableChannelInt(ADC_CH_0_MSK | ADC_CH_2_MSK | ADC_CH_3_MSK);
    ClearAdcSyncFlags();
    ADC_EnableChannelInt(cfg.last_channel_mask);

    ADC_EnableEPWMCmp0TriggerChannel(cfg.channel_mask);
    ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP0);
    if (s_current_sample_multi_enabled != 0U)
    {
        ADC_EnableEPWMCmp1TriggerChannel(cfg.channel_mask);
        ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP1);
    }

    s_current_adc_last_ch = cfg.last_channel;
    s_current_adc_last_ch_mask = cfg.last_channel_mask;
}

static uint8_t GetPairConfig(uint8_t pair, CurrentPairConfig_t* cfg)
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

        case CURRENT_SAMPLE_VW:
        default:
            cfg->channel_mask = ADC_CH_2_MSK | ADC_CH_3_MSK;
            cfg->last_channel = ADC_CH_3;
            cfg->last_channel_mask = ADC_CH_3_MSK;
            return 1U;
    }
}

static void SelectCurrentSampleWindow(void)
{
    volatile uint16_t duty_u;
    volatile uint16_t duty_v;
    volatile uint16_t duty_w;
    volatile uint8_t output_on;
    volatile uint8_t brake_on;
    uint8_t pair;
    uint16_t common_width;
    uint16_t center;
    uint16_t delta;

    Board_GetPwmDebug(&duty_u, &duty_v, &duty_w, &output_on, &brake_on);
    (void)output_on;
    (void)brake_on;

    s_current_sample_window_u = ClampU16ToPwm((uint16_t)duty_u);
    s_current_sample_window_v = ClampU16ToPwm((uint16_t)duty_v);
    s_current_sample_window_w = ClampU16ToPwm((uint16_t)duty_w);

    if (SelectBestPair(s_current_sample_window_u, s_current_sample_window_v,
                       s_current_sample_window_w, &pair, &common_width) == 0U)
    {
        s_current_sample_hold = 1U;
        if (s_current_sample_hold_count < 65535U)
        {
            s_current_sample_hold_count++;
        }
        if (s_current_sample_pair == CURRENT_SAMPLE_PAIR_NONE)
        {
            s_current_sample_pair = CURRENT_SAMPLE_PAIR;
            ConfigCurrentTrigger(s_current_sample_pair);
        }
        return;
    }

    center = (uint16_t)(common_width / 2U);
    delta = (uint16_t)(common_width / 4U);
    if (delta > CURRENT_SAMPLE_MULTI_DELTA_TICK)
    {
        delta = CURRENT_SAMPLE_MULTI_DELTA_TICK;
    }
    if (delta == 0U)
    {
        delta = 1U;
    }

    s_current_sample_hold = 0U;
    s_current_sample_pair = pair;
    s_current_sample_center_tick = center;
    s_current_sample_tick_a = ClampU16ToPwm((uint16_t)(center + delta));
    s_current_sample_tick_b = (center > delta) ? (uint16_t)(center - delta) : 0U;

    if (s_current_sample_multi_enabled != 0U)
    {
        Board_SetAdcTriggerTicks(s_current_sample_tick_a, s_current_sample_tick_b);
    }
    else
    {
        Board_SetAdcTriggerTick(center);
    }
    ConfigCurrentTrigger(pair);
}

static uint8_t SelectBestPair(uint16_t wu, uint16_t wv, uint16_t ww, uint8_t* pair,
                              uint16_t* common_width)
{
    uint8_t best_pair = CURRENT_SAMPLE_PAIR_NONE;
    uint16_t best_common = 0U;
    uint16_t best_sum = 0U;

    TryPair(CURRENT_SAMPLE_UV, wu, wv, &best_pair, &best_common, &best_sum);
    TryPair(CURRENT_SAMPLE_UW, wu, ww, &best_pair, &best_common, &best_sum);
    TryPair(CURRENT_SAMPLE_VW, wv, ww, &best_pair, &best_common, &best_sum);

    if (best_pair == CURRENT_SAMPLE_PAIR_NONE)
    {
        return 0U;
    }

    *pair = best_pair;
    *common_width = best_common;
    return 1U;
}

static void TryPair(uint8_t pair, uint16_t wa, uint16_t wb, uint8_t* best_pair,
                    uint16_t* best_common, uint16_t* best_sum)
{
    uint16_t common;
    uint16_t sum;

    if ((wa < CURRENT_SAMPLE_MIN_WINDOW_TICK) || (wb < CURRENT_SAMPLE_MIN_WINDOW_TICK))
    {
        return;
    }

    common = MinU16(wa, wb);
    sum = (uint16_t)(wa + wb);
    if ((common > *best_common) || ((common == *best_common) && (sum > *best_sum)))
    {
        *best_pair = pair;
        *best_common = common;
        *best_sum = sum;
    }
}

static void CaptureSelectedPairRaw(CurrentPhysicalSample_t* sample)
{
    int16_t first = 0;
    int16_t second = 0;

    sample->u = 0;
    sample->v = 0;
    sample->w = 0;

    switch (s_current_sample_pair)
    {
        case CURRENT_SAMPLE_UV:
            sample->u = (int16_t)((int16_t)ADC_GetResult(ADC_CH_0) - (int16_t)s_iu_offset);
            sample->v = (int16_t)((int16_t)ADC_GetResult(ADC_CH_2) - (int16_t)s_iv_offset);
            first = sample->u;
            second = sample->v;
            break;

        case CURRENT_SAMPLE_UW:
            sample->u = (int16_t)((int16_t)ADC_GetResult(ADC_CH_0) - (int16_t)s_iu_offset);
            sample->w = (int16_t)((int16_t)ADC_GetResult(ADC_CH_3) - (int16_t)s_iw_offset);
            first = sample->u;
            second = sample->w;
            break;

        case CURRENT_SAMPLE_VW:
        default:
            sample->v = (int16_t)((int16_t)ADC_GetResult(ADC_CH_2) - (int16_t)s_iv_offset);
            sample->w = (int16_t)((int16_t)ADC_GetResult(ADC_CH_3) - (int16_t)s_iw_offset);
            first = sample->v;
            second = sample->w;
            break;
    }

    if (s_current_sample_stage == 0U)
    {
        s_current_sample_a_first = first;
        s_current_sample_a_second = second;
    }
    else
    {
        s_current_sample_b_first = first;
        s_current_sample_b_second = second;
    }
}

static void ResolveSelectedPairRaw(void)
{
    CurrentPhysicalSample_t avg;
    CurrentPhysicalSample_t physical;

    if (s_current_sample_hold != 0U)
    {
        return;
    }

    avg.u = (int16_t)(((int32_t)s_sample_a.u + (int32_t)s_sample_b.u) / 2);
    avg.v = (int16_t)(((int32_t)s_sample_a.v + (int32_t)s_sample_b.v) / 2);
    avg.w = (int16_t)(((int32_t)s_sample_a.w + (int32_t)s_sample_b.w) / 2);

    s_current_sample_spread_first =
        ClampCurrentSampleSpread((int32_t)s_current_sample_b_first -
                                 (int32_t)s_current_sample_a_first);
    s_current_sample_spread_second =
        ClampCurrentSampleSpread((int32_t)s_current_sample_b_second -
                                 (int32_t)s_current_sample_a_second);
    ReconstructPhysicalCurrent(s_current_sample_pair, &avg, &physical);
    ApplyPhysicalCurrent(&physical);
}

static void ReconstructPhysicalCurrent(uint8_t pair, const CurrentPhysicalSample_t* sample,
                                       CurrentPhysicalSample_t* physical)
{
    physical->u = sample->u;
    physical->v = sample->v;
    physical->w = sample->w;

    switch (pair)
    {
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

static void ApplyPhysicalCurrent(const CurrentPhysicalSample_t* physical)
{
    s_iu_raw_cnt = physical->u;
    s_iv_raw_cnt = physical->v;
    s_iw_raw_cnt = physical->w;
    s_iuvw_raw_sum = (int16_t)(physical->u + physical->v + physical->w);

    s_iu_raw = CurrentSampleCntToAdc(physical->u, s_iu_offset);
    s_iv_raw = CurrentSampleCntToAdc(physical->v, s_iv_offset);
    s_iw_raw = CurrentSampleCntToAdc(physical->w, s_iw_offset);

    MapPhysicalToLogicCurrent(physical);
}

static void MapPhysicalToLogicCurrent(const CurrentPhysicalSample_t* physical)
{
    /*
     * 保持现有 FOC 坐标：logic U/V/W = physical V/W/U。
     * 这样动态切 UV/UW/VW 采样时，不需要重新改电角零点和相序。
     */
    s_iu_cnt = physical->v;
    s_iv_cnt = physical->w;
    s_iw_cnt = physical->u;
    s_iuvw_sum = (int16_t)(s_iu_cnt + s_iv_cnt + s_iw_cnt);
}

static uint16_t ClampU16ToPwm(uint16_t tick)
{
    if (tick > PWM_PERIOD)
    {
        return PWM_PERIOD;
    }
    return tick;
}

static uint16_t MinU16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

static int16_t ClampCurrentSampleSpread(int32_t value)
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

static uint16_t CurrentSampleCntToAdc(int16_t cnt, uint16_t offset)
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
