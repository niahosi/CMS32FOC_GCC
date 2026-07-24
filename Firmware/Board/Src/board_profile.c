/**
 * @file board_profile.c
 * @brief Internal timing profiler based on TMR1 free-running counter.
 */

#include "board_profile.h"

#include "BoardConfig.h"
#include "CMS32M6510.h"
#include "cgc.h"
#include "timer.h"

#include <stdint.h>

volatile BoardProfileWatch_t g_board_profile;

static uint32_t s_adc_irq_start;
static uint32_t s_motor_fast_start;
static uint32_t s_encoder_fast_start;
static uint32_t s_current_control_start;
static uint32_t s_foc_math_start;
static uint32_t s_pwm_update_start;
static uint32_t s_speed_slot_start;

static uint32_t timer_now(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    return TMR_GetTimingData(TMR1);
#else
    return 0U;
#endif
}

static uint32_t elapsed_ticks(uint32_t start, uint32_t end)
{
    /*
     * TMR0/TMR1 behave as LOAD-based down counters. Unsigned subtraction keeps
     * the elapsed value correct across one 32-bit wrap.
     */
    return start - end;
}

static void update_max(volatile uint32_t* target, uint32_t value)
{
    if (value > *target)
    {
        *target = value;
    }
}

static void handle_clear_request(void)
{
    const uint32_t seq = g_board_profile.clear_seq;
    if (seq == g_board_profile.clear_seen_seq)
    {
        return;
    }

    g_board_profile.adc_irq_max_ticks = 0U;
    g_board_profile.adc_irq_over_budget_count = 0U;
    g_board_profile.motor_fast_max_ticks = 0U;
    g_board_profile.encoder_fast_max_ticks = 0U;
    g_board_profile.current_control_max_ticks = 0U;
    g_board_profile.foc_math_max_ticks = 0U;
    g_board_profile.pwm_update_max_ticks = 0U;
    g_board_profile.speed_slot_max_ticks = 0U;
    g_board_profile.clear_seen_seq = seq;
}

void BoardProfile_Init(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    CGC_PER11PeriphClockCmd(CGC_PER11Periph_TIMER01, ENABLE);

    NVIC_DisableIRQ(TIMER1_IRQn);
    NVIC_SetPriority(TIMER1_IRQn, BOARD_PROFILE_TIMER_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(TIMER1_IRQn);

    TMR_Stop(TMR1);
    TMR_DisableOverflowInt(TMR1);
    TMR_ConfigRunMode(TMR1, TMR_COUNT_CONTINUONS_MODE, TMR_BIT_32_MODE);
    TMR_ConfigClk(TMR1, TMR_CLK_DIV_16);
    TMR_SetPeriod(TMR1, 0xFFFFFFFFUL);
    TMR1->BGLOAD = 0xFFFFFFFFUL;
    TMR_ClearOverflowIntFlag(TMR1);

    g_board_profile = (BoardProfileWatch_t){
        .timer_hz = SystemCoreClock / BOARD_PROFILE_TIMER_DIVIDER,
        .tick_ns = 1000000000UL / (SystemCoreClock / BOARD_PROFILE_TIMER_DIVIDER),
        .adc_budget_ticks =
            (SystemCoreClock / BOARD_PROFILE_TIMER_DIVIDER) / PWM_FREQ_HZ,
    };

    TMR_Start(TMR1);
#else
    g_board_profile = (BoardProfileWatch_t){0};
#endif
}

void BoardProfile_AdcIrqBegin(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    handle_clear_request();
    s_adc_irq_start = timer_now();
#endif
}

void BoardProfile_AdcIrqEnd(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    const uint32_t elapsed = elapsed_ticks(s_adc_irq_start, timer_now());
    g_board_profile.adc_irq_last_ticks = elapsed;
    g_board_profile.adc_irq_count++;
    update_max(&g_board_profile.adc_irq_max_ticks, elapsed);
    if (elapsed > g_board_profile.adc_budget_ticks)
    {
        g_board_profile.adc_irq_over_budget_count++;
    }
#endif
}

void BoardProfile_MotorFastBegin(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    s_motor_fast_start = timer_now();
#endif
}

void BoardProfile_MotorFastEnd(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    const uint32_t elapsed = elapsed_ticks(s_motor_fast_start, timer_now());
    g_board_profile.motor_fast_last_ticks = elapsed;
    update_max(&g_board_profile.motor_fast_max_ticks, elapsed);
#endif
}

void BoardProfile_EncoderFastBegin(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    s_encoder_fast_start = timer_now();
#endif
}

void BoardProfile_EncoderFastEnd(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    const uint32_t elapsed = elapsed_ticks(s_encoder_fast_start, timer_now());
    g_board_profile.encoder_fast_last_ticks = elapsed;
    update_max(&g_board_profile.encoder_fast_max_ticks, elapsed);
#endif
}

void BoardProfile_CurrentControlBegin(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    s_current_control_start = timer_now();
#endif
}

void BoardProfile_CurrentControlEnd(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    const uint32_t elapsed = elapsed_ticks(s_current_control_start, timer_now());
    g_board_profile.current_control_last_ticks = elapsed;
    update_max(&g_board_profile.current_control_max_ticks, elapsed);
#endif
}

void BoardProfile_FocMathBegin(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    s_foc_math_start = timer_now();
#endif
}

void BoardProfile_FocMathEnd(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    const uint32_t elapsed = elapsed_ticks(s_foc_math_start, timer_now());
    g_board_profile.foc_math_last_ticks = elapsed;
    update_max(&g_board_profile.foc_math_max_ticks, elapsed);
#endif
}

void BoardProfile_PwmUpdateBegin(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    s_pwm_update_start = timer_now();
#endif
}

void BoardProfile_PwmUpdateEnd(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    const uint32_t elapsed = elapsed_ticks(s_pwm_update_start, timer_now());
    g_board_profile.pwm_update_last_ticks = elapsed;
    update_max(&g_board_profile.pwm_update_max_ticks, elapsed);
#endif
}

void BoardProfile_SpeedSlotBegin(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    s_speed_slot_start = timer_now();
#endif
}

void BoardProfile_SpeedSlotEnd(void)
{
#if (BOARD_PROFILE_ENABLE != 0U)
    const uint32_t elapsed = elapsed_ticks(s_speed_slot_start, timer_now());
    g_board_profile.speed_slot_last_ticks = elapsed;
    g_board_profile.speed_slot_count++;
    update_max(&g_board_profile.speed_slot_max_ticks, elapsed);
#endif
}
