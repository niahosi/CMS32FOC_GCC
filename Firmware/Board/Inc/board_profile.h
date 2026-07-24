/**
 * @file board_profile.h
 * @brief Board-level timing profiler using a free-running hardware timer.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /** Timer tick frequency. With current 64 MHz / 16 setup this is 4 MHz. */
    uint32_t timer_hz;
    /** One profiling tick in nanoseconds. Current value is about 250 ns. */
    uint32_t tick_ns;
    /** Expected ADC IRQ period budget in profiling ticks. Current 20 kHz is 200 ticks. */
    uint32_t adc_budget_ticks;

    /** Last measured full ADC_IRQHandler duration, in profiling ticks. */
    uint32_t adc_irq_last_ticks;
    /** Max measured full ADC_IRQHandler duration, in profiling ticks. */
    uint32_t adc_irq_max_ticks;
    /** Number of ADC IRQ samples whose duration exceeded adc_budget_ticks. */
    uint32_t adc_irq_over_budget_count;

    /** Last measured MotorControl_FastLoopFromAdcIrq() duration, in profiling ticks. */
    uint32_t motor_fast_last_ticks;
    /** Max measured MotorControl_FastLoopFromAdcIrq() duration, in profiling ticks. */
    uint32_t motor_fast_max_ticks;

    /** Last MA600 angle-read duration inside the current fast loop, in ticks. */
    uint32_t encoder_fast_last_ticks;
    /** Max MA600 angle-read duration inside the current fast loop, in ticks. */
    uint32_t encoder_fast_max_ticks;

    /** Last FOC current-control calculation duration, in ticks. */
    uint32_t current_control_last_ticks;
    /** Max FOC current-control calculation duration, in ticks. */
    uint32_t current_control_max_ticks;

    /** Last Clarke/Park/PI/InvPark/SVPWM calculation duration, in ticks. */
    uint32_t foc_math_last_ticks;
    /** Max Clarke/Park/PI/InvPark/SVPWM calculation duration, in ticks. */
    uint32_t foc_math_max_ticks;

    /** Last PWM duty write plus next ADC-window update duration, in ticks. */
    uint32_t pwm_update_last_ticks;
    /** Max PWM duty write plus next ADC-window update duration, in ticks. */
    uint32_t pwm_update_max_ticks;

    /** Last measured 1 kHz encoder-speed/position/speed-PI slot duration, in ticks. */
    uint32_t speed_slot_last_ticks;
    /** Max measured 1 kHz encoder-speed/position/speed-PI slot duration, in ticks. */
    uint32_t speed_slot_max_ticks;
    /** Number of measured 1 kHz speed/position slots. */
    uint32_t speed_slot_count;

    /** Full ADC IRQ profiling sample count. */
    uint32_t adc_irq_count;
    /** Write a new value from Ozone to clear max counters. */
    uint32_t clear_seq;
    /** Internal echo of the last handled clear_seq. */
    uint32_t clear_seen_seq;
} BoardProfileWatch_t;

extern volatile BoardProfileWatch_t g_board_profile;

void BoardProfile_Init(void);
void BoardProfile_AdcIrqBegin(void);
void BoardProfile_AdcIrqEnd(void);
void BoardProfile_MotorFastBegin(void);
void BoardProfile_MotorFastEnd(void);
void BoardProfile_EncoderFastBegin(void);
void BoardProfile_EncoderFastEnd(void);
void BoardProfile_CurrentControlBegin(void);
void BoardProfile_CurrentControlEnd(void);
void BoardProfile_FocMathBegin(void);
void BoardProfile_FocMathEnd(void);
void BoardProfile_PwmUpdateBegin(void);
void BoardProfile_PwmUpdateEnd(void);
void BoardProfile_SpeedSlotBegin(void);
void BoardProfile_SpeedSlotEnd(void);

#ifdef __cplusplus
}
#endif
