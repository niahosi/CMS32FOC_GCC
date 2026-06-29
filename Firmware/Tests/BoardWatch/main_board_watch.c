#include <stdint.h>

#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_pwm.h"

#define BOARD_WATCH_BOOT_MAGIC 0x32574F46UL
#define BOARD_WATCH_INIT_DONE 0x00000001UL
#define BOARD_WATCH_PWM_FORCED_OFF 0x00000002UL

typedef struct
{
    volatile uint32_t boot_magic;
    volatile uint32_t flags;
    volatile uint32_t loop_count;
    volatile uint32_t adc_irq_count;
    volatile uint32_t adc_sync_count;

    volatile uint16_t ma600_raw;
    volatile uint8_t ma600_ok;
    volatile uint8_t ma600_age;
    volatile uint32_t ma600_success_count;
    volatile uint32_t ma600_fail_count;

    volatile uint16_t iu_raw_adc;
    volatile uint16_t iv_raw_adc;
    volatile uint16_t iw_raw_adc;
    volatile int16_t iu_cnt;
    volatile int16_t iv_cnt;
    volatile int16_t iw_cnt;
    volatile int16_t iuvw_sum;

    volatile uint16_t duty_u;
    volatile uint16_t duty_v;
    volatile uint16_t duty_w;
    volatile uint16_t adc_trigger_tick;
    volatile uint16_t adc_trigger_tick_a;
    volatile uint16_t adc_trigger_tick_b;
    volatile uint8_t pwm_output_on;
    volatile uint8_t pwm_brake_on;
    volatile uint8_t pwm_off_safe;
    volatile uint8_t pwm_run_ok;

    volatile uint8_t sample_pair;
    volatile uint8_t sample_hold;
    volatile uint16_t sample_hold_count;
    volatile uint16_t sample_pair_hold_left;
    volatile uint16_t sample_center_tick;
    volatile uint16_t sample_window_u;
    volatile uint16_t sample_window_v;
    volatile uint16_t sample_window_w;
    volatile uint8_t sample_three_shunt;
    volatile uint16_t sample_common_window;
    volatile uint32_t sample_switch_count;
    volatile uint32_t sample_fallback_count;
    volatile uint32_t iv_spike_count;
    volatile uint32_t iw_spike_count;
    volatile uint16_t iv_max_step;
    volatile uint16_t iw_max_step;
} BoardWatchState_t;

volatile BoardWatchState_t g_board_watch;

static void BoardWatch_UpdateSnapshot(void);
static void BoardWatch_Delay(void);

int main(void)
{
    g_board_watch.boot_magic = BOARD_WATCH_BOOT_MAGIC;

    bsp_init();
    g_board_watch.flags |= BOARD_WATCH_INIT_DONE;

    pwm_off();
    g_board_watch.flags |= BOARD_WATCH_PWM_FORCED_OFF;

    while (1)
    {
        if (bsp_update_angle() != 0U)
        {
            g_board_watch.ma600_success_count++;
        }
        else
        {
            g_board_watch.ma600_fail_count++;
        }

        BoardWatch_UpdateSnapshot();
        g_board_watch.loop_count++;
        BoardWatch_Delay();
    }
}

void ADC_IRQHandler(void)
{
    if (bsp_adc_irq() != 0U)
    {
        g_board_watch.adc_irq_count++;
    }
}

static void BoardWatch_UpdateSnapshot(void)
{
    volatile uint16_t duty_u = 0U;
    volatile uint16_t duty_v = 0U;
    volatile uint16_t duty_w = 0U;
    volatile uint8_t output_on = 0U;
    volatile uint8_t brake_on = 0U;

    pwm_snapshot(&duty_u, &duty_v, &duty_w, &output_on, &brake_on);

    g_board_watch.adc_sync_count = curr_sync_count();

    g_board_watch.ma600_raw = bsp_angle_raw();
    g_board_watch.ma600_ok = bsp_angle_ok();
    g_board_watch.ma600_age = bsp_angle_age();

    g_board_watch.iu_raw_adc = curr_raw_adc_u();
    g_board_watch.iv_raw_adc = curr_raw_adc_v();
    g_board_watch.iw_raw_adc = curr_raw_adc_w();
    g_board_watch.iu_cnt = curr_u();
    g_board_watch.iv_cnt = curr_v();
    g_board_watch.iw_cnt = curr_w();
    g_board_watch.iuvw_sum = curr_sum();

    g_board_watch.duty_u = duty_u;
    g_board_watch.duty_v = duty_v;
    g_board_watch.duty_w = duty_w;
    g_board_watch.adc_trigger_tick = pwm_adc_trigger();
    g_board_watch.adc_trigger_tick_a = pwm_adc_trigger_a();
    g_board_watch.adc_trigger_tick_b = pwm_adc_trigger_b();
    g_board_watch.pwm_output_on = output_on;
    g_board_watch.pwm_brake_on = brake_on;
    g_board_watch.pwm_off_safe = pwm_is_off_safe();
    g_board_watch.pwm_run_ok = pwm_is_running();

    g_board_watch.sample_pair = curr_pair();
    g_board_watch.sample_hold = curr_is_hold();
    g_board_watch.sample_hold_count = curr_hold_count();
    g_board_watch.sample_pair_hold_left = curr_sample_pair_hold_left();
    g_board_watch.sample_center_tick = curr_center_tick();
    g_board_watch.sample_window_u = curr_window_u();
    g_board_watch.sample_window_v = curr_window_v();
    g_board_watch.sample_window_w = curr_window_w();
    g_board_watch.sample_three_shunt = curr_three_shunt_active();
    g_board_watch.sample_common_window = curr_window_common();
    g_board_watch.sample_switch_count = curr_sample_switch_count();
    g_board_watch.sample_fallback_count = curr_sample_fallback_count();
    g_board_watch.iv_spike_count = curr_iv_spike_count();
    g_board_watch.iw_spike_count = curr_iw_spike_count();
    g_board_watch.iv_max_step = curr_iv_max_step();
    g_board_watch.iw_max_step = curr_iw_max_step();
}

static void BoardWatch_Delay(void)
{
    for (volatile uint32_t i = 0U; i < 200000U; i++)
    {
    }
}
