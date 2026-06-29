#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t enable;
    uint8_t control_mode;
    int16_t id_ref;
    int16_t iq_ref;
    int32_t speed_ref;
    int16_t iq_limit;
    int16_t current_kp;
    int16_t current_ki;
    int16_t current_v_limit;
    int32_t open_loop_speed_ref;
    int16_t vf_voltage;
    int16_t if_id_ref;
    int16_t if_iq_ref;
    uint16_t open_loop_timeout_ms;
    int16_t elec_zero_trim;
} MotorControlCommand_t;

typedef struct
{
    uint8_t ma600_ok;
    uint8_t current_ok;
    uint8_t pwm_off_safe;
    uint8_t ready_closed_loop;
} MotorControlCheck_t;

typedef struct
{
    uint8_t state;
    uint8_t control_mode;
    uint8_t fault_reason;
    uint8_t enable;
    uint32_t slow_loop_count;
    uint32_t fast_loop_count;
    uint32_t adc_sample_count;
    uint16_t encoder_raw;
    uint16_t encoder_elec;
    int16_t encoder_delta;
    int32_t encoder_pos;
    uint8_t encoder_age;
    uint8_t encoder_ok;
    int16_t iu_cnt;
    int16_t iv_cnt;
    int16_t iw_cnt;
    int16_t i_sum;
    int16_t id_ref;
    int16_t iq_ref;
    int32_t speed_ref;
    int32_t speed_fb;
    int16_t id;
    int16_t iq;
    int16_t vd;
    int16_t vq;
    uint8_t v_limited;
    uint16_t duty_u;
    uint16_t duty_v;
    uint16_t duty_w;
    uint8_t sample_pair;
    uint8_t sample_three_shunt;
    uint8_t sample_hold;
    uint16_t sample_hold_count;
    uint16_t sample_pair_hold_left;
    uint16_t sample_common_window;
    uint32_t sample_switch_count;
    uint32_t sample_fallback_count;
    uint32_t iv_spike_count;
    uint32_t iw_spike_count;
    uint16_t iv_max_step;
    uint16_t iw_max_step;
    uint8_t pwm_safe;
    uint8_t pwm_running;
    MotorControlCheck_t check;
} MotorControlWatch_t;

void MotorControl_Init(void);
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command);
void MotorControl_RunSlowLoop(void);
uint8_t MotorControl_FastLoopFromAdcIrq(void);
void MotorControl_GetWatch(MotorControlWatch_t* out);
void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out);

#ifdef __cplusplus
}
#endif
