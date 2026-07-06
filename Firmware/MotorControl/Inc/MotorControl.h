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
    int16_t speed_ref_rpm;
    int16_t iq_limit;
    int16_t current_kp;
    int16_t current_ki;
    int16_t speed_kp;
    int16_t speed_ki;
    int16_t current_v_limit;
    int32_t open_loop_speed_ref;
    int16_t vf_voltage;
    int16_t if_id_ref;
    int16_t if_iq_ref;
    uint16_t open_loop_timeout_ms;
    int16_t elec_zero_trim;
    int16_t voltage_theta_offset;
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
    uint8_t align_done;
    uint32_t align_ticks;
    uint16_t align_theta;
    uint16_t align_raw;
    int16_t align_zero_trim;
    uint16_t align_encoder_elec;
    uint8_t align_stage;
    int16_t align_pull_delta;
    uint16_t align_sample_count;
    int32_t align_delta_sum;
    int16_t iu_cnt;
    int16_t iv_cnt;
    int16_t iw_cnt;
    int16_t i_sum;
    int16_t id_ref;
    int16_t iq_ref;
    int32_t speed_ref;
    int16_t speed_ref_rpm;
    int32_t speed_fb;
    int16_t speed_fb_rpm;
    int32_t speed_fb_diff;
    int16_t speed_fb_diff_rpm;
    int32_t speed_fb_ma600;
    int16_t speed_fb_ma600_rpm;
    int16_t ma600_speed_raw;
    uint8_t speed_fb_source;
    int16_t speed_err_rpm;
    int16_t speed_iq_cmd;
    int32_t speed_pi_integral;
    int16_t id;
    int16_t iq;
    int16_t vd;
    int16_t vq;
    uint16_t voltage_theta;
    uint8_t v_limited;
    uint16_t duty_u;
    uint16_t duty_v;
    uint16_t duty_w;
    uint8_t sample_pair;
    uint8_t sample_hold;
    uint16_t sample_hold_count;
    uint16_t sample_common_window;
    uint32_t sample_switch_count;
    int16_t sample_center_bias;
    uint16_t sample_tick_a;
    uint16_t sample_tick_b;
    uint8_t sample_single_point;
    uint8_t sample_region;
    uint8_t sample_recon_mode;
    uint8_t sample_valid_mask;
    uint8_t sample_bad_count;
    uint16_t sample_t1;
    uint16_t sample_t2;
    uint16_t sample_t3;
    int16_t sample_spread0;
    int16_t sample_spread1;
    uint32_t iv_spike_count;
    uint32_t iw_spike_count;
    uint16_t iv_max_step;
    uint16_t iw_max_step;
    uint8_t pwm_safe;
    uint8_t pwm_running;
    MotorControlCheck_t check;
    uint32_t command_apply_count;
    uint8_t command_enable;
    uint8_t command_control_mode;
    int16_t command_vf_voltage;
    int32_t command_open_loop_speed_ref;
    int16_t command_speed_ref_rpm;
    int16_t command_iq_limit;
    int16_t command_current_v_limit;
    int16_t command_voltage_theta_offset;
    int16_t command_speed_kp;
    int16_t command_speed_ki;
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
