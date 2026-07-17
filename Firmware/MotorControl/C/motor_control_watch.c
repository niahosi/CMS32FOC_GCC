#include "MotorControl.h"
#include "motor_control_internal.h"
#include "motor_control_vf.h"

#include "foc_curr.h"
#include "foc_pwm.h"
#include <stdint.h>

/** @brief 填充 Current/Speed/VF 主线 watch 快照。 */
void MotorControl_WatchFill(const MotorControlCState* mc, MotorControlWatch_t* out)
{
    volatile uint16_t duty_u = 0U;
    volatile uint16_t duty_v = 0U;
    volatile uint16_t duty_w = 0U;
    volatile uint8_t pwm_out = 0U;
    volatile uint8_t pwm_brake = 0U;

    pwm_snapshot(&duty_u, &duty_v, &duty_w, &pwm_out, &pwm_brake);
    (void)pwm_brake;

    out->state = mc->state;
    out->control_mode = mc->mode;
    out->fault_reason = mc->fault;
    out->enable = mc->enabled;
    out->slow_loop_count = mc->slow_loop_count;
    out->fast_loop_count = mc->fast_loop_count;
    out->speed_reset_count = mc->speed_reset_count;
    out->safe_state_count = mc->safe_state_count;
    out->speed_loop_count = mc->speed_loop_count;
    out->speed_deadband_count = mc->speed_deadband_count;
    out->adc_sample_count = curr_sync_count();
    out->encoder_raw = mc->encoder_raw;
    out->encoder_elec = mc->encoder_elec;
    out->encoder_delta = mc->encoder_delta;
    out->encoder_pos = mc->encoder_pos;
    out->encoder_age = mc->encoder_age;
    out->encoder_ok = mc->encoder_ok;
    out->encoder_raw_step = mc->encoder_raw_step;
    out->encoder_reject_step = mc->encoder_reject_step;
    out->encoder_reject_count = mc->encoder_reject_count;
    out->encoder_retry_count = mc->encoder_retry_count;
    out->encoder_retry_accept_count = mc->encoder_retry_accept_count;
    out->encoder_reject_raw = mc->encoder_reject_raw;
    out->encoder_retry_raw = mc->encoder_retry_raw;
    out->encoder_prev_raw = mc->encoder_prev_raw;
    out->encoder_hold_count = mc->encoder_hold_count;
    out->speed_reject_count = mc->speed_reject_count;
    out->speed_reject_delta = mc->speed_reject_delta;
    out->iu_cnt = curr_u();
    out->iv_cnt = curr_v();
    out->iw_cnt = curr_w();
    out->i_sum = curr_sum();
    out->id_ref = mc->id_ref_active;
    out->iq_ref = mc->iq_ref_active;
    out->speed_ref = mc->speed_command.speed_ref;
    out->speed_ref_rpm = MotorControl_InternalSpeedCountsToRpm(mc->speed_command.speed_ref);
    out->speed_ref_active_rpm = MotorControl_InternalSpeedCountsToRpm(mc->speed_ref_active);
    out->speed_fb = mc->speed_fb;
    out->speed_fb_rpm = MotorControl_InternalSpeedCountsToRpm(mc->speed_fb);
    out->speed_err_rpm = mc->speed_err_rpm;
    out->speed_iq_target = mc->speed_iq_target;
    out->speed_iq_cmd = mc->speed_iq_ref;
    out->speed_pi_output = mc->speed_pi.output;
    out->speed_iq_ff = mc->speed_iq_ff;
    out->speed_pi_integral = mc->speed_pi.integral;
    out->id = mc->current_dq.d;
    out->iq = mc->current_dq.q;
    out->vd = mc->voltage_dq.d;
    out->vq = mc->voltage_dq.q;
    out->voltage_theta = mc->voltage_theta;
    out->open_loop_theta = MotorControlVf_OpenLoopTheta();
    out->open_loop_ticks = MotorControlVf_OpenLoopTicks();
    out->open_loop_reset_count = MotorControlVf_OpenLoopResetCount();
    out->vf_voltage = mc->vf_command.vf_voltage;
    out->v_limited = mc->voltage_limited;
    out->duty_u = (uint16_t)duty_u;
    out->duty_v = (uint16_t)duty_v;
    out->duty_w = (uint16_t)duty_w;
    out->pwm_safe = pwm_is_off_safe();
    out->pwm_running = (uint8_t)((pwm_out != 0U) && (pwm_is_running() != 0U));
    out->check = mc->check;
}

/** @brief 将主线 watch 快照逐字段写入 volatile 目标。 */
void MotorControl_WatchCopyToVolatile(volatile MotorControlWatch_t* dst,
                                      const MotorControlWatch_t* src)
{
    dst->state = src->state;
    dst->control_mode = src->control_mode;
    dst->fault_reason = src->fault_reason;
    dst->enable = src->enable;
    dst->slow_loop_count = src->slow_loop_count;
    dst->fast_loop_count = src->fast_loop_count;
    dst->speed_reset_count = src->speed_reset_count;
    dst->safe_state_count = src->safe_state_count;
    dst->speed_loop_count = src->speed_loop_count;
    dst->speed_deadband_count = src->speed_deadband_count;
    dst->adc_sample_count = src->adc_sample_count;
    dst->encoder_raw = src->encoder_raw;
    dst->encoder_elec = src->encoder_elec;
    dst->encoder_delta = src->encoder_delta;
    dst->encoder_pos = src->encoder_pos;
    dst->encoder_age = src->encoder_age;
    dst->encoder_ok = src->encoder_ok;
    dst->encoder_raw_step = src->encoder_raw_step;
    dst->encoder_reject_step = src->encoder_reject_step;
    dst->encoder_reject_count = src->encoder_reject_count;
    dst->encoder_retry_count = src->encoder_retry_count;
    dst->encoder_retry_accept_count = src->encoder_retry_accept_count;
    dst->encoder_reject_raw = src->encoder_reject_raw;
    dst->encoder_retry_raw = src->encoder_retry_raw;
    dst->encoder_prev_raw = src->encoder_prev_raw;
    dst->encoder_hold_count = src->encoder_hold_count;
    dst->speed_reject_count = src->speed_reject_count;
    dst->speed_reject_delta = src->speed_reject_delta;
    dst->iu_cnt = src->iu_cnt;
    dst->iv_cnt = src->iv_cnt;
    dst->iw_cnt = src->iw_cnt;
    dst->i_sum = src->i_sum;
    dst->id_ref = src->id_ref;
    dst->iq_ref = src->iq_ref;
    dst->speed_ref = src->speed_ref;
    dst->speed_ref_rpm = src->speed_ref_rpm;
    dst->speed_ref_active_rpm = src->speed_ref_active_rpm;
    dst->speed_fb = src->speed_fb;
    dst->speed_fb_rpm = src->speed_fb_rpm;
    dst->speed_err_rpm = src->speed_err_rpm;
    dst->speed_iq_target = src->speed_iq_target;
    dst->speed_iq_cmd = src->speed_iq_cmd;
    dst->speed_pi_output = src->speed_pi_output;
    dst->speed_iq_ff = src->speed_iq_ff;
    dst->speed_pi_integral = src->speed_pi_integral;
    dst->id = src->id;
    dst->iq = src->iq;
    dst->vd = src->vd;
    dst->vq = src->vq;
    dst->voltage_theta = src->voltage_theta;
    dst->open_loop_theta = src->open_loop_theta;
    dst->open_loop_ticks = src->open_loop_ticks;
    dst->open_loop_reset_count = src->open_loop_reset_count;
    dst->vf_voltage = src->vf_voltage;
    dst->v_limited = src->v_limited;
    dst->duty_u = src->duty_u;
    dst->duty_v = src->duty_v;
    dst->duty_w = src->duty_w;
    dst->pwm_safe = src->pwm_safe;
    dst->pwm_running = src->pwm_running;
    dst->check.ma600_ok = src->check.ma600_ok;
    dst->check.current_ok = src->check.current_ok;
    dst->check.pwm_off_safe = src->check.pwm_off_safe;
    dst->check.ready_closed_loop = src->check.ready_closed_loop;
}
