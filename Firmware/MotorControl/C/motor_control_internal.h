#pragma once

#include <stdint.h>

#include "Config.h"
#include "MotorControl.h"
#include "foc_math.h"

#define MC_STATE_IDLE 0U
#define MC_STATE_CLOSED_LOOP 3U
#define MC_STATE_FAULT 4U

#define MC_MODE_OFF 0U
#define MC_MODE_CURRENT 1U
#define MC_MODE_SPEED 2U
#define MC_MODE_VF_OPEN_LOOP 3U
#define MC_MODE_ALIGN_LOCK 4U
#define MC_MODE_ENCODER_VOLTAGE 5U

#define MC_FAULT_NONE 0U
#define MC_FAULT_UNSUPPORTED_MODE 1U
#define MC_FAULT_CURRENT 2U
#define MC_FAULT_OPEN_LOOP_TIMEOUT 3U
#define MC_FAULT_ENCODER 4U

#define MC_CURRENT_HZ (PWM_FREQ_HZ / CTRL_FAST_LOOP_DIV)
#define MC_SPEED_SAMPLE_DIV (MC_CURRENT_HZ / CTRL_SPD_EST_HZ)
#define MC_SPEED_COUNTS_PER_REV ((int32_t)MOT_SENSOR_CPR * (int32_t)MOT_SENSOR_POLE_PAIRS)

typedef struct
{
    uint8_t state;
    uint8_t fault;
    uint8_t enabled;
    uint8_t mode;
    uint8_t pwm_output;
    uint32_t command_apply_count;
    uint32_t slow_loop_count;
    uint32_t fast_loop_count;
    uint16_t current_loop_div;
    uint16_t speed_sample_div;
    uint16_t encoder_raw;
    uint16_t encoder_elec;
    uint16_t encoder_prev_raw;
    int16_t encoder_delta;
    int32_t encoder_pos;
    int16_t encoder_raw_step;
    int16_t encoder_reject_step;
    uint16_t encoder_reject_prev_raw;
    uint16_t encoder_reject_raw;
    uint32_t encoder_reject_count;
    uint32_t encoder_retry_count;
    uint32_t encoder_retry_accept_count;
    uint16_t encoder_retry_raw;
    int32_t speed_fb;
    int32_t speed_fb_diff;
    int32_t speed_fb_ma600;
    int32_t speed_diff_accum;
    int16_t ma600_speed_raw;
    int16_t speed_err_rpm;
    int16_t speed_iq_ref;
    uint8_t speed_diff_count;
    uint8_t encoder_age;
    uint8_t encoder_ok;
    uint8_t encoder_initialized;
    FocPi_t speed_pi;
    FocPi_t current_pi_d;
    FocPi_t current_pi_q;
    MotorControlCommand_t command;
    FocPhaseCurrent_t current;
    FocDq_t current_dq;
    int16_t id_ref_active;
    int16_t iq_ref_active;
    FocAlphaBeta_t voltage_ab;
    FocDq_t voltage_dq;
    uint16_t voltage_theta;
    FocDuty_t duty;
    uint8_t voltage_limited;
    MotorControlCheck_t check;
} MotorControlCState;

uint8_t MotorControl_InternalCurrentOk(MotorControlCState* mc);
uint8_t MotorControl_InternalUpdateEncoderAngle(MotorControlCState* mc);
uint8_t MotorControl_InternalUpdateEncoderSpeed(MotorControlCState* mc);
void MotorControl_InternalApplyVoltageVector(MotorControlCState* mc, int16_t vd, int16_t vq,
                                             uint16_t theta);
void MotorControl_InternalEnterSafeState(MotorControlCState* mc);
int16_t MotorControl_InternalSpeedCountsToRpm(int32_t speed);
