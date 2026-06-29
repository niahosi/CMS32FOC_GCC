/**
 * @file App_Debug.h
 * @brief 应用层调试命令和观察快照接口。
 */

#pragma once

#include <stdint.h>
#include "Motor.h"
#include "Motor_ZeroScanTest.h"

typedef struct
{
    uint8_t motor_enable;
    uint8_t ctrl_mode;
} AppCmdRun_t;

typedef enum
{
    APP_CURRENT_SAMPLE_PAIR_UV = 0u,
    APP_CURRENT_SAMPLE_PAIR_UW = 1u,
    APP_CURRENT_SAMPLE_PAIR_VW = 2u,
    APP_CURRENT_SAMPLE_PAIR_NONE = 255u
} AppCurrentSamplePair_t;

typedef struct
{
    int16_t id_ref;
    int16_t iq_ref;
    int16_t kp;
    int16_t ki;
    int16_t v_limit;
} AppCmdCurrent_t;

typedef struct
{
    int32_t speed_ref;
    int16_t iq_limit;
} AppCmdSpeed_t;

typedef struct
{
    int32_t elec_zero_trim;
} AppCmdAngle_t;

typedef struct
{
    int32_t ol_speed_ref;
    int16_t vf_voltage;
    int16_t if_id_ref;
    int16_t if_iq_ref;
    uint16_t ol_timeout_ms;
} AppCmdOpenLoop_t;

typedef struct
{
    uint8_t rtt_enable;
} AppCmdDebug_t;

/**
 * @brief 用户可写调试命令。
 * @details Watch/JScope 中只向此结构体写命令值，不直接改观察结构体。
 */
typedef struct
{
    AppCmdRun_t run;
    AppCmdCurrent_t current;
    AppCmdSpeed_t speed;
    AppCmdAngle_t angle;
    AppCmdOpenLoop_t open_loop;
    MotorZeroScanTestCmd_t zero_scan_test;
    AppCmdDebug_t debug;
} AppCmd_t;

typedef struct
{
    MotorState_t state;
    MotorControlMode_t ctrl_mode;
    uint8_t enable;
    MotorFaultReason_t fault_reason;
} AppElemMotor_t;

typedef struct
{
    uint16_t raw;
    uint16_t elec;
    int32_t pos;
    int16_t delta;
} AppElemAngle_t;

typedef struct
{
    int16_t iu;
    int16_t iv;
    int16_t iw;
    int16_t sum;
    uint16_t iu_raw_adc;
    uint16_t iv_raw_adc;
    uint16_t iw_raw_adc;
    int16_t iu_raw_cnt;
    int16_t iv_raw_cnt;
    int16_t iw_raw_cnt;
    int16_t raw_sum;
    uint32_t adc_sync_count;
    uint16_t adc_trigger_tick;
    uint8_t sample_dynamic_mode;
    AppCurrentSamplePair_t sample_pair;
    uint16_t sample_center_tick;
    uint16_t sample_diag_tick_a;
    uint16_t sample_diag_tick_b;
    uint16_t sample_window_u;
    uint16_t sample_window_v;
    uint16_t sample_window_w;
    uint8_t sample_hold;
    uint16_t sample_hold_count;
    uint8_t sample_diag_stage;
    uint8_t sample_diag_count;
    int16_t sample_a_first;
    int16_t sample_a_second;
    int16_t sample_b_first;
    int16_t sample_b_second;
    int16_t sample_spread_first;
    int16_t sample_spread_second;
} AppElemCurrent_t;

typedef struct
{
    int16_t id_ref;
    int16_t iq_ref;
    int16_t id;
    int16_t iq;
    int16_t vd;
    int16_t vq;
    int16_t kp;
    int16_t ki;
    int16_t v_limit;
    uint8_t v_limited;
} AppElemFoc_t;

typedef struct
{
    uint16_t duty_u;
    uint16_t duty_v;
    uint16_t duty_w;
    uint8_t output_on;
    uint8_t brake_on;
} AppElemPwm_t;

typedef struct
{
    int32_t ref;
    int32_t fb;
} AppElemSpeed_t;

typedef struct
{
    uint8_t sensor_ma600_ok;
    uint8_t sensor_current_ok;
    uint8_t current_over_count;
} AppElemSafety_t;

typedef struct
{
    uint16_t theta;
    int16_t vf_voltage;
    int16_t if_id_ref;
    int16_t if_iq_ref;
    uint8_t current_over_count;
} AppElemOpenLoop_t;

/**
 * @brief 运行观察快照。
 * @details Watch/JScope 中默认只读此结构体，用于观察状态和调试结果。
 */
typedef struct
{
    AppElemMotor_t motor;
    AppElemAngle_t angle;
    AppElemCurrent_t current;
    AppElemFoc_t foc;
    AppElemPwm_t pwm;
    AppElemSpeed_t speed;
    AppElemSafety_t safety;
    AppElemOpenLoop_t open_loop;
    MotorZeroScanTestSnap_t zero_scan_test;
} AppElementary_t;

/** @brief 用户可写命令入口。 */
extern volatile AppCmd_t g_cmd;

/** @brief 用户只读观察入口。 */
extern volatile AppElementary_t g_elementary;

/**
 * @brief 初始化调试命令和 RTT 输出状态。
 */
void Debug_InitRtt(void);

/**
 * @brief 更新运行观察快照。
 */
void Debug_UpdateElementary(void);

/**
 * @brief RTT 周期输出任务。
 */
void Debug_RttTask(void);
