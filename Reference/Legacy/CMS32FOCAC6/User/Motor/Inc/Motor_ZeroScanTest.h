/**
 * @file Motor_ZeroScanTest.h
 * @brief 临时电角度零点扫描测试接口。
 * @details 该模块只用于 bring-up 阶段定位电流环零给定时的最佳电角度零点。
 *          调通后可以整体删除，不属于最终闭环控制主线。
 */

#pragma once

#include <stdint.h>
#include "Motor.h"

typedef enum
{
    MOTOR_ZERO_SCAN_TEST_IDLE = 0u,
    MOTOR_ZERO_SCAN_TEST_WAIT_CLOSED_LOOP,
    MOTOR_ZERO_SCAN_TEST_SETTLE,
    MOTOR_ZERO_SCAN_TEST_SAMPLE,
    MOTOR_ZERO_SCAN_TEST_DONE,
    MOTOR_ZERO_SCAN_TEST_FAULT
} MotorZeroScanTestState_t;

typedef enum
{
    MOTOR_ZERO_SCAN_TEST_FAULT_NONE = 0u,
    MOTOR_ZERO_SCAN_TEST_FAULT_STOP_REQ,
    MOTOR_ZERO_SCAN_TEST_FAULT_BAD_PARAM,
    MOTOR_ZERO_SCAN_TEST_FAULT_NOT_CLOSED_LOOP,
    MOTOR_ZERO_SCAN_TEST_FAULT_NOT_CURRENT_MODE,
    MOTOR_ZERO_SCAN_TEST_FAULT_MOTOR_FAULT,
    MOTOR_ZERO_SCAN_TEST_FAULT_V_LIMIT
} MotorZeroScanTestFault_t;

typedef enum
{
    MOTOR_ZERO_SCAN_TEST_PHASE_POS = 0u,
    MOTOR_ZERO_SCAN_TEST_PHASE_NEG = 1u
} MotorZeroScanTestPhase_t;

typedef struct
{
    uint8_t enable;
    uint8_t start_req;
    uint8_t stop_req;
    int16_t iq_ref;
    int32_t start_trim;
    int32_t end_trim;
    int32_t step;
    uint16_t settle_ticks;
    uint16_t sample_ticks;
} MotorZeroScanTestCmd_t;

typedef struct
{
    MotorZeroScanTestState_t state;
    MotorZeroScanTestPhase_t phase;
    int16_t iq_ref;
    int32_t trim;
    int32_t best_trim;
    int32_t score;
    int32_t best_score;
    int16_t id_avg;
    int16_t iq_avg;
    int16_t iq_err_avg;
    int16_t vq_avg;
    int16_t best_id_avg;
    int16_t best_iq_err_avg;
    int16_t best_vq_avg;
    MotorZeroScanTestFault_t fault;
} MotorZeroScanTestSnap_t;

void Motor_ZeroScanTestInit(void);
void Motor_ZeroScanTestTask(const volatile MotorZeroScanTestCmd_t* cmd);
uint8_t Motor_ZeroScanTestIsActive(void);
int32_t Motor_ZeroScanTestGetTrim(void);
int16_t Motor_ZeroScanTestGetIqRef(void);
void Motor_ZeroScanTestFillSnap(MotorZeroScanTestSnap_t* snap);
