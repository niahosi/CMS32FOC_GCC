/**
 * @file Motor_OpenLoop.h
 * @brief 电机开环检测接口。
 * @details 提供 VF/IF 开环检测路径，用于相序、电流采样方向和电角度趋势验证。
 */

#pragma once

#include <stdint.h>
#include "Motor.h"

/** @brief 开环检测返回故障码。 */
typedef enum
{
    MOTOR_OPEN_LOOP_FAULT_NONE = 0u, /**< 无故障 */
    MOTOR_OPEN_LOOP_FAULT_ANGLE,     /**< 角度不可用 */
    MOTOR_OPEN_LOOP_FAULT_CURRENT,   /**< 运行中过流 */
    MOTOR_OPEN_LOOP_FAULT_TIMEOUT    /**< 开环运行超时 */
} MotorOpenLoopFault_t;

typedef struct
{
    uint16_t theta;
    int16_t vf_voltage;
    int16_t if_id_ref;
    int16_t if_iq_ref;
    uint8_t current_over_count;
} MotorOpenLoopSnap_t;

/**
 * @brief 初始化开环检测模块。
 */
void Motor_OpenLoopInit(void);

/**
 * @brief 运行开环检测。
 * @param[in] ctrl_mode 控制模式，支持 MOTOR_CTRL_VF 和 MOTOR_CTRL_IF。
 * @return MotorOpenLoopFault_t 开环检测故障码。
 */
MotorOpenLoopFault_t Motor_OpenLoopRun(uint8_t ctrl_mode);

/**
 * @brief Setter: 设置开环速度给定。
 * @param[in] speed_ref 开环速度给定，单位 sensor counts/s。
 */
void Motor_OpenLoopSetSpeedRef(int32_t speed_ref);

/**
 * @brief Setter: 设置 VF 电压幅值。
 * @param[in] voltage VF 电压幅值，单位 PWM count。
 */
void Motor_OpenLoopSetVfVoltage(int16_t voltage);

/**
 * @brief Setter: 设置 IF 开环 d/q 轴电流给定。
 * @param[in] id_ref d 轴电流给定，单位 ADC count。
 * @param[in] iq_ref q 轴电流给定，单位 ADC count。
 */
void Motor_OpenLoopSetIfCurrentRef(int16_t id_ref, int16_t iq_ref);

/**
 * @brief Setter: 设置 VF/IF 开环超时时间。
 * @param[in] timeout_ms 超时时间，单位 ms。
 */
void Motor_OpenLoopSetTimeoutMs(uint16_t timeout_ms);

/**
 * @brief Getter: 填充开环检测快照。
 * @param[out] snap 开环快照输出指针，传入 0 时函数直接返回。
 */
void Motor_OpenLoopFillSnap(MotorOpenLoopSnap_t* snap);
