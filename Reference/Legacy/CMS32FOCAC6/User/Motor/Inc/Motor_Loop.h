/**
 * @file Motor_Loop.h
 * @brief 电机闭环控制模块接口
 * @details 提供电流环、速度估算、速度环和闭环前对齐等功能。
 *          此模块负责实际的 FOC 闭环计算，由 Motor_FastLoop() 调用。
 */

#pragma once
#include <stdint.h>
#include "Motor.h"

/** @brief 闭环控制返回故障码。 */
typedef enum
{
    MOTOR_LOOP_FAULT_NONE = 0u,      /**< 无故障 */
    MOTOR_LOOP_FAULT_MA600,          /**< MA600 检查失败 */
    MOTOR_LOOP_FAULT_ANGLE,          /**< 角度缓存过期 */
    MOTOR_LOOP_FAULT_CURRENT,        /**< 运行中过流 */
    MOTOR_LOOP_FAULT_OL_TIMEOUT      /**< VF/IF 开环超时 */
} MotorLoopFault_t;

/**
 * @brief 初始化闭环控制模块
 * @details 清除所有闭环状态和 PI 控制器，应在 Motor_Init() 中调用
 */
void Motor_LoopInit(void);

/**
 * @brief 运行闭环控制主函数
 * @details 根据 ctrl_mode 执行对应的控制路径：
 *          - MOTOR_CTRL_CURRENT：正式电流环
 *          - MOTOR_CTRL_SPEED：速度环 + 电流环
 * @param[in] ctrl_mode 当前控制模式
 * @param[in] ma600_ok MA600 传感器是否正常
 * @return MotorLoopFault_t 闭环故障码。
 */
MotorLoopFault_t Motor_LoopRun(uint8_t ctrl_mode, uint8_t ma600_ok);

/**
 * @brief 运行转子对齐循环
 * @details 在闭环前输出固定 d 轴电压将转子拉到已知角度。
 *          由 Motor_TASK 在 MOTOR_STATE_ALIGN 状态下调用。
 * @return 1 表示对齐完成，0 表示仍在对齐中
 */
uint8_t Motor_LoopRunAlign(void);

/**
 * @brief Setter: 设置速度环速度给定
 * @param[in] speed_ref 速度给定值（sensor counts/s）
 */
void Motor_LoopSetSpeedRef(int32_t speed_ref);

/**
 * @brief Setter: 设置速度环 iq 限幅
 * @param[in] iq_limit iq 限幅值（正值）
 */
void Motor_LoopSetIqLimit(int16_t iq_limit);

/**
 * @brief Setter: 设置电流环电流给定
 * @param[in] id_ref d 轴电流给定
 * @param[in] iq_ref q 轴电流给定
 */
void Motor_LoopSetCurrentRef(int16_t id_ref, int16_t iq_ref);

/**
 * @brief Setter: 设置电流环 PI 参数。
 * @param[in] kp 比例系数。
 * @param[in] ki 积分系数。
 * @param[in] v_limit d/q 电压输出限幅，单位 PWM count。
 */
void Motor_LoopSetCurrentPi(int16_t kp, int16_t ki, int16_t v_limit);

/**
 * @brief Setter: 设置输出就绪标志
 * @param[in] ready 1 表示输出就绪，0 表示不输出
 */
void Motor_LoopSetOutputReady(uint8_t ready);

/**
 * @brief Getter: 获取输出就绪标志
 * @return 1 表示输出就绪，0 表示不输出
 */
uint8_t Motor_LoopGetOutputReady(void);

/**
 * @brief Getter: 获取闭环活跃标志
 * @return 1 表示闭环正在运行，0 表示空闲
 */
uint8_t Motor_LoopGetActive(void);

/**
 * @brief 填充运行快照结构体
 * @details 由 Motor_GetRunSnap() 调用
 * @param[out] snap 运行快照结构体指针
 */
void Motor_LoopFillRunSnap(MotorRunSnap_t* snap);
