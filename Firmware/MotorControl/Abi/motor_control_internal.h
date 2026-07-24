#pragma once

#include <stdint.h>

#include "motor_control_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 检查内部状态中的三相电流是否处于安全范围。 */
uint8_t MotorControl_InternalCurrentOk(void);
/** @brief 清空编码器、速度反馈和编码器诊断状态。 */
void MotorControl_EncoderReset(void);
/** @brief 快环读取并接受一次编码器角度；读取失败时保持上一角度。 */
uint8_t MotorControl_InternalUpdateEncoderAngle(void);
/** @brief 按当前编码器 raw 更新速度反馈。 */
uint8_t MotorControl_InternalUpdateEncoderSpeed(void);
/** @brief 清空电流 PI 和当前电流给定斜坡状态。 */
void MotorControl_CurrentReset(void);
/** @brief 清空速度 PI、速度估算和编码器状态。 */
void MotorControl_SpeedReset(void);
/** @brief 运行 Current/Speed 主线快环；Current 也更新速度观察，Speed 额外运行速度 PI。
 */
void MotorControl_CurrentRunFastLoop(uint8_t speed_mode);
/** @brief 获取当前电流环/诊断电压限幅，命令未设置时回退默认值。 */
int16_t MotorControl_InternalVoltageLimit(void);
/** @brief 输出 dq 电压矢量，统一执行限幅、反 Park、SVPWM 和 PWM 使能。 */
void MotorControl_InternalApplyVoltageVector(int16_t vd, int16_t vq, uint16_t theta);
/** @brief 进入安全态，关闭 PWM 并清零输出相关状态。 */
void MotorControl_InternalEnterSafeState(void);

#ifdef __cplusplus
}
#endif
