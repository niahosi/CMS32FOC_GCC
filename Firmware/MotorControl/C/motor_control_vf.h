#pragma once

#include "motor_control_internal.h"

/** @brief 初始化 VF 应急开环状态。 */
void MotorControlVf_Init(void);
/**
 * @brief 按明确模式切换重置 VF 状态。
 *
 * VF 开环角只允许在切入 VF 时重置，运行中不得因慢环重入重置。
 */
void MotorControlVf_ResetForMode(uint8_t mode);
/** @brief 运行 VF 应急开环快环。 */
void MotorControlVf_RunFastLoop(MotorControlCState* mc);
/** @brief 获取 VF 开环角。 */
uint16_t MotorControlVf_OpenLoopTheta(void);
/** @brief 获取 VF 开环角累计 tick。 */
uint32_t MotorControlVf_OpenLoopTicks(void);
/** @brief 获取 VF 开环角重置次数。 */
uint32_t MotorControlVf_OpenLoopResetCount(void);
