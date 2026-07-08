#pragma once

#include "motor_control_internal.h"

/** @brief 初始化诊断模式私有状态。 */
void MotorControlDiag_Init(void);
/**
 * @brief 按明确模式切换重置诊断状态。
 *
 * VF 开环角只允许在切入 VF/Align 等显式模式切换时重置，运行中不得因慢环重入重置。
 */
void MotorControlDiag_ResetForMode(uint8_t mode);
/** @brief 运行 VF、Align 或 EncoderVoltage 诊断快环。 */
void MotorControlDiag_RunFastLoop(MotorControlCState* mc);
/** @brief 填充诊断 watch，不参与控制决策。 */
void MotorControlDiag_FillWatch(const MotorControlCState* mc, MotorControlDiagWatch_t* out);
