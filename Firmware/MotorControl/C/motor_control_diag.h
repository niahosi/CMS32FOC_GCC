#pragma once

#include "motor_control_internal.h"

void MotorControlDiag_Init(void);
void MotorControlDiag_ResetForMode(uint8_t mode);
void MotorControlDiag_RunFastLoop(MotorControlCState* mc);
void MotorControlDiag_FillWatch(const MotorControlCState* mc, MotorControlDiagWatch_t* out);
