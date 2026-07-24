/**
 * @file motor_controller.hpp
 * @author setsuna
 * @brief MotorControl C++ owner object and Ozone-visible runtime state
 * @version 0.1
 * @date 2026-07-23
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>

#include "MotorControl.h"
#include "command_sanitizer.hpp"
#include "config.hpp"
#include "debug_state.hpp"
#include "foc_math.h"
#include "low_pass.hpp"
#include "motor_control_state.h"
#include "slew_limiter.hpp"
#include "speed_math.hpp"
#include "types.hpp"

namespace cms32::motor
{

struct MCSpeedLoopState
{
    /** 当前选用速度反馈，编码器电角 count/s。 */
    int32_t feedback{0};
    /** 速度环内部斜坡后的目标，编码器电角 count/s。 */
    cms32::support::SlewLimiter<int32_t, SpeedMath::ramp_step()> ref_active{};
    /** raw 差分速度反馈滤波器内部状态，编码器电角 count/s。 */
    cms32::support::LowPassI32<SpeedLoopConfig::filter_shift> feedback_filter{};
    /** 速度环 rpm 误差。 */
    int16_t err_rpm{0};
    /** 速度环斜率限制前的 q 轴电流目标。 */
    int16_t iq_target{0};
    /** 速度目标产生的 q 轴电流前馈。 */
    int16_t iq_ff{0};
    /** 速度环输出的 q 轴电流给定。 */
    cms32::support::SlewLimiter<int16_t, SpeedLoopConfig::iq_slew_step> iq_ref{};
    /** 模式切入后速度估算空白窗口计数。 */
    uint8_t startup_blank{0U};
    /** 速度估算/速度环分频计数。 */
    uint16_t sample_div{0U};
    /** 速度环 PI。 */
    FocPi_t pi{};
};

struct MCCurrentLoopState
{
    /** 最新三相电流。 */
    FocPhaseCurrent_t phase{0, 0, 0};
    /** Park 后 dq 电流。 */
    FocDq_t dq{0, 0};
    /** 当前实际 d 轴给定。 */
    cms32::support::SlewLimiter<int16_t, CurrentLoopConfig::ref_ramp_step>
        id_ref_active{};
    /** 当前实际 q 轴给定。 */
    cms32::support::SlewLimiter<int16_t, CurrentLoopConfig::ref_ramp_step>
        iq_ref_active{};
    /** 快环分频计数，用于降低电流环执行频率。 */
    uint16_t loop_div{0U};
    /** d 轴电流 PI。 */
    FocPi_t pi_d{};
    /** q 轴电流 PI。 */
    FocPi_t pi_q{};
};

struct MCPositionLoopState
{
    /** 位置目标，单位 encoder count，对应 encoder.pos。 */
    int32_t target{0};
    /** 位置反馈，单位 encoder count。 */
    int32_t feedback{0};
    /** 位置误差，target - feedback，单位 encoder count。 */
    int32_t error{0};
    /** 位置环 P 输出速度，单位 encoder count/s。 */
    int32_t speed_ref{0};
    /** 位置环 P 输出速度，单位 mechanical rpm。 */
    int16_t speed_ref_rpm{0};
    /** 按剩余距离和制动加速度计算的最大允许速度，单位 mechanical rpm。 */
    int16_t brake_speed_limit_rpm{0};
    /** 位置误差进入 deadband 后置 1。 */
    uint8_t at_target{0U};
};

struct MCVfState
{
    uint32_t open_loop_ticks{0U};
    uint32_t open_loop_reset_count{0U};
    uint32_t open_loop_timeout_ticks{0U};
    int32_t open_loop_theta_acc{0};
    uint16_t open_loop_theta{0U};
};

class MotorController final
{
public:
    MotorController() = default;
    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;
    MotorController(MotorController&&) = delete;
    MotorController& operator=(MotorController&&) = delete;

    MCRuntimeState_t runtime{};
    MCCommandCache_t command{};
    MCEncoderState_t encoder{};
    MCPositionLoopState position{};
    MCSpeedLoopState speed{};
    MCCurrentLoopState current{};
    MCOutputState_t output{};
    MotorControlCheck_t check{};
    MotorControlDiag_t diag{};
    MotorControlDebugState debug{};
    MCVfState vf{};

    void init() noexcept;
    void applyCommand(const volatile MotorControlCommand_t& command) noexcept;
    void runSlowLoop() noexcept;
    uint8_t fastLoopFromAdcIrq() noexcept;

    void publishDebugState() noexcept;
    void currentReset() noexcept;
    void speedReset() noexcept;
    void speedControllerReset() noexcept;
    void encoderReset() noexcept;
    void positionReset() noexcept;
    void runCurrentFastLoop(ControlMode mode) noexcept;
    uint8_t updateEncoderAngle() noexcept;
    uint8_t updateEncoderSpeed() noexcept;
    int16_t speedCountsToRpm(int32_t speed_counts) const noexcept;
    uint8_t currentOk() const noexcept;
    int16_t voltageLimit() const noexcept;
    void applyVoltageVector(int16_t vd, int16_t vq, uint16_t theta) noexcept;
    void enterSafeState() noexcept;

    void vfInit() noexcept;
    void vfResetForMode(uint8_t mode) noexcept;
    void vfRunFastLoop() noexcept;
    uint16_t vfOpenLoopTheta() const noexcept;
    uint32_t vfOpenLoopTicks() const noexcept;
    uint32_t vfOpenLoopResetCount() const noexcept;

private:
    ControlMode currentMode() const noexcept;
    ControlState currentState() const noexcept;
    void setState(ControlState state) noexcept;
    void setFault(ControlFault fault) noexcept;
    void setMode(ControlMode mode) noexcept;
    bool readyForMode(ControlMode mode) const noexcept;
    ControlFault faultForNotReady(ControlMode mode) const noexcept;
    bool safeStateRequired() const noexcept;
    void resetForModeChange(ControlMode next_mode) noexcept;
    void applyVfVoltageMirror(const MotorControlCommand_t& command) noexcept;
    void enterIdleIfDisabled() noexcept;
    void refreshSlowChecks(ControlMode mode) noexcept;
    void enterReadyState(ControlMode mode) noexcept;
    void enterFaultState(ControlFault fault) noexcept;
    void applyRuntimeCommands(const CommandSanitizer& sanitizer,
                              const MotorControlCommand_t& command) noexcept;
    void updatePositionLoop() noexcept;
    void updateSpeedLoop() noexcept;
};

extern MotorController g_motor;

void publish_debug_state() noexcept;

} // namespace cms32::motor
