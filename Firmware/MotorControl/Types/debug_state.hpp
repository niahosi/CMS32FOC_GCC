/**
 * @file debug_state.hpp
 * @author setsuna
 * @brief MotorControl enum debug state for Ozone
 * @version 0.1
 * @date 2026-07-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "types.hpp"

namespace cms32::motor
{

struct MotorControlDebugState
{
    /** Ozone 友好的枚举/布尔摘要，不参与闭环控制。 */
    ControlState state{ControlState::Idle};
    ControlMode mode{ControlMode::Off};
    ControlFault fault{ControlFault::None};
    bool enabled{false};
    bool pwm_output{false};
    /** 命令缓存里的目标速度，单位 mechanical rpm。 */
    int16_t speed_ref_cmd_rpm{0};
    /** 速度斜坡后的当前目标，单位 mechanical rpm。 */
    int16_t speed_ref_active_rpm{0};
    /** 当前速度反馈，单位 mechanical rpm。 */
    int16_t speed_fb_rpm{0};
    /** 速度环误差，单位 mechanical rpm。 */
    int16_t speed_err_rpm{0};
    /** 位置目标，单位 encoder count。 */
    int32_t position_ref{0};
    /** 位置误差，单位 encoder count。 */
    int32_t position_error{0};
    /** 位置环输出速度，单位 mechanical rpm。 */
    int16_t position_speed_ref_rpm{0};
    /** 位置环是否在 deadband 内。 */
    bool position_at_target{false};
};

void publish_debug_state() noexcept;

} // namespace cms32::motor
