/**
 * @file motor_command_sanitizer.hpp
 * @author setsuna
 * @brief 命令快照和命令限幅
 * @version 0.1
 * @date 2026-07-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "BoardConfig.h"
#include "MotorControl.h"
#include "TuneConfig.h"
#include "motor_control_internal.h"

#include "clamp.hpp"
#include "units.hpp"
#include <cstdint>

namespace cms32::motor
{
class CommandSanitizer
{
public:
    MotorControlCommand_t snapshot(const volatile MotorControlCommand_t& input) const noexcept
    {
        MotorControlCommand_t out{};

        out.enable = input.enable;
        out.control_mode = input.control_mode;
        out.id_ref = input.id_ref;
        out.iq_ref = input.iq_ref;
        out.speed_ref = input.speed_ref;
        out.speed_ref_rpm = input.speed_ref_rpm;
        out.iq_limit = input.iq_limit;
        out.current_kp = input.current_kp;
        out.current_ki = input.current_ki;
        out.speed_kp = input.speed_kp;
        out.speed_ki = input.speed_ki;
        out.current_v_limit = input.current_v_limit;
        out.open_loop_speed_ref = input.open_loop_speed_ref;
        out.vf_voltage = input.vf_voltage;
        out.if_id_ref = input.if_id_ref;
        out.if_iq_ref = input.if_iq_ref;
        out.open_loop_timeout_ms = input.open_loop_timeout_ms;
        out.elec_zero_trim = input.elec_zero_trim;
        out.voltage_theta_offset = input.voltage_theta_offset;
        return out;
    }

    MotorControlCommand_t sanitize(MotorControlCommand_t command) const noexcept
    {
        command.iq_limit = positive_limit(command.iq_limit, CTRL_CUR_REF_LIMIT);
        command.current_kp = clamp_gain(command.current_kp);
        command.current_ki = clamp_gain(command.current_ki);
        command.speed_kp = clamp_gain(command.speed_kp);
        command.speed_ki = clamp_gain(command.speed_ki);
        command.current_v_limit =
            positive_limit(command.current_v_limit, static_cast<int16_t>(PWM_SVPWM_V_LIMIT));
        command.id_ref = symmetric_limit(command.id_ref, CTRL_CUR_REF_LIMIT);
        command.iq_ref = symmetric_limit(command.iq_ref, command.iq_limit);
        command.open_loop_speed_ref = cms32::support::clamp<int32_t>(
            command.open_loop_speed_ref, -CTRL_SPD_REF_LIMIT, CTRL_SPD_REF_LIMIT);

        if (command.speed_ref_rpm != 0)
        {
            command.speed_ref = cms32::support::rpm_to_speed_counts<MC_SPEED_COUNTS_PER_REV>(
                                    cms32::support::Rpm{command.speed_ref_rpm})
                                    .value;
        }

        command.speed_ref =
            cms32::support::clamp<int32_t>(command.speed_ref, -CTRL_SPD_REF_LIMIT, CTRL_SPD_REF_LIMIT);
        command.vf_voltage = symmetric_limit(command.vf_voltage, CTRL_CUR_V_LIMIT);

        return command;
    }

    MCCurrentCommand_t current_command(const MotorControlCommand_t& command) const noexcept
    {
        return MCCurrentCommand_t{command.id_ref,
                                            command.iq_ref,
                                            command.iq_limit,
                                            command.current_kp,
                                            command.current_ki,
                                            command.current_v_limit,
                                            command.elec_zero_trim,
                                            command.voltage_theta_offset};
    }

    MCSpeedCommand_t speed_command(const MotorControlCommand_t& command) const noexcept
    {
        return MCSpeedCommand_t{command.speed_ref,
                                          command.speed_kp,
                                          command.speed_ki,
                                          command.iq_limit};
    }

    MCVfCommand_t vf_command(const MotorControlCommand_t& command) const noexcept
    {
        return MCVfCommand_t{command.open_loop_speed_ref,
                                       command.vf_voltage,
                                       command.open_loop_timeout_ms};
    }

private:
    static constexpr int16_t clamp_gain(int16_t value) noexcept
    {
        return cms32::support::clamp<int16_t>(value, 0, 32767);
    }

    static constexpr int16_t symmetric_limit(int16_t value, int16_t limit) noexcept
    {
        return cms32::support::clamp<int16_t>(value, static_cast<int16_t>(-limit), limit);
    }

    static constexpr int16_t positive_limit(int16_t value, int16_t limit) noexcept
    {
        const int32_t magnitude = (value < 0) ? -static_cast<int32_t>(value) : value;
        return static_cast<int16_t>(cms32::support::clamp<int32_t>(magnitude, 0, limit));
    }
};
} // namespace cms32::motor
