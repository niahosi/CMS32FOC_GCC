/**
 * @file motor_control_config.hpp
 * @author setsuna
 * @brief 当前参数镜像和编译期检查
 * @version 0.1
 * @date 2026-07-17
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <stdint.h>
#include "BoardConfig.h"
#include "Config.h"
#include "TuneConfig.h"

namespace cms32::motor
{
struct CurrentLoopConfig
{
    static constexpr int16_t ref_limit = CTRL_CUR_REF_LIMIT;
    static constexpr int16_t voltage_limit = CTRL_CUR_V_LIMIT;
    static constexpr uint8_t pi_shift = CTRL_CUR_PI_SHIFT;
    static constexpr int16_t kp = CTRL_CUR_KP;
    static constexpr int16_t ki = CTRL_CUR_KI;
    static constexpr int16_t ref_ramp_step = CTRL_CUR_REF_RAMP_STEP;
};
struct SpeedLoopConfig
{
    static constexpr int32_t estimate_hz = CTRL_SPD_EST_HZ;
    static constexpr uint8_t startup_blank_samples = CTRL_SPD_STARTUP_BLANK_SAMPLES;
    static constexpr int16_t kp = CTRL_SPD_KP;
    static constexpr int16_t ki = CTRL_SPD_KI;
    static constexpr uint8_t err_shift = CTRL_SPD_ERR_SHIFT;
    static constexpr uint8_t filter_shift = CTRL_SPD_FILTER_SHIFT;
    static constexpr int16_t command_deadband_rpm = CTRL_SPD_CMD_DEADBAND_RPM;
    static constexpr int32_t ref_ramp_rpm_per_s = CTRL_SPD_REF_RAMP_RPM_PER_S;
    static constexpr int32_t ref_limit_rpm = CTRL_SPD_REF_LIMIT_RPM;
    static constexpr int16_t iq_limit = CTRL_SPD_IQ_LIMIT;
    static constexpr int16_t iq_slew_step = CTRL_SPD_IQ_SLEW_STEP;
};

struct EncoderConfig
{
    // 65536 * 4
    static constexpr int32_t counts_per_rev =
        static_cast<int32_t>(MOT_SENSOR_CPR) * static_cast<int32_t>(MOT_SENSOR_POLE_PAIRS);
    static constexpr int8_t direction = MOT_SENSOR_DIR; // 1 or -1
    static constexpr int16_t elec_zero = MOT_ELEC_ZERO; //-13478
};

static_assert(CurrentLoopConfig::pi_shift < 15U, "current PI shift too large");
static_assert(SpeedLoopConfig::err_shift < 15U, "speed PI shift too large");
static_assert(EncoderConfig::counts_per_rev > 0, "invalid encoder scale");
static_assert((EncoderConfig::direction == 1) || (EncoderConfig::direction == -1),
              "MOT_SENSOR_DIR must be 1 or -1");

} // namespace cms32::motor