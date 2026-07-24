/**
 * @file config.hpp
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
struct PwmConfig
{
    static constexpr uint32_t frequency_hz = PWM_FREQ_HZ;
    static constexpr uint16_t period = PWM_PERIOD;
    static constexpr uint16_t duty_center = PWM_DUTY_50;
    static constexpr uint16_t duty_min = PWM_DUTY_MIN;
    static constexpr uint16_t duty_max = PWM_DUTY_MAX;
    static constexpr int16_t svpwm_voltage_limit =
        static_cast<int16_t>(PWM_SVPWM_V_LIMIT);
    static constexpr uint16_t adc_trigger_tick = PWM_ADC_TRIGGER_TICK_DEFAULT;
    static constexpr uint16_t deadtime_ticks = PWM_DEADTIME_TICKS;
};

struct FastLoopConfig
{
    static constexpr uint8_t divider = CTRL_FAST_LOOP_DIV;
    static constexpr uint32_t current_hz = PwmConfig::frequency_hz / divider;
};

struct CurrentLoopConfig
{
    static constexpr int16_t ref_limit = CTRL_CUR_REF_LIMIT;
    static constexpr int16_t voltage_limit = CTRL_CUR_V_LIMIT;
    static constexpr uint8_t pi_shift = CTRL_CUR_PI_SHIFT;
    static constexpr int16_t kp = CTRL_CUR_KP;
    static constexpr int16_t ki = CTRL_CUR_KI;
    static constexpr int16_t ref_ramp_step = CTRL_CUR_REF_RAMP_STEP;
    static constexpr int16_t safe_limit = CTRL_CUR_SAFE_LIMIT;
    static constexpr uint8_t over_limit_count = CTRL_CUR_OVER_LIMIT;
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
    static constexpr int32_t diff_spike_rpm = CTRL_SPD_DIFF_SPIKE_RPM;
    static constexpr int32_t pos_deadband = CTRL_SPD_POS_DEADBAND;
    static constexpr int32_t zero_snap = CTRL_SPD_ZERO_SNAP;
    static constexpr uint16_t sample_div =
        static_cast<uint16_t>(FastLoopConfig::current_hz / estimate_hz);
    static constexpr int32_t ref_limit_counts =
        (ref_limit_rpm * static_cast<int32_t>(MOT_SENSOR_CPR) *
         static_cast<int32_t>(MOT_POLE_PAIRS)) /
        60L;
};

struct PositionLoopConfig
{
    static constexpr int16_t kp = CTRL_POS_KP;
    static constexpr uint8_t err_shift = CTRL_POS_ERR_SHIFT;
    static constexpr int16_t speed_limit_rpm = CTRL_POS_SPEED_LIMIT_RPM;
    static constexpr int32_t brake_accel_rpm_per_s = CTRL_POS_BRAKE_ACCEL_RPM_PER_S;
    static constexpr int32_t deadband_counts = CTRL_POS_DEADBAND_COUNTS;
};

struct EncoderConfig
{
    static constexpr int32_t sensor_cpr = static_cast<int32_t>(MOT_SENSOR_CPR);
    static constexpr int32_t motor_pole_pairs = static_cast<int32_t>(MOT_POLE_PAIRS);
    static constexpr int32_t counts_per_rev = sensor_cpr * motor_pole_pairs;
    static constexpr int8_t direction = MOT_SENSOR_DIR; // 1 or -1
    static constexpr int16_t elec_zero = MOT_ELEC_ZERO; //-13478
    static constexpr uint16_t max_step_raw = MOT_ENCODER_MAX_STEP_RAW;
    static constexpr uint8_t max_angle_age = MOT_ANGLE_MAX_AGE;
};

struct CurrentCheckConfig
{
    static constexpr int16_t phase_limit = MOT_CHECK_CURR_CNT_LIMIT;
    static constexpr int16_t sum_limit = MOT_CHECK_SUM_CNT_LIMIT;
};

struct OpenLoopConfig
{
    static constexpr int32_t speed_ref = OL_SPEED_REF;
    static constexpr int16_t vf_voltage = OL_VF_VOLTAGE;
    static constexpr int16_t if_id_ref = OL_IF_ID_REF;
    static constexpr int16_t if_iq_ref = OL_IF_IQ_REF;
    static constexpr uint16_t timeout_ms = OL_TIMEOUT_MS;
    static constexpr int32_t speed_to_theta_step = OL_SPEED_TO_THETA_STEP;
    static constexpr uint8_t speed_to_theta_shift = OL_SPEED_TO_THETA_SHIFT;
};

static_assert(PwmConfig::duty_min < PwmConfig::duty_center, "invalid PWM min");
static_assert(PwmConfig::duty_center < PwmConfig::duty_max, "invalid PWM max");
static_assert(PwmConfig::adc_trigger_tick > PwmConfig::deadtime_ticks,
              "ADC trigger is too close to PWM edge");
static_assert(PwmConfig::adc_trigger_tick < PwmConfig::period,
              "ADC trigger is outside PWM period");

static_assert(FastLoopConfig::divider > 0U, "fast loop divider must be non-zero");
static_assert(FastLoopConfig::current_hz > 0U, "invalid current loop frequency");

static_assert(CurrentLoopConfig::pi_shift < 15U, "current PI shift too large");
static_assert(CurrentLoopConfig::ref_limit > 0, "current ref limit must be positive");
static_assert(CurrentLoopConfig::voltage_limit > 0, "current voltage limit must be positive");

static_assert(SpeedLoopConfig::err_shift < 15U, "speed PI shift too large");
static_assert(SpeedLoopConfig::estimate_hz > 0, "invalid speed estimate frequency");
static_assert((FastLoopConfig::current_hz % SpeedLoopConfig::estimate_hz) == 0U,
              "speed estimate frequency must divide current loop frequency");
static_assert(SpeedLoopConfig::sample_div > 0U, "invalid speed sample divider");
static_assert(SpeedLoopConfig::diff_spike_rpm > 0, "invalid speed spike limit");
static_assert((SpeedLoopConfig::pos_deadband >= 0) && (SpeedLoopConfig::pos_deadband <= 32767),
              "speed position deadband must fit int16 delta");
static_assert(SpeedLoopConfig::ref_limit_counts == CTRL_SPD_REF_LIMIT,
              "speed ref limit count mismatch");

static_assert(PositionLoopConfig::kp >= 0, "position kp must be non-negative");
static_assert(PositionLoopConfig::err_shift < 31U, "position error shift too large");
static_assert(PositionLoopConfig::speed_limit_rpm > 0, "position speed limit must be positive");
static_assert(PositionLoopConfig::speed_limit_rpm <= SpeedLoopConfig::ref_limit_rpm,
              "position speed limit exceeds speed loop limit");
static_assert(PositionLoopConfig::brake_accel_rpm_per_s > 0,
              "position brake acceleration must be positive");
static_assert(PositionLoopConfig::brake_accel_rpm_per_s <=
                  SpeedLoopConfig::ref_ramp_rpm_per_s,
              "position brake acceleration exceeds speed reference ramp");
static_assert(PositionLoopConfig::deadband_counts >= 0, "position deadband must be non-negative");

static_assert(EncoderConfig::counts_per_rev > 0, "invalid encoder scale");
static_assert(EncoderConfig::motor_pole_pairs > 0, "invalid motor pole pairs");
static_assert((EncoderConfig::direction == 1) || (EncoderConfig::direction == -1),
              "MOT_SENSOR_DIR must be 1 or -1");
static_assert((EncoderConfig::max_step_raw > 0U) && (EncoderConfig::max_step_raw <= 32767U),
              "encoder max step must fit int16 delta");

static_assert(OpenLoopConfig::speed_to_theta_shift > 0U, "VF theta shift must allow rounding");

} // namespace cms32::motor
