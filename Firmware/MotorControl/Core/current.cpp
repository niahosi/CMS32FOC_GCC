#include "config.hpp"
#include "motor_control_state.h"
#include "motor_controller.hpp"

#include "fixed_pi.hpp"
#include "foc_math.h"
#include "speed_math.hpp"

#include "clamp.hpp"
#include "board_profile.h"
#include "foc_curr.h"
#include "types.hpp"
#include "units.hpp"

#include <stdint.h>

namespace cms32::motor
{

namespace
{

/** 64-bit 非负整数平方根，位置环 1 kHz 调用，不引入 float/libm 依赖。 */
uint32_t isqrt_u64(uint64_t value) noexcept
{
    uint64_t result = 0U;
    uint64_t bit = UINT64_C(1) << 62U;

    while (bit > value)
    {
        bit >>= 2U;
    }

    while (bit != 0U)
    {
        if (value >= (result + bit))
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }

    return static_cast<uint32_t>(result);
}

/**
 * 根据剩余机械位置返回可刹停的最高机械转速。
 *
 * v^2 = 2 * alpha * theta，换成 rpm、rpm/s 和 encoder count 后：
 * v_rpm^2 = 120 * alpha_rpm_per_s * abs(error_counts) / counts_per_rev。
 */
int16_t braking_speed_limit_rpm(int32_t error_counts) noexcept
{
    const int64_t error = static_cast<int64_t>(error_counts);
    const uint64_t distance_counts = static_cast<uint64_t>((error >= 0) ? error : -error);
    const uint64_t numerator = UINT64_C(120) *
                               static_cast<uint64_t>(PositionLoopConfig::brake_accel_rpm_per_s) *
                               distance_counts;
    const uint64_t speed_squared = numerator /
                                   static_cast<uint64_t>(EncoderConfig::counts_per_rev);
    const uint32_t speed_rpm = isqrt_u64(speed_squared);

    return static_cast<int16_t>(cms32::support::clamp<uint32_t>(speed_rpm,
                                                                 0U,
                                                                 static_cast<uint32_t>(INT16_MAX)));
}

} // namespace

/** @brief 清空电流 PI 和当前电流给定斜坡状态，不改变命令缓存。 */
void MotorController::currentReset() noexcept
{
    auto& cur = current;

    FixedPiRef{cur.pi_d}.reset();
    FixedPiRef{cur.pi_q}.reset();
    cur.loop_div = 0U;
    cur.dq = FocDq_t{0, 0};
    cur.id_ref_active.reset();
    cur.iq_ref_active.reset();
}

/** @brief 清空速度 PI、速度滤波和速度输出，不改变累计编码器位置。 */
void MotorController::speedControllerReset() noexcept
{
    FixedPiRef{speed.pi}.reset();
    speed.sample_div = 0U;
    speed.feedback = 0;
    speed.feedback_filter.reset();
    speed.err_rpm = 0;
    speed.iq_target = 0;
    speed.iq_ff = 0;
    speed.iq_ref.reset();
    speed.ref_active.reset();
    speed.startup_blank = SpeedLoopConfig::startup_blank_samples;
}

/** @brief 清空速度 PI、速度估算和编码器状态，用于非位置模式闭环重入。 */
void MotorController::speedReset() noexcept
{
    diag.speed_reset_count++;
    encoderReset();
    speedControllerReset();
}

/**
 * @brief 运行 Current/Speed/Position 主线快环。
 *
 * Current/Speed/Position 都会按 SpeedLoopConfig::estimate_hz 更新编码器差分速度，
 * 方便电流环调试时直接观察 speed_fb_rpm。Position 先在 1 kHz tick 上把位置误差
 * 转成速度目标，随后复用速度 PI；Speed 直接复用命令里的速度目标；Current 只用
 * 命令里的 iq_ref。
 */
void MotorController::positionReset() noexcept
{
    position.target = encoder.pos;
    position.feedback = encoder.pos;
    position.error = 0;
    position.speed_ref = 0;
    position.speed_ref_rpm = 0;
    position.brake_speed_limit_rpm = 0;
    position.at_target = 1U;
}

void MotorController::runCurrentFastLoop(ControlMode mode) noexcept
{
    auto& cur = current;
    auto& spd = speed;
    auto& out = output;
    auto& enc = encoder;
    const auto& current_command = command.current;

    cur.phase.u = curr_u();
    cur.phase.v = curr_v();
    cur.phase.w = curr_w();
    if (currentOk() == 0U)
    {
        runtime.state = MC_STATE_FAULT;
        runtime.fault = MC_FAULT_CURRENT;
        enterSafeState();
        publishDebugState();
        return;
    }

    /*
     * ADC/PWM 同步频率可以高于电流环执行频率。
     * 分频返回时仍保留最新电流采样，方便 Ozone 看到实时电流。
     */
    if (++cur.loop_div < FastLoopConfig::divider)
    {
        return;
    }
    cur.loop_div = 0U;

    /*
     * Current 模式也需要电角度做 Park/InvPark，所以 Current/Speed/Position
     * 都必须在电流环执行拍读取并校验编码器角度。
     */
    BoardProfile_EncoderFastBegin();
    const uint8_t encoder_ok = updateEncoderAngle();
    BoardProfile_EncoderFastEnd();
    if (encoder_ok == 0U)
    {
        runtime.state = MC_STATE_FAULT;
        runtime.fault = MC_FAULT_ENCODER;
        enterSafeState();
        publishDebugState();
        return;
    }

    /*
     * 速度估算和速度 PI 低频运行。Current 模式仍更新速度反馈，
     * 这样电流环调试时可以同步观察 speed_fb_rpm。
     */
    if (++spd.sample_div >= SpeedLoopConfig::sample_div)
    {
        spd.sample_div = 0U;
        BoardProfile_SpeedSlotBegin();
        if (updateEncoderSpeed() == 0U)
        {
            BoardProfile_SpeedSlotEnd();
            runtime.state = MC_STATE_FAULT;
            runtime.fault = MC_FAULT_ENCODER;
            enterSafeState();
            publishDebugState();
            return;
        }
        if (mode == ControlMode::Position)
        {
            updatePositionLoop();
        }
        if ((mode == ControlMode::Speed) || (mode == ControlMode::Position))
        {
            updateSpeedLoop();
        }
        BoardProfile_SpeedSlotEnd();
        publishDebugState();
    }

    BoardProfile_CurrentControlBegin();
    const int16_t iq_ref = (mode != ControlMode::Current) ? spd.iq_ref.value
                                                          : current_command.iq_ref;

    cur.id_ref_active.update(current_command.id_ref);
    cur.iq_ref_active.update(foc_clamp_s16(iq_ref,
                                           (int16_t)-current_command.iq_limit,
                                           current_command.iq_limit));

    /*
     * voltage_theta_offset 是调试用相位提前/滞后入口，只影响电流环角度，
     * 不改变编码器 raw/electrical 观察值。
     */
    BoardProfile_FocMathBegin();
    const uint16_t theta_used = static_cast<uint16_t>(
        enc.elec + static_cast<uint16_t>(current_command.voltage_theta_offset));
    const FocAlphaBeta_t current_ab = foc_clarke_3phase(cur.phase);
    cur.dq = foc_park(current_ab, theta_used);

    const int16_t v_limit = voltageLimit();
    const FixedPiConfig current_pi_config{current_command.current_kp,
                                          current_command.current_ki,
                                          (int16_t)-v_limit,
                                          v_limit,
                                          CurrentLoopConfig::pi_shift};
    FixedPiRef{cur.pi_d}.set_gains(current_pi_config);
    FixedPiRef{cur.pi_q}.set_gains(current_pi_config);

    out.voltage_dq.d =
        FixedPiRef{cur.pi_d}.update(cur.id_ref_active.value, cur.dq.d);
    out.voltage_dq.q =
        FixedPiRef{cur.pi_q}.update(cur.iq_ref_active.value, cur.dq.q);
    applyVoltageVector(out.voltage_dq.d, out.voltage_dq.q, theta_used);
    BoardProfile_CurrentControlEnd();
    diag.fast_loop_count++;
}

/** @brief 运行 1 kHz 位置 P 环，输出速度环目标。 */
void MotorController::updatePositionLoop() noexcept
{
    auto& pos = position;
    auto& spd = speed;
    const auto& position_command = command.position;

    diag.position_loop_count++;
    pos.target = position_command.position_ref;
    pos.feedback = encoder.pos;
    const int64_t error_counts =
        static_cast<int64_t>(pos.target) - static_cast<int64_t>(pos.feedback);
    pos.error = static_cast<int32_t>(cms32::support::clamp<int64_t>(
        error_counts,
        static_cast<int64_t>(INT32_MIN),
        static_cast<int64_t>(INT32_MAX)));

    const int32_t deadband = position_command.deadband_counts;
    if ((pos.error >= -deadband) && (pos.error <= deadband))
    {
        pos.at_target = 1U;
        pos.speed_ref_rpm = 0;
        pos.speed_ref = 0;
        pos.brake_speed_limit_rpm = 0;
        spd.ref_active.reset();
        spd.iq_ref.reset();
        spd.err_rpm = 0;
        spd.iq_target = 0;
        spd.iq_ff = 0;
        FixedPiRef{spd.pi}.reset();
        command.speed.speed_ref = 0;
        return;
    }

    pos.at_target = 0U;
    const int64_t rpm_unclamped =
        (static_cast<int64_t>(pos.error) *
         static_cast<int64_t>(position_command.position_kp)) >>
        position_command.position_err_shift;
    const int16_t rpm_limit = position_command.speed_limit_rpm;
    pos.brake_speed_limit_rpm =
        static_cast<int16_t>(cms32::support::clamp<int16_t>(
            braking_speed_limit_rpm(pos.error), 0, rpm_limit));
    pos.speed_ref_rpm = static_cast<int16_t>(
        cms32::support::clamp<int64_t>(rpm_unclamped,
                                       static_cast<int64_t>(-pos.brake_speed_limit_rpm),
                                       static_cast<int64_t>(pos.brake_speed_limit_rpm)));
    pos.speed_ref =
        SpeedMath::to_speed(cms32::support::Rpm{pos.speed_ref_rpm}).value;
    command.speed.speed_ref = pos.speed_ref;
}

/** @brief 运行速度 PI，输出经斜率限制的 q 轴电流命令。 */
void MotorController::updateSpeedLoop() noexcept
{
    auto& spd = speed;
    const auto& speed_command = command.speed;
    const int32_t ref_target = speed_command.speed_ref;
    const cms32::support::Rpm ref_target_rpm =
        SpeedMath::to_rpm(cms32::support::SpeedCounts{ref_target});
    const cms32::support::Rpm fb_rpm =
        SpeedMath::to_rpm(cms32::support::SpeedCounts{spd.feedback});
    const int16_t iq_min = (int16_t)-speed_command.iq_limit;
    const int16_t iq_max = speed_command.iq_limit;

    diag.speed_loop_count++;
    spd.ref_active.update(ref_target);
    const cms32::support::Rpm ref_rpm =
        SpeedMath::to_rpm(cms32::support::SpeedCounts{spd.ref_active.value});
    spd.err_rpm = SpeedMath::error_rpm(ref_rpm, fb_rpm);

    /*
     * 速度给定接近 0 时直接清 PI 和斜坡，避免零速附近积分保持造成抖动。
     * 判据使用原始目标 ref_target_rpm，而不是 ramp 后的 ref_rpm。
     */
    if (SpeedMath::in_deadband(ref_target_rpm))
    {
        diag.speed_deadband_count++;
        FixedPiRef{spd.pi}.reset();
        spd.err_rpm = 0;
        spd.iq_target = 0;
        spd.iq_ff = 0;
        spd.iq_ref.reset();
        spd.ref_active.reset();
        return;
    }

    FixedPiRef{spd.pi}.set_gains(FixedPiConfig{speed_command.speed_kp,
                                               speed_command.speed_ki,
                                               iq_min,
                                               iq_max,
                                               SpeedLoopConfig::err_shift});
    spd.iq_target = FixedPiRef{spd.pi}.update(ref_rpm.value, fb_rpm.value);
    spd.iq_ff = 0;
    spd.iq_ref.update(spd.iq_target);
}

} // namespace cms32::motor

extern "C" void MotorControl_CurrentReset(void)
{
    cms32::motor::g_motor.currentReset();
}

extern "C" void MotorControl_SpeedReset(void)
{
    cms32::motor::g_motor.speedReset();
}

extern "C" void MotorControl_CurrentRunFastLoop(uint8_t speed_mode)
{
    cms32::motor::g_motor.runCurrentFastLoop(
        (speed_mode != 0U) ? cms32::motor::ControlMode::Speed
                           : cms32::motor::ControlMode::Current);
}
