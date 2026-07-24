#include "motor_controller.hpp"

#include "encoder_math.hpp"
#include "speed_math.hpp"
#include "units.hpp"

#include "foc_bsp.h"

#include <stdint.h>

namespace
{

using cms32::motor::angle_step_max_delta_raw;
using cms32::motor::EncoderConfig;
using cms32::motor::raw_delta;
using cms32::motor::raw_delta_plausible;
using cms32::motor::raw_delta_to_speed_counts;
using cms32::motor::speed_diff_max_delta_raw;
using cms32::motor::SpeedMath;
using cms32::motor::zero_snap_speed;
using cms32::support::EncoderRaw;
using cms32::support::SpeedCounts;

/** @brief 将 MA600 raw 角度换算成控制快环使用的 16-bit 电角度。 */
uint16_t electrical_from_raw(uint16_t raw, int16_t trim) noexcept
{
    const int32_t zero =
        static_cast<int32_t>(EncoderConfig::elec_zero) + static_cast<int32_t>(trim);

    /*
     * 当前磁环极对和电机极对一致，MA600 raw 已经是电角度标尺。
     * direction 只决定正反转方向。
     * 最后强转 uint16_t 是故意使用 16-bit 自然回绕，一整圈就是 0..65535。
     */
    if constexpr (EncoderConfig::direction > 0)
    {
        return static_cast<uint16_t>(zero + static_cast<int32_t>(raw));
    }
    else
    {
        return static_cast<uint16_t>(zero - static_cast<int32_t>(raw));
    }
}

} // namespace

namespace cms32::motor
{

/** @brief 计算当前 raw 相对上一帧已接受 raw 的差值，保留 16-bit 回绕语义。 */
static int16_t encoder_raw_delta(const MCEncoderState_t& encoder, uint16_t raw) noexcept
{
    return raw_delta(EncoderRaw{encoder.raw}, EncoderRaw{raw});
}

/** @brief 本次没有可用新角度时，继续使用上一帧角度并更新 age/ok。 */
static uint8_t hold_last_encoder_angle(MCEncoderState_t& encoder,
                                       MotorControlDiag_t& diag) noexcept
{
    diag.encoder_hold_count++;
    if (encoder.age < 255U)
    {
        encoder.age++;
    }

    /*
     * hold 不是立即 fault。短时间 SPI 失败可以容忍，超过 EncoderConfig::max_angle_age
     * 才把 encoder_ok 拉低，让上层闭环进入 encoder fault。
     */
    encoder.ok = static_cast<uint8_t>((encoder.initialized != 0U) &&
                                      (encoder.age <= EncoderConfig::max_angle_age));
    return encoder.ok;
}

/** @brief 判断本次 raw 差分是否适合写入多圈位置累计。 */
static bool encoder_position_delta_plausible(const MCEncoderState_t& encoder,
                                             int16_t delta) noexcept
{
    return raw_delta_plausible(delta, angle_step_max_delta_raw(encoder.age));
}

/** @brief 拒绝明显不可信 raw 样本，保持上一帧角度和位置。 */
static uint8_t reject_encoder_angle(MCEncoderState_t& encoder,
                                    MotorControlDiag_t& diag,
                                    uint16_t raw,
                                    int16_t delta) noexcept
{
    diag.encoder_reject_count++;
    diag.encoder_reject_step = delta;
    diag.encoder_reject_prev_raw = encoder.raw;
    diag.encoder_reject_raw = raw;
    return hold_last_encoder_angle(encoder, diag);
}

/** @brief 接受 raw 样本，并同步更新电角度、20 kHz 位置累计和最近步进。 */
uint8_t accept_encoder_angle(MotorController& motor, uint16_t raw) noexcept
{
    auto& encoder = motor.encoder;
    auto& diag = motor.diag;

    if (encoder.initialized == 0U)
    {
        /*
         * 首帧只建立 raw 基准。速度估算也会从这个 raw 开始，
         * 所以 encoder_prev_raw 同步到同一个值，避免首拍出现假速度。
         */
        diag.encoder_raw_step = 0;
        encoder.prev_raw = raw;
        encoder.delta = 0;
        encoder.initialized = 1U;
    }
    else
    {
        const int16_t delta = encoder_raw_delta(encoder, raw);
        diag.encoder_raw_step = delta;
        if (!encoder_position_delta_plausible(encoder, delta))
        {
            return reject_encoder_angle(encoder, diag, raw, delta);
        }

        /*
         * 多圈位置在 20 kHz 角度链路累计，而不是等 1 kHz 速度估算再累计。
         * 5000 rpm 时 1 ms 约 21845 count，离 int16 半圈极限很近；20 kHz
         * 每拍约 1092 count，能显著降低高速短行程时丢圈/方向歧义的风险。
         */
        encoder.pos += delta;
    }

    /*
     * encoder_raw 是后续 angle/speed 的状态源。
     * encoder_elec 只给 FOC 用，不反过来参与 raw 连续性判断。
     */
    encoder.raw = raw;
    encoder.elec = electrical_from_raw(raw, motor.command.current.elec_zero_trim);
    encoder.ok = 1U;
    encoder.age = 0U;
    return 1U;
}

/** @brief 快环角度入口：读取 MA600；只要 Board 层读到有效角度就直接接受 raw。 */
uint8_t MotorController::updateEncoderAngle() noexcept
{
    const uint8_t ok = bsp_update_angle_fast();
    const uint16_t raw = bsp_angle_raw();

    /*
     * SPI 读失败或者 Board 层标记角度无效时，不更新 raw。
     * 短时间继续使用上一帧角度，避免单次通信失败直接打断闭环。
     */
    if ((ok == 0U) || (bsp_angle_ok() == 0U))
    {
        return hold_last_encoder_angle(encoder, diag);
    }

    return accept_encoder_angle(*this, raw);
}

/** @brief 速度估算入口：用已接受 raw 的 1 kHz 差分更新 speed feedback。 */
uint8_t MotorController::updateEncoderSpeed() noexcept
{
    const uint16_t raw = encoder.raw;
    if (encoder.initialized == 0U)
    {
        /*
         * 理论上速度估算前角度已经初始化。这里保留保护逻辑：
         * 如果还没有基准 raw，就先建立基准并输出零速度。
         */
        encoder.prev_raw = raw;
        encoder.delta = 0;
        encoder.initialized = 1U;
    }

    /*
     * 速度估算使用 encoder_prev_raw，而不是 encoder_raw。
     * encoder_raw 是角度链路最近接受值；encoder_prev_raw 是速度链路上一次采样值。
     */
    const int16_t delta = raw_delta(EncoderRaw{encoder.prev_raw}, EncoderRaw{raw});
    encoder.prev_raw = raw;
    encoder.delta = delta;

    const uint16_t max_delta = speed_diff_max_delta_raw();
    if (!raw_delta_plausible(delta, max_delta))
    {
        /*
         * 速度估算的异常 raw 差分按“拒绝但不断流”处理：
         * 位置已经由 20 kHz 角度链路累计；这里仅清本次差分速度，
         * 避免同一个毛刺进入速度低通和速度 PI。
         */
        diag.speed_reject_count++;
        diag.speed_reject_delta = delta;
        speed.feedback_filter.reset();
        return 1U;
    }

    if (speed.startup_blank != 0U)
    {
        /*
         * 模式切入后前几拍只建立 prev/raw 连续性，不输出速度。
         * 这样可以避免刚进入闭环时的第一几个差分样本冲击速度 PI。
         */
        speed.startup_blank--;
        speed.feedback = 0;
        speed.feedback_filter.reset();
        return 1U;
    }

    /*
     * raw_delta_to_speed_counts() 内部会做小 delta deadband：
     * 小抖动不进入 speed_sample，但上面的 encoder_pos 仍保留真实累计。
     */
    const int32_t speed_sample = raw_delta_to_speed_counts(delta).value;
    speed.feedback_filter.update(speed_sample);
    /*
     * speed.feedback_filter 是滤波器内部状态；speed.feedback 是对外反馈。
     * zero snap 只作用在 speed.feedback，避免零速附近显示和 PI 输入抖动。
     */
    speed.feedback = zero_snap_speed(SpeedCounts{speed.feedback_filter.value}).value;
    return 1U;
}

int16_t MotorController::speedCountsToRpm(int32_t speed_counts) const noexcept
{
    /* C ABI 仍返回裸 int16_t，内部用 SpeedCounts/Rpm 保留单位语义。 */
    return SpeedMath::to_rpm(SpeedCounts{speed_counts}).value;
}

void MotorController::encoderReset() noexcept
{
    /*
     * 这里只清真实的 g_motor 状态，不清任何隐藏 C++ 对象状态。
     * 当前正式状态源仍然完全可在 Ozone 中观察。
     */
    encoder.raw = 0U;
    encoder.elec = 0U;
    encoder.prev_raw = 0U;
    encoder.delta = 0;
    encoder.pos = 0;
    diag.encoder_raw_step = 0;
    diag.encoder_reject_step = 0;
    diag.encoder_reject_prev_raw = 0U;
    diag.encoder_reject_raw = 0U;
    diag.encoder_reject_count = 0U;
    diag.encoder_retry_count = 0U;
    diag.encoder_retry_accept_count = 0U;
    diag.encoder_retry_raw = 0U;
    diag.encoder_hold_count = 0U;
    diag.speed_reject_count = 0U;
    diag.speed_reject_delta = 0;
    speed.feedback = 0;
    speed.feedback_filter.reset();
    speed.err_rpm = 0;
    speed.iq_target = 0;
    speed.iq_ff = 0;
    speed.iq_ref.reset();
    speed.ref_active.reset();
    speed.startup_blank = SpeedLoopConfig::startup_blank_samples;
    encoder.age = 255U;
    encoder.ok = 0U;
    encoder.initialized = 0U;
}

} // namespace cms32::motor

extern "C" int16_t MotorControl_InternalSpeedCountsToRpm(int32_t speed)
{
    return cms32::motor::g_motor.speedCountsToRpm(speed);
}

extern "C" void MotorControl_EncoderReset(void)
{
    cms32::motor::g_motor.encoderReset();
}

extern "C" uint8_t MotorControl_InternalUpdateEncoderAngle(void)
{
    return cms32::motor::g_motor.updateEncoderAngle();
}

extern "C" uint8_t MotorControl_InternalUpdateEncoderSpeed(void)
{
    return cms32::motor::g_motor.updateEncoderSpeed();
}
