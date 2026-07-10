#include "motor_control_internal.h"

#include "foc_bsp.h"

#if (CTRL_SPD_FB_SOURCE != CTRL_SPD_FB_SOURCE_DIFF) && \
    (CTRL_SPD_FB_SOURCE != CTRL_SPD_FB_SOURCE_MA600)
#error "Unsupported CTRL_SPD_FB_SOURCE"
#endif

static uint16_t electrical_from_raw(uint16_t raw, int16_t trim);
static int16_t encoder_raw_delta(MotorControlCState* mc, uint16_t raw);
static uint8_t encoder_raw_plausible(MotorControlCState* mc, uint16_t raw);
static uint8_t hold_last_encoder_angle(MotorControlCState* mc);
static uint8_t reject_bad_encoder_angle(MotorControlCState* mc, uint16_t raw);
static void accept_encoder_angle(MotorControlCState* mc, uint16_t raw);
static uint8_t retry_encoder_angle(MotorControlCState* mc);
static uint8_t update_encoder_angle_state(MotorControlCState* mc);
static uint8_t update_encoder_speed_state(MotorControlCState* mc);
static uint16_t speed_diff_max_delta_raw(void);
#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
static int32_t rpm_to_speed_counts_s32(int32_t rpm);
static int32_t ma600_speed_to_counts(int16_t speed_raw);
#endif

/** @brief 将编码器电角 count/s 转为机械 rpm 观察单位。 */
int16_t MotorControl_InternalSpeedCountsToRpm(int32_t speed)
{
    int32_t rpm = (speed * 60L) / MC_SPEED_COUNTS_PER_REV;
    return (int16_t)foc_clamp_s32(rpm, -32768, 32767);
}

/** @brief 清空编码器、速度反馈和坏角诊断状态。 */
void MotorControl_EncoderReset(MotorControlCState* mc)
{
    mc->encoder_raw = 0U;
    mc->encoder_elec = 0U;
    mc->encoder_prev_raw = 0U;
    mc->encoder_delta = 0;
    mc->encoder_pos = 0;
    mc->encoder_raw_step = 0;
    mc->encoder_reject_step = 0;
    mc->encoder_reject_prev_raw = 0U;
    mc->encoder_reject_raw = 0U;
    mc->encoder_reject_count = 0U;
    mc->encoder_retry_count = 0U;
    mc->encoder_retry_accept_count = 0U;
    mc->encoder_retry_raw = 0U;
    mc->encoder_hold_count = 0U;
    mc->speed_reject_count = 0U;
    mc->speed_reject_delta = 0;
    mc->speed_fb = 0;
    mc->speed_fb_diff = 0;
    mc->speed_fb_ma600 = 0;
    mc->ma600_speed_raw = 0;
    mc->speed_err_rpm = 0;
    mc->speed_iq_target = 0;
    mc->speed_iq_ff = 0;
    mc->speed_iq_ref = 0;
    mc->speed_ref_active = 0;
    mc->speed_startup_blank = CTRL_SPD_STARTUP_BLANK_SAMPLES;
    mc->encoder_age = 255U;
    mc->encoder_ok = 0U;
    mc->encoder_initialized = 0U;
}

/** @brief 供控制快环和 VF 观察路径复用的编码器角度更新入口。 */
uint8_t MotorControl_InternalUpdateEncoderAngle(MotorControlCState* mc)
{
    return update_encoder_angle_state(mc);
}

/** @brief 供速度环和 VF 观察路径复用的速度反馈更新入口。 */
uint8_t MotorControl_InternalUpdateEncoderSpeed(MotorControlCState* mc)
{
    return update_encoder_speed_state(mc);
}

/** @brief 将 MA600 raw 按零位、方向和极对映射为 16-bit 电角度。 */
static uint16_t electrical_from_raw(uint16_t raw, int16_t trim)
{
    const int32_t zero = (int32_t)MOT_ELEC_ZERO + (int32_t)trim;
#if MOT_SENSOR_DIR > 0
    return (uint16_t)(zero + (int32_t)raw * (int32_t)MOT_SENSOR_ELEC);
#else
    return (uint16_t)(zero - (int32_t)raw * (int32_t)MOT_SENSOR_ELEC);
#endif
}

/** @brief 计算带 16-bit 回绕语义的 raw 增量。 */
static int16_t encoder_raw_delta(MotorControlCState* mc, uint16_t raw)
{
    return (int16_t)(raw - mc->encoder_raw);
}

/** @brief 根据单拍最大 raw 步进判断角度样本是否可信。 */
static uint8_t encoder_raw_plausible(MotorControlCState* mc, uint16_t raw)
{
    const int16_t delta = encoder_raw_delta(mc, raw);

    if (mc->encoder_initialized == 0U)
    {
        return 1U;
    }

    return (uint8_t)((delta <= (int16_t)MOT_ENCODER_MAX_STEP_RAW) &&
                     (delta >= -(int16_t)MOT_ENCODER_MAX_STEP_RAW));
}

/** @brief 角度读取失败时保持上一角度，并更新 age/ok。 */
static uint8_t hold_last_encoder_angle(MotorControlCState* mc)
{
    mc->encoder_hold_count++;
    if (mc->encoder_age < 255U)
    {
        mc->encoder_age++;
    }

    mc->encoder_ok = (uint8_t)((mc->encoder_initialized != 0U) &&
                               (mc->encoder_age <= MOT_ANGLE_MAX_AGE));
    return mc->encoder_ok;
}

/** @brief 记录坏角样本并保持上一角度。 */
static uint8_t reject_bad_encoder_angle(MotorControlCState* mc, uint16_t raw)
{
    mc->encoder_reject_count++;
    mc->encoder_reject_step = encoder_raw_delta(mc, raw);
    mc->encoder_reject_prev_raw = mc->encoder_raw;
    mc->encoder_reject_raw = raw;
    return hold_last_encoder_angle(mc);
}

/** @brief 接受 raw 样本，更新电角度、步进和缓存状态。 */
static void accept_encoder_angle(MotorControlCState* mc, uint16_t raw)
{
    if (mc->encoder_initialized == 0U)
    {
        mc->encoder_raw_step = 0;
        mc->encoder_prev_raw = raw;
        mc->encoder_delta = 0;
        mc->encoder_initialized = 1U;
    }
    else
    {
        mc->encoder_raw_step = encoder_raw_delta(mc, raw);
    }

    mc->encoder_raw = raw;
    mc->encoder_elec = electrical_from_raw(raw, mc->command.elec_zero_trim);
    mc->encoder_ok = 1U;
    mc->encoder_age = 0U;
}

/** @brief 坏角后立即重读一次，用于过滤单帧 SPI 毛刺。 */
static uint8_t retry_encoder_angle(MotorControlCState* mc)
{
    uint16_t retry_raw;

    mc->encoder_retry_count++;
    if ((bsp_update_angle_fast() == 0U) || (bsp_angle_ok() == 0U))
    {
        return 0U;
    }

    retry_raw = bsp_angle_raw();
    mc->encoder_retry_raw = retry_raw;
    if (encoder_raw_plausible(mc, retry_raw) == 0U)
    {
        return 0U;
    }

    mc->encoder_retry_accept_count++;
    accept_encoder_angle(mc, retry_raw);
    return 1U;
}

/** @brief 快环读取编码器角度，执行可信度检查、重读和接受/拒绝。 */
static uint8_t update_encoder_angle_state(MotorControlCState* mc)
{
    uint8_t ok;
    uint16_t raw;

    ok = bsp_update_angle_fast();
    raw = bsp_angle_raw();

    if ((ok == 0U) || (bsp_angle_ok() == 0U))
    {
        return hold_last_encoder_angle(mc);
    }

    if (encoder_raw_plausible(mc, raw) == 0U)
    {
        if (retry_encoder_angle(mc) != 0U)
        {
            return 1U;
        }
        return reject_bad_encoder_angle(mc, raw);
    }

    accept_encoder_angle(mc, raw);
    return 1U;
}

/** @brief 根据编码器 raw 差分或 MA600 speed frame 更新速度反馈。 */
static uint8_t update_encoder_speed_state(MotorControlCState* mc)
{
    uint16_t raw;
    int16_t delta;
    uint16_t max_delta;
    int32_t speed_sample;

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
    int32_t speed_ma600;
#endif

    raw = mc->encoder_raw;
    if (mc->encoder_initialized == 0U)
    {
        mc->encoder_prev_raw = raw;
        mc->encoder_delta = 0;
        mc->encoder_initialized = 1U;
    }

    delta = (int16_t)(raw - mc->encoder_prev_raw);
    max_delta = speed_diff_max_delta_raw();
    if ((delta > (int16_t)max_delta) || (delta < -(int16_t)max_delta))
    {
        mc->speed_reject_count++;
        mc->speed_reject_delta = delta;
        mc->encoder_prev_raw = raw;
        mc->encoder_delta = 0;
        mc->speed_fb_diff = 0;
        return 1U;
    }

    mc->encoder_prev_raw = raw;
    mc->encoder_delta = delta;
    mc->encoder_pos += delta;

    if (mc->speed_startup_blank != 0U)
    {
        mc->speed_startup_blank--;
        mc->speed_fb = 0;
        mc->speed_fb_diff = 0;
        return 1U;
    }

    if ((delta > -CTRL_SPD_POS_DEADBAND) && (delta < CTRL_SPD_POS_DEADBAND))
    {
        delta = 0;
    }

    speed_sample = (int32_t)delta * (int32_t)CTRL_SPD_EST_HZ * (int32_t)MOT_SENSOR_DIR;
    mc->speed_fb_diff += (speed_sample - mc->speed_fb_diff) >> CTRL_SPD_FILTER_SHIFT;

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
    mc->ma600_speed_raw = bsp_angle_speed_raw();
    speed_ma600 = ma600_speed_to_counts(mc->ma600_speed_raw);
    if ((speed_ma600 - mc->speed_fb_ma600 <=
         rpm_to_speed_counts_s32(CTRL_SPD_MA600_SPIKE_RPM)) &&
        (speed_ma600 - mc->speed_fb_ma600 >=
         -rpm_to_speed_counts_s32(CTRL_SPD_MA600_SPIKE_RPM)))
    {
        mc->speed_fb_ma600 +=
            (speed_ma600 - mc->speed_fb_ma600) >> CTRL_SPD_MA600_FILTER_SHIFT;
    }
#else
    mc->ma600_speed_raw = 0;
    mc->speed_fb_ma600 = 0;
#endif

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
    mc->speed_fb = mc->speed_fb_ma600;
#else
    mc->speed_fb = mc->speed_fb_diff;
#endif

    if ((mc->speed_fb > -CTRL_SPD_ZERO_SNAP) && (mc->speed_fb < CTRL_SPD_ZERO_SNAP))
    {
        mc->speed_fb = 0;
    }
    return 1U;
}

/** @brief 将速度 spike rpm 阈值换算为一次速度采样允许的 raw 差分。 */
static uint16_t speed_diff_max_delta_raw(void)
{
    int32_t limit = ((int32_t)CTRL_SPD_DIFF_SPIKE_RPM *
                     (int32_t)MOT_SENSOR_CPR *
                     (int32_t)MOT_SENSOR_POLE_PAIRS) /
                    (60L * (int32_t)CTRL_SPD_EST_HZ);

    if (limit < 1)
    {
        limit = 1;
    }
    if (limit > 32767L)
    {
        limit = 32767L;
    }
    return (uint16_t)limit;
}

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
/** @brief 将 int32 rpm 转为编码器电角 count/s，供 MA600 spike 限幅使用。 */
static int32_t rpm_to_speed_counts_s32(int32_t rpm)
{
    return (rpm * MC_SPEED_COUNTS_PER_REV) / 60L;
}

/** @brief 将 MA600 speed raw 换算为编码器电角 count/s。 */
static int32_t ma600_speed_to_counts(int16_t speed_raw)
{
    return (int32_t)speed_raw * (int32_t)CTRL_SPD_MA600_COUNTS_PER_SEC_PER_LSB *
           (int32_t)MOT_SENSOR_DIR * (int32_t)CTRL_SPD_MA600_SIGN;
}
#endif
