#include "motor_control_internal.h"

#include "foc_curr.h"

static int16_t slew_s16(int16_t current, int16_t target, int16_t step);
static int32_t slew_s32(int32_t current, int32_t target, int32_t step);
static int16_t slew_speed_iq(int16_t current, int16_t target);
static int32_t speed_ref_ramp_step_counts(void);
static void update_speed_loop(MotorControlCState* mc);

/** @brief 清空电流 PI 和当前电流给定斜坡状态。 */
void MotorControl_CurrentReset(MotorControlCState* mc)
{
    foc_pi_reset(&mc->current_pi_d);
    foc_pi_reset(&mc->current_pi_q);
    mc->current_loop_div = 0U;
    mc->current_dq = (FocDq_t){0, 0};
    mc->id_ref_active = 0;
    mc->iq_ref_active = 0;
}

/** @brief 清空速度 PI、速度估算和编码器状态。 */
void MotorControl_SpeedReset(MotorControlCState* mc)
{
    mc->speed_reset_count++;
    MotorControl_EncoderReset(mc);
    foc_pi_reset(&mc->speed_pi);
    mc->speed_sample_div = 0U;
}

/**
 * @brief 运行 Current/Speed 主线快环。
 *
 * Current 和 Speed 都会按 CTRL_SPD_EST_HZ 更新编码器差分速度，方便电流环
 * 调试时直接观察 speed_fb_rpm；只有 speed_mode 非 0 时才运行速度 PI，并用
 * speed_iq_ref 作为 q 轴电流给定。
 */
void MotorControl_CurrentRunFastLoop(MotorControlCState* mc, uint8_t speed_mode)
{
    FocAlphaBeta_t current_ab;
    uint16_t theta_used;
    int16_t v_limit;
    int16_t iq_ref;

    mc->current.u = curr_u();
    mc->current.v = curr_v();
    mc->current.w = curr_w();
    if (MotorControl_InternalCurrentOk(mc) == 0U)
    {
        mc->state = MC_STATE_FAULT;
        mc->fault = MC_FAULT_CURRENT;
        MotorControl_InternalEnterSafeState(mc);
        return;
    }

    if (++mc->current_loop_div < CTRL_FAST_LOOP_DIV)
    {
        return;
    }
    mc->current_loop_div = 0U;

    if (MotorControl_InternalUpdateEncoderAngle(mc) == 0U)
    {
        mc->state = MC_STATE_FAULT;
        mc->fault = MC_FAULT_ENCODER;
        MotorControl_InternalEnterSafeState(mc);
        return;
    }

    if (++mc->speed_sample_div >= MC_SPEED_SAMPLE_DIV)
    {
        mc->speed_sample_div = 0U;
        if (MotorControl_InternalUpdateEncoderSpeed(mc) == 0U)
        {
            mc->state = MC_STATE_FAULT;
            mc->fault = MC_FAULT_ENCODER;
            MotorControl_InternalEnterSafeState(mc);
            return;
        }
        if (speed_mode != 0U)
        {
            update_speed_loop(mc);
        }
    }

    if (speed_mode != 0U)
    {
        iq_ref = mc->speed_iq_ref;
    }
    else
    {
        iq_ref = mc->command.iq_ref;
    }

    mc->id_ref_active = slew_s16(mc->id_ref_active, mc->command.id_ref,
                                 CTRL_CUR_REF_RAMP_STEP);
    mc->iq_ref_active =
        slew_s16(mc->iq_ref_active,
                 foc_clamp_s16(iq_ref, (int16_t)-mc->command.iq_limit,
                               mc->command.iq_limit),
                 CTRL_CUR_REF_RAMP_STEP);

    theta_used = (uint16_t)(mc->encoder_elec + (uint16_t)mc->command.voltage_theta_offset);
    current_ab = foc_clarke_3phase(mc->current);
    mc->current_dq = foc_park(current_ab, theta_used);

    v_limit = MotorControl_InternalVoltageLimit(mc);
    foc_pi_set_gains(&mc->current_pi_d, mc->command.current_kp, mc->command.current_ki,
                     (int16_t)-v_limit, v_limit, CTRL_CUR_PI_SHIFT);
    foc_pi_set_gains(&mc->current_pi_q, mc->command.current_kp, mc->command.current_ki,
                     (int16_t)-v_limit, v_limit, CTRL_CUR_PI_SHIFT);

    mc->voltage_dq.d = foc_pi_update(&mc->current_pi_d, mc->id_ref_active,
                                     mc->current_dq.d);
    mc->voltage_dq.q = foc_pi_update(&mc->current_pi_q, mc->iq_ref_active,
                                     mc->current_dq.q);
    MotorControl_InternalApplyVoltageVector(mc, mc->voltage_dq.d, mc->voltage_dq.q,
                                            theta_used);
    mc->fast_loop_count++;
}

/** @brief 对 int16 命令做斜率限制，避免电流/速度命令突变。 */
static int16_t slew_s16(int16_t current, int16_t target, int16_t step)
{
    int32_t delta;

    if (step <= 0)
    {
        return target;
    }

    delta = (int32_t)target - (int32_t)current;
    if (delta > step)
    {
        return (int16_t)(current + step);
    }
    if (delta < -step)
    {
        return (int16_t)(current - step);
    }
    return target;
}

/** @brief 对 int32 命令做斜率限制，用于速度目标斜坡。 */
static int32_t slew_s32(int32_t current, int32_t target, int32_t step)
{
    const int32_t delta = target - current;

    if (step <= 0)
    {
        return target;
    }
    if (delta > step)
    {
        return current + step;
    }
    if (delta < -step)
    {
        return current - step;
    }
    return target;
}

/** @brief 运行速度 PI，输出经斜率限制的 q 轴电流命令。 */
static void update_speed_loop(MotorControlCState* mc)
{
    const int32_t ref_target = mc->command.speed_ref;
    const int16_t ref_target_rpm = MotorControl_InternalSpeedCountsToRpm(ref_target);
    const int16_t fb_rpm = MotorControl_InternalSpeedCountsToRpm(mc->speed_fb);
    int16_t iq_min = (int16_t)-mc->command.iq_limit;
    int16_t iq_max = mc->command.iq_limit;
    int16_t ref_rpm;

    mc->speed_loop_count++;
    mc->speed_ref_active = slew_s32(mc->speed_ref_active, ref_target,
                                    speed_ref_ramp_step_counts());
    ref_rpm = MotorControl_InternalSpeedCountsToRpm(mc->speed_ref_active);
    mc->speed_err_rpm = (int16_t)foc_clamp_s32((int32_t)ref_rpm - (int32_t)fb_rpm,
                                               -32768, 32767);

    if ((ref_target_rpm > -CTRL_SPD_CMD_DEADBAND_RPM) &&
        (ref_target_rpm < CTRL_SPD_CMD_DEADBAND_RPM))
    {
        mc->speed_deadband_count++;
        foc_pi_reset(&mc->speed_pi);
        mc->speed_err_rpm = 0;
        mc->speed_iq_target = 0;
        mc->speed_iq_ff = 0;
        mc->speed_iq_ref = 0;
        mc->speed_ref_active = 0;
        return;
    }

    foc_pi_set_gains(&mc->speed_pi, mc->command.speed_kp, mc->command.speed_ki, iq_min,
                     iq_max, CTRL_SPD_ERR_SHIFT);
    mc->speed_iq_target = foc_pi_update(&mc->speed_pi, ref_rpm, fb_rpm);
    mc->speed_iq_ff = 0;
    mc->speed_iq_ref = slew_speed_iq(mc->speed_iq_ref, mc->speed_iq_target);
}

/** @brief 速度环 iq 输出使用对称斜率，避免低惯量电机收油滞后。 */
static int16_t slew_speed_iq(int16_t current, int16_t target)
{
    return slew_s16(current, target, CTRL_SPD_IQ_SLEW_STEP);
}

/** @brief 将 rpm/s 速度斜坡换算为每次速度环更新的编码器电角 count/s 步进。 */
static int32_t speed_ref_ramp_step_counts(void)
{
    int32_t step = ((int32_t)CTRL_SPD_REF_RAMP_RPM_PER_S * MC_SPEED_COUNTS_PER_REV) /
                   (60L * (int32_t)CTRL_SPD_EST_HZ);

    if (step <= 0)
    {
        step = 1;
    }
    return step;
}
