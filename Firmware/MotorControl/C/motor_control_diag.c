#include "motor_control_diag.h"

#include "foc_curr.h"

#define MC_ALIGN_STAGE_REV_FAST 1U
#define MC_ALIGN_STAGE_FWD_FAST 2U
#define MC_ALIGN_STAGE_FWD_SAMPLE 3U
#define MC_ALIGN_STAGE_REV_SAMPLE 4U
#define MC_ALIGN_STAGE_DONE 5U

/** @brief 诊断模式私有状态，避免 VF/Align 变量污染 Current/Speed 主线。 */
typedef struct
{
    uint32_t open_loop_ticks;
    uint32_t open_loop_reset_count;
    uint32_t open_loop_timeout_ticks;
    int32_t open_loop_theta_acc;
    uint16_t open_loop_theta;
    uint8_t align_done;
    uint32_t align_ticks;
    uint16_t align_raw;
    int16_t align_zero_trim;
    uint16_t align_encoder_elec;
    uint8_t align_stage;
    uint16_t align_halfcycles;
    int16_t align_theta_prev;
    int16_t align_first_delta;
    int16_t align_pull_delta;
    uint16_t align_sample_count;
    int32_t align_delta_sum;
} MotorControlDiagState;

static MotorControlDiagState s_diag;

static void reset_open_loop_theta(void);
static void reset_align_state(void);
static uint16_t update_open_loop_theta(const MotorControlCommand_t* command);
static uint16_t update_align_theta(int32_t speed);
static void run_vf_fast_loop(MotorControlCState* mc);
static void run_align_fast_loop(MotorControlCState* mc);
static void run_encoder_voltage_fast_loop(MotorControlCState* mc);
static void set_align_stage(uint8_t stage);
static int16_t align_trim_from_raw(uint16_t raw, uint16_t target_theta);
static void count_align_halfcycle(uint16_t theta);
static void sample_align_trim(uint16_t raw, uint16_t theta);
static void finish_align_scan(void);

/** @brief 初始化 VF/Align/EncoderVoltage 诊断状态。 */
void MotorControlDiag_Init(void)
{
    s_diag.open_loop_timeout_ticks = (uint32_t)OL_TIMEOUT_MS * 2U;
    reset_align_state();
    s_diag.open_loop_reset_count = 0U;
}

/** @brief 显式模式切换时重置对应诊断状态。 */
void MotorControlDiag_ResetForMode(uint8_t mode)
{
    if (mode == MC_MODE_VF_OPEN_LOOP)
    {
        reset_open_loop_theta();
    }
    else if (mode == MC_MODE_ALIGN_LOCK)
    {
        reset_align_state();
        s_diag.open_loop_reset_count++;
    }
    else if (mode == MC_MODE_ENCODER_VOLTAGE)
    {
        s_diag.open_loop_ticks = 0U;
    }
}

/** @brief 根据当前诊断 mode 分发到 VF、Align 或 EncoderVoltage 快环。 */
void MotorControlDiag_RunFastLoop(MotorControlCState* mc)
{
    if (mc->mode == MC_MODE_VF_OPEN_LOOP)
    {
        run_vf_fast_loop(mc);
    }
    else if (mc->mode == MC_MODE_ALIGN_LOCK)
    {
        run_align_fast_loop(mc);
    }
    else if (mc->mode == MC_MODE_ENCODER_VOLTAGE)
    {
        run_encoder_voltage_fast_loop(mc);
    }
}

/** @brief 汇总诊断私有状态和主控制基础状态到诊断 watch。 */
void MotorControlDiag_FillWatch(const MotorControlCState* mc, MotorControlDiagWatch_t* out)
{
    out->open_loop_reset_count = s_diag.open_loop_reset_count;
    out->open_loop_ticks = s_diag.open_loop_ticks;
    out->open_loop_theta = s_diag.open_loop_theta;
    out->voltage_theta = mc->voltage_theta;
    out->encoder_raw_step = mc->encoder_raw_step;
    out->encoder_reject_step = mc->encoder_reject_step;
    out->encoder_reject_prev_raw = mc->encoder_reject_prev_raw;
    out->encoder_reject_raw = mc->encoder_reject_raw;
    out->encoder_reject_count = mc->encoder_reject_count;
    out->encoder_retry_count = mc->encoder_retry_count;
    out->encoder_retry_accept_count = mc->encoder_retry_accept_count;
    out->encoder_retry_raw = mc->encoder_retry_raw;
    out->align_done = s_diag.align_done;
    out->align_ticks = s_diag.align_ticks;
    out->align_theta = (uint16_t)MOT_ALIGN_THETA;
    out->align_raw = s_diag.align_raw;
    out->align_zero_trim = s_diag.align_zero_trim;
    out->align_encoder_elec = s_diag.align_encoder_elec;
    out->align_stage = s_diag.align_stage;
    out->align_pull_delta = s_diag.align_pull_delta;
    out->align_sample_count = s_diag.align_sample_count;
    out->align_delta_sum = s_diag.align_delta_sum;
    out->speed_fb_diff = mc->speed_fb_diff;
    out->speed_fb_diff_rpm = MotorControl_InternalSpeedCountsToRpm(mc->speed_fb_diff);
    out->speed_fb_ma600 = mc->speed_fb_ma600;
    out->speed_fb_ma600_rpm = MotorControl_InternalSpeedCountsToRpm(mc->speed_fb_ma600);
    out->ma600_speed_raw = mc->ma600_speed_raw;
    out->speed_fb_source = CTRL_SPD_FB_SOURCE;
    out->command_apply_count = mc->command_apply_count;
    out->command_enable = mc->command.enable;
    out->command_control_mode = mc->command.control_mode;
    out->command_vf_voltage = mc->command.vf_voltage;
    out->command_open_loop_speed_ref = mc->command.open_loop_speed_ref;
    out->command_speed_ref_rpm = mc->command.speed_ref_rpm;
    out->command_iq_limit = mc->command.iq_limit;
    out->command_current_v_limit = mc->command.current_v_limit;
    out->command_voltage_theta_offset = mc->command.voltage_theta_offset;
    out->command_speed_kp = mc->command.speed_kp;
    out->command_speed_ki = mc->command.speed_ki;
}

/** @brief 重置 VF 开环角并递增 reset 计数。 */
static void reset_open_loop_theta(void)
{
    s_diag.open_loop_ticks = 0U;
    s_diag.open_loop_theta = 0U;
    s_diag.open_loop_theta_acc = 0;
    s_diag.open_loop_reset_count++;
}

/** @brief 重置 Align 扫描状态，同时清空开环角累计。 */
static void reset_align_state(void)
{
    s_diag.align_ticks = 0U;
    s_diag.align_done = 0U;
    s_diag.align_raw = 0U;
    s_diag.align_zero_trim = 0;
    s_diag.align_encoder_elec = 0U;
    s_diag.align_stage = MC_ALIGN_STAGE_REV_FAST;
    s_diag.align_halfcycles = 0U;
    s_diag.align_theta_prev = 0;
    s_diag.align_first_delta = 0;
    s_diag.align_pull_delta = 0;
    s_diag.align_sample_count = 0U;
    s_diag.align_delta_sum = 0;
    s_diag.open_loop_ticks = 0U;
    s_diag.open_loop_theta = 0U;
    s_diag.open_loop_theta_acc = 0;
}

/** @brief 按命令 open_loop_speed_ref 推进 VF 开环角。 */
static uint16_t update_open_loop_theta(const MotorControlCommand_t* command)
{
    int32_t step = (command->open_loop_speed_ref * (int32_t)OL_SPEED_TO_THETA_STEP +
                    (1L << (OL_SPEED_TO_THETA_SHIFT - 1U))) >>
                   OL_SPEED_TO_THETA_SHIFT;
    s_diag.open_loop_theta_acc += step;
    s_diag.open_loop_theta =
        (uint16_t)((uint32_t)s_diag.open_loop_theta_acc & 0xFFFFUL);
    s_diag.open_loop_ticks++;
    return (uint16_t)((int32_t)s_diag.open_loop_theta * (int32_t)MOT_SENSOR_DIR);
}

/** @brief VF 诊断快环：生成开环角并输出 q 轴电压，同时观测编码器速度。 */
static void run_vf_fast_loop(MotorControlCState* mc)
{
    uint16_t theta;

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

    s_diag.open_loop_timeout_ticks = (uint32_t)mc->command.open_loop_timeout_ms * 2U;
    if ((s_diag.open_loop_timeout_ticks > 0U) &&
        (s_diag.open_loop_ticks >= s_diag.open_loop_timeout_ticks))
    {
        mc->state = MC_STATE_FAULT;
        mc->fault = MC_FAULT_OPEN_LOOP_TIMEOUT;
        MotorControl_InternalEnterSafeState(mc);
        return;
    }

    theta = update_open_loop_theta(&mc->command);
    MotorControl_InternalApplyVoltageVector(mc, 0, mc->command.vf_voltage, theta);
    if (++mc->speed_sample_div >= MC_SPEED_SAMPLE_DIV)
    {
        mc->speed_sample_div = 0U;
        if (MotorControl_InternalUpdateEncoderAngle(mc) != 0U)
        {
            (void)MotorControl_InternalUpdateEncoderSpeed(mc);
        }
    }
    mc->fast_loop_count++;
}

/** @brief Align 诊断快环：往返扫描电角度并采样零位 trim。 */
static void run_align_fast_loop(MotorControlCState* mc)
{
    int32_t speed = 0;
    uint16_t theta;

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

    if (s_diag.align_done != 0U)
    {
        MotorControl_InternalApplyVoltageVector(mc, 0, 0, mc->voltage_theta);
        mc->fast_loop_count++;
        return;
    }

    if (s_diag.align_stage == MC_ALIGN_STAGE_REV_FAST)
    {
        speed = -MOT_ALIGN_SCAN_FAST_SPEED;
    }
    else if (s_diag.align_stage == MC_ALIGN_STAGE_FWD_FAST)
    {
        speed = MOT_ALIGN_SCAN_FAST_SPEED;
    }
    else if (s_diag.align_stage == MC_ALIGN_STAGE_FWD_SAMPLE)
    {
        speed = MOT_ALIGN_SCAN_SLOW_SPEED;
    }
    else if (s_diag.align_stage == MC_ALIGN_STAGE_REV_SAMPLE)
    {
        speed = -MOT_ALIGN_SCAN_SLOW_SPEED;
    }
    else
    {
        finish_align_scan();
        mc->fast_loop_count++;
        return;
    }

    theta = update_align_theta(speed);
    MotorControl_InternalApplyVoltageVector(mc, MOT_ALIGN_SCAN_VD, 0, theta);

    if (MotorControl_InternalUpdateEncoderAngle(mc) != 0U)
    {
        s_diag.align_raw = mc->encoder_raw;
        s_diag.align_encoder_elec = mc->encoder_elec;
        if ((s_diag.align_stage == MC_ALIGN_STAGE_FWD_SAMPLE) ||
            (s_diag.align_stage == MC_ALIGN_STAGE_REV_SAMPLE))
        {
            sample_align_trim(mc->encoder_raw, theta);
        }
    }

    if (s_diag.align_ticks < 0xFFFFFFFFUL)
    {
        s_diag.align_ticks++;
    }

    count_align_halfcycle(theta);

    if ((s_diag.align_stage == MC_ALIGN_STAGE_REV_FAST) &&
        (s_diag.align_halfcycles >= MOT_ALIGN_SCAN_REV_FAST_HALFCYCLES))
    {
        set_align_stage(MC_ALIGN_STAGE_FWD_FAST);
    }
    else if ((s_diag.align_stage == MC_ALIGN_STAGE_FWD_FAST) &&
             (s_diag.align_halfcycles >= MOT_ALIGN_SCAN_FWD_FAST_HALFCYCLES))
    {
        set_align_stage(MC_ALIGN_STAGE_FWD_SAMPLE);
    }
    else if ((s_diag.align_stage == MC_ALIGN_STAGE_FWD_SAMPLE) &&
             (s_diag.align_halfcycles >= MOT_ALIGN_SCAN_SAMPLE_HALFCYCLES) &&
             (s_diag.align_sample_count >= MOT_ALIGN_SCAN_MIN_SAMPLES))
    {
        set_align_stage(MC_ALIGN_STAGE_REV_SAMPLE);
    }
    else if ((s_diag.align_stage == MC_ALIGN_STAGE_REV_SAMPLE) &&
             (s_diag.align_halfcycles >= MOT_ALIGN_SCAN_SAMPLE_HALFCYCLES) &&
             (s_diag.align_sample_count >= MOT_ALIGN_SCAN_MIN_SAMPLES))
    {
        finish_align_scan();
    }

    mc->fast_loop_count++;
}

/** @brief EncoderVoltage 诊断快环：使用编码器角度直接输出命令电压矢量。 */
static void run_encoder_voltage_fast_loop(MotorControlCState* mc)
{
    FocAlphaBeta_t current_ab;
    uint16_t voltage_theta;

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
    }

    mc->id_ref_active = mc->command.id_ref;
    mc->iq_ref_active = mc->command.iq_ref;
    voltage_theta = (uint16_t)(mc->encoder_elec + (uint16_t)mc->command.voltage_theta_offset);
    current_ab = foc_clarke_3phase(mc->current);
    mc->current_dq = foc_park(current_ab, voltage_theta);
    MotorControl_InternalApplyVoltageVector(mc, mc->command.id_ref, mc->command.iq_ref,
                                            voltage_theta);
    mc->fast_loop_count++;
}

/** @brief 切换 Align 阶段并重置半周期计数基准。 */
static void set_align_stage(uint8_t stage)
{
    s_diag.align_stage = stage;
    s_diag.align_halfcycles = 0U;
    s_diag.align_theta_prev =
        (int16_t)((int32_t)s_diag.open_loop_theta * (int32_t)MOT_SENSOR_DIR);
}

/** @brief 按指定开环速度推进 Align 扫描角。 */
static uint16_t update_align_theta(int32_t speed)
{
    int32_t step;

    step = speed * (int32_t)OL_SPEED_TO_THETA_STEP;
    if (step >= 0)
    {
        step += (1L << (OL_SPEED_TO_THETA_SHIFT - 1U));
    }
    else
    {
        step -= (1L << (OL_SPEED_TO_THETA_SHIFT - 1U));
    }
    step >>= OL_SPEED_TO_THETA_SHIFT;

    s_diag.open_loop_theta_acc += step;
    s_diag.open_loop_theta =
        (uint16_t)((uint32_t)s_diag.open_loop_theta_acc & 0xFFFFUL);
    s_diag.open_loop_ticks++;
    return (uint16_t)((int32_t)s_diag.open_loop_theta * (int32_t)MOT_SENSOR_DIR);
}

/** @brief 根据 raw 和目标电角度反推临时零位 trim。 */
static int16_t align_trim_from_raw(uint16_t raw, uint16_t target_theta)
{
    int32_t trim;

#if MOT_SENSOR_DIR > 0
    trim = (int32_t)target_theta - (int32_t)MOT_ELEC_ZERO -
           (int32_t)raw * (int32_t)MOT_SENSOR_ELEC;
#else
    trim = (int32_t)target_theta - (int32_t)MOT_ELEC_ZERO +
           (int32_t)raw * (int32_t)MOT_SENSOR_ELEC;
#endif
    return (int16_t)trim;
}

/** @brief 统计 Align 扫描角跨越符号的半周期次数。 */
static void count_align_halfcycle(uint16_t theta)
{
    const int16_t theta_now = (int16_t)theta;

    if (((s_diag.align_theta_prev > 0) && (theta_now < 0)) ||
        ((s_diag.align_theta_prev < 0) && (theta_now > 0)))
    {
        if (s_diag.align_halfcycles < 0xFFFFU)
        {
            s_diag.align_halfcycles++;
        }
    }
    s_diag.align_theta_prev = theta_now;
}

/** @brief 采样 Align trim，并以首样本为基准累计差值。 */
static void sample_align_trim(uint16_t raw, uint16_t theta)
{
    const int16_t trim = align_trim_from_raw(raw, theta);
    int16_t delta;

    s_diag.align_pull_delta = trim;
    if (s_diag.align_sample_count == 0U)
    {
        s_diag.align_first_delta = trim;
        s_diag.align_delta_sum = 0;
        s_diag.align_sample_count = 1U;
        return;
    }

    delta = (int16_t)((uint16_t)trim - (uint16_t)s_diag.align_first_delta);
    s_diag.align_delta_sum += (int32_t)delta;
    if (s_diag.align_sample_count < 0xFFFFU)
    {
        s_diag.align_sample_count++;
    }
}

/** @brief 结束 Align 扫描，计算平均 trim 并进入完成阶段。 */
static void finish_align_scan(void)
{
    int32_t average = s_diag.align_first_delta;

    if (s_diag.align_sample_count > 1U)
    {
        average += s_diag.align_delta_sum / (int32_t)(s_diag.align_sample_count - 1U);
    }

    s_diag.align_zero_trim = (int16_t)average;
    s_diag.align_done = 1U;
    set_align_stage(MC_ALIGN_STAGE_DONE);
}
