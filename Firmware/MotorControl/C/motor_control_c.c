#include "MotorControl.h"

#include "motor_control_internal.h"
#include "motor_control_vf.h"

#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_pwm.h"

#include "CMS32M6510.h"

volatile MotorControlCommand_t g_motor_command = {
    .enable = 0U,
    .control_mode = 0U,
    .id_ref = 0,
    .iq_ref = 0,
    .speed_ref = 0,
    .speed_ref_rpm = 0,
    .iq_limit = CTRL_SPD_IQ_LIMIT,
    .current_kp = CTRL_CUR_KP,
    .current_ki = CTRL_CUR_KI,
    .speed_kp = CTRL_SPD_KP,
    .speed_ki = CTRL_SPD_KI,
    .current_v_limit = CTRL_CUR_V_LIMIT,
    .vf_voltage = OL_VF_VOLTAGE,
    .open_loop_speed_ref = OL_SPEED_REF,
    .if_id_ref = OL_IF_ID_REF,
    .if_iq_ref = OL_IF_IQ_REF,
    .open_loop_timeout_ms = OL_TIMEOUT_MS,
    .elec_zero_trim = 0,
    .voltage_theta_offset = 0,
};

volatile MotorControlWatch_t g_motor_status;

static MotorControlCState s_mc;

static void copy_command(const volatile MotorControlCommand_t* src, MotorControlCommand_t* dst);
static int16_t clamp_ref(int16_t value, int16_t limit);
static int16_t abs_limit(int16_t value, int16_t limit);
static int32_t clamp_s32_local(int32_t value, int32_t limit);
static int32_t rpm_to_speed_counts(int16_t rpm);

/** @brief 初始化控制层全局状态、PI 控制器和 PWM 安全态。 */
void MotorControl_Init(void)
{
    s_mc.state = MC_STATE_IDLE;
    s_mc.fault = MC_FAULT_NONE;
    s_mc.enabled = 0U;
    s_mc.mode = MC_MODE_OFF;
    s_mc.pwm_output = 0U;
    s_mc.command_apply_count = 0U;
    s_mc.slow_loop_count = 0U;
    s_mc.fast_loop_count = 0U;
    s_mc.speed_reset_count = 0U;
    s_mc.safe_state_count = 0U;
    s_mc.speed_loop_count = 0U;
    s_mc.speed_deadband_count = 0U;
    s_mc.current_loop_div = 0U;
    s_mc.speed_sample_div = 0U;
    MotorControl_EncoderReset(&s_mc);
    foc_pi_init(&s_mc.speed_pi, CTRL_SPD_KP, CTRL_SPD_KI, -CTRL_SPD_IQ_LIMIT,
                CTRL_SPD_IQ_LIMIT, CTRL_SPD_ERR_SHIFT);
    foc_pi_init(&s_mc.current_pi_d, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);
    foc_pi_init(&s_mc.current_pi_q, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);
    s_mc.current = (FocPhaseCurrent_t){0, 0, 0};
    s_mc.current_dq = (FocDq_t){0, 0};
    s_mc.id_ref_active = 0;
    s_mc.iq_ref_active = 0;
    s_mc.voltage_ab = (FocAlphaBeta_t){0, 0};
    s_mc.voltage_dq = (FocDq_t){0, 0};
    s_mc.voltage_theta = 0U;
    s_mc.duty = (FocDuty_t){PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    s_mc.voltage_limited = 0U;
    MotorControlVf_Init();
    s_mc.check = (MotorControlCheck_t){0U, 1U, 1U, 0U};
    pwm_off();
}

/** @brief 主循环复制 Ozone 命令并完成限幅、模式切换和诊断 reset。 */
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command)
{
    MotorControlCommand_t next_command;
    uint8_t next_mode;

    if (command == 0)
    {
        return;
    }

    s_mc.command_apply_count++;
    copy_command(command, &next_command);
    next_command.iq_limit = abs_limit(next_command.iq_limit, CTRL_CUR_REF_LIMIT);
    next_command.current_kp = foc_clamp_s16(next_command.current_kp, 0, 32767);
    next_command.current_ki = foc_clamp_s16(next_command.current_ki, 0, 32767);
    next_command.speed_kp = foc_clamp_s16(next_command.speed_kp, 0, 32767);
    next_command.speed_ki = foc_clamp_s16(next_command.speed_ki, 0, 32767);
    next_command.current_v_limit = abs_limit(next_command.current_v_limit,
                                             (int16_t)PWM_SVPWM_V_LIMIT);
    next_command.id_ref = clamp_ref(next_command.id_ref, CTRL_CUR_REF_LIMIT);
    next_command.iq_ref = clamp_ref(next_command.iq_ref, next_command.iq_limit);
    next_command.open_loop_speed_ref =
        clamp_s32_local(next_command.open_loop_speed_ref, CTRL_SPD_REF_LIMIT);
    if (next_command.speed_ref_rpm != 0)
    {
        next_command.speed_ref = rpm_to_speed_counts(next_command.speed_ref_rpm);
    }
    next_command.speed_ref = clamp_s32_local(next_command.speed_ref, CTRL_SPD_REF_LIMIT);
    next_command.vf_voltage = clamp_ref(next_command.vf_voltage, CTRL_CUR_V_LIMIT);

    next_mode = (next_command.enable != 0U) ? next_command.control_mode : MC_MODE_OFF;

    NVIC_DisableIRQ(ADC_IRQn);
    if (next_mode != s_mc.mode)
    {
        MotorControl_SpeedReset(&s_mc);
        MotorControl_CurrentReset(&s_mc);
        if (next_mode == MC_MODE_VF_OPEN_LOOP)
        {
            MotorControlVf_ResetForMode(next_mode);
        }
    }
    s_mc.enabled = (uint8_t)(next_command.enable != 0U);
    s_mc.mode = next_mode;
    s_mc.command = next_command;
    NVIC_EnableIRQ(ADC_IRQn);

    if ((s_mc.enabled != 0U) && (s_mc.mode == MC_MODE_VF_OPEN_LOOP))
    {
        curr_set_vf_voltage(next_command.vf_voltage);
    }
    else
    {
        curr_set_vf_voltage(0);
    }

    if (s_mc.enabled == 0U)
    {
        if ((s_mc.pwm_output != 0U) || (s_mc.state != MC_STATE_IDLE))
        {
            MotorControl_InternalEnterSafeState(&s_mc);
        }
        s_mc.state = MC_STATE_IDLE;
        s_mc.fault = MC_FAULT_NONE;
    }
}

/** @brief 主循环慢环，执行板级维护、安全检查和状态机更新。 */
void MotorControl_RunSlowLoop(void)
{
    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    s_mc.check.current_ok = MotorControl_InternalCurrentOk(&s_mc);
    s_mc.check.pwm_off_safe = (pwm_is_off_safe() != 0U) ? 1U : 0U;
    s_mc.check.ma600_ok = 0U;
    s_mc.check.ready_closed_loop = 0U;

    if (s_mc.enabled == 0U)
    {
        s_mc.slow_loop_count++;
        return;
    }

    if ((s_mc.mode == MC_MODE_VF_OPEN_LOOP) || (s_mc.mode == MC_MODE_CURRENT) ||
        (s_mc.mode == MC_MODE_SPEED))
    {
        s_mc.check.ma600_ok =
            ((s_mc.mode == MC_MODE_CURRENT) || (s_mc.mode == MC_MODE_SPEED))
                ? (uint8_t)((s_mc.encoder_ok != 0U) || (s_mc.encoder_initialized == 0U))
                : 1U;
        s_mc.check.ready_closed_loop =
            (uint8_t)((s_mc.check.current_ok != 0U) &&
                      ((s_mc.mode == MC_MODE_VF_OPEN_LOOP) ||
                       (s_mc.encoder_ok != 0U) || (s_mc.encoder_initialized == 0U)));

        if (s_mc.check.ready_closed_loop != 0U)
        {
            if (s_mc.state != MC_STATE_CLOSED_LOOP)
            {
                if (s_mc.mode != MC_MODE_VF_OPEN_LOOP)
                {
                    MotorControl_SpeedReset(&s_mc);
                }
                MotorControl_CurrentReset(&s_mc);
            }
            s_mc.state = MC_STATE_CLOSED_LOOP;
            s_mc.fault = MC_FAULT_NONE;
        }
        else
        {
            const uint8_t should_enter_safe =
                (uint8_t)((s_mc.state != MC_STATE_FAULT) || (s_mc.pwm_output != 0U));
            s_mc.state = MC_STATE_FAULT;
            if (s_mc.check.current_ok == 0U)
            {
                s_mc.fault = MC_FAULT_CURRENT;
            }
            else if (((s_mc.mode == MC_MODE_CURRENT) || (s_mc.mode == MC_MODE_SPEED)) &&
                     (s_mc.check.ma600_ok == 0U))
            {
                s_mc.fault = MC_FAULT_ENCODER;
            }
            else
            {
                s_mc.fault = MC_FAULT_CURRENT;
            }
            if (should_enter_safe != 0U)
            {
                MotorControl_InternalEnterSafeState(&s_mc);
            }
        }
    }
    else
    {
        const uint8_t should_enter_safe =
            (uint8_t)((s_mc.state != MC_STATE_FAULT) || (s_mc.pwm_output != 0U));
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_UNSUPPORTED_MODE;
        if (should_enter_safe != 0U)
        {
            MotorControl_InternalEnterSafeState(&s_mc);
        }
    }

    s_mc.slow_loop_count++;
}

/** @brief ADC IRQ 快环入口，按控制模式运行 Current/Speed 或 VF 快环。 */
uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    const uint8_t sample_ready = bsp_adc_irq();
    if (sample_ready == 0U)
    {
        return 0U;
    }

    if ((s_mc.enabled != 0U) && (s_mc.state == MC_STATE_CLOSED_LOOP))
    {
        if (s_mc.mode == MC_MODE_VF_OPEN_LOOP)
        {
            MotorControlVf_RunFastLoop(&s_mc);
        }
        else if (s_mc.mode == MC_MODE_SPEED)
        {
            MotorControl_CurrentRunFastLoop(&s_mc, 1U);
        }
        else if (s_mc.mode == MC_MODE_CURRENT)
        {
            MotorControl_CurrentRunFastLoop(&s_mc, 0U);
        }
    }
    return 1U;
}

/** @brief 复制当前主线 watch 到普通内存。 */
void MotorControl_GetWatch(MotorControlWatch_t* out)
{
    if (out == 0)
    {
        return;
    }
    MotorControl_WatchFill(&s_mc, out);
}

/** @brief 复制当前主线 watch 到 volatile Ozone 观察变量。 */
void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out)
{
    MotorControlWatch_t snapshot;

    if (out == 0)
    {
        return;
    }

    MotorControl_WatchFill(&s_mc, &snapshot);
    MotorControl_WatchCopyToVolatile(out, &snapshot);
}

/** @brief 从 volatile 命令入口逐字段复制，避免快环直接读取 volatile 结构。 */
static void copy_command(const volatile MotorControlCommand_t* src, MotorControlCommand_t* dst)
{
    dst->enable = src->enable;
    dst->control_mode = src->control_mode;
    dst->id_ref = src->id_ref;
    dst->iq_ref = src->iq_ref;
    dst->speed_ref = src->speed_ref;
    dst->speed_ref_rpm = src->speed_ref_rpm;
    dst->iq_limit = src->iq_limit;
    dst->current_kp = src->current_kp;
    dst->current_ki = src->current_ki;
    dst->speed_kp = src->speed_kp;
    dst->speed_ki = src->speed_ki;
    dst->current_v_limit = src->current_v_limit;
    dst->open_loop_speed_ref = src->open_loop_speed_ref;
    dst->vf_voltage = src->vf_voltage;
    dst->if_id_ref = src->if_id_ref;
    dst->if_iq_ref = src->if_iq_ref;
    dst->open_loop_timeout_ms = src->open_loop_timeout_ms;
    dst->elec_zero_trim = src->elec_zero_trim;
    dst->voltage_theta_offset = src->voltage_theta_offset;
}

/** @brief 将有符号给定限制到正负 limit。 */
static int16_t clamp_ref(int16_t value, int16_t limit)
{
    return foc_clamp_s16(value, (int16_t)-limit, limit);
}

/** @brief 将限幅参数转为非负并限制最大值。 */
static int16_t abs_limit(int16_t value, int16_t limit)
{
    if (value < 0)
    {
        value = (int16_t)-value;
    }
    return foc_clamp_s16(value, 0, limit);
}

/** @brief 将 int32 给定限制到正负 limit。 */
static int32_t clamp_s32_local(int32_t value, int32_t limit)
{
    return foc_clamp_s32(value, -limit, limit);
}

/** @brief 将机械 rpm 转为编码器电角 count/s。 */
static int32_t rpm_to_speed_counts(int16_t rpm)
{
    return ((int32_t)rpm * MC_SPEED_COUNTS_PER_REV) / 60L;
}
