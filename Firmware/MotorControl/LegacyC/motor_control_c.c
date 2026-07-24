#if 0
/*
 * Frozen legacy C implementation. The current firmware uses
 * Firmware/MotorControl/Core/core.cpp and split g_mc_* state objects.
 */
#include "BoardConfig.h"
#include "MotorControl.h"

#include "TuneConfig.h"
#include "foc_math.h"
#include "motor_control_internal.h"
#include "motor_control_vf.h"

#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_pwm.h"

#include "CMS32M6510.h"
#include <stdint.h>

volatile MotorControlCommand_t g_motor_cmd = {
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

static MotorControlCState s_mc;

static void copy_command(const volatile MotorControlCommand_t *src,
                         MotorControlCommand_t *dst);
static int16_t clamp_ref(int16_t value, int16_t limit);
static int16_t abs_limit(int16_t value, int16_t limit);
static int32_t clamp_s32_local(int32_t value, int32_t limit);
static int32_t rpm_to_speed_counts(int16_t rpm);
static void apply_runtime_commands(const MotorControlCommand_t *command);

/** @brief 初始化控制层全局状态、PI 控制器和 PWM 安全态。 */
void MotorControl_Init(void)
{
    s_mc.runtime.state = MC_STATE_IDLE;
    s_mc.runtime.fault = MC_FAULT_NONE;
    s_mc.runtime.enabled = 0U;
    s_mc.runtime.mode = MC_MODE_OFF;
    s_mc.runtime.pwm_output = 0U;
    g_motor_diag.command_apply_count = 0U;
    g_motor_diag.slow_loop_count = 0U;
    g_motor_diag.fast_loop_count = 0U;
    g_motor_diag.speed_reset_count = 0U;
    g_motor_diag.safe_state_count = 0U;
    g_motor_diag.speed_loop_count = 0U;
    g_motor_diag.speed_deadband_count = 0U;
    s_mc.closed_loop.current.loop_div = 0U;
    s_mc.closed_loop.speed.sample_div = 0U;
    MotorControl_EncoderReset(&s_mc);
    foc_pi_init(&s_mc.closed_loop.speed.pi,
                CTRL_SPD_KP,
                CTRL_SPD_KI,
                -CTRL_SPD_IQ_LIMIT,
                CTRL_SPD_IQ_LIMIT,
                CTRL_SPD_ERR_SHIFT);
    foc_pi_init(&s_mc.closed_loop.current.pi_d,
                CTRL_CUR_KP,
                CTRL_CUR_KI,
                -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT,
                CTRL_CUR_PI_SHIFT);
    foc_pi_init(&s_mc.closed_loop.current.pi_q,
                CTRL_CUR_KP,
                CTRL_CUR_KI,
                -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT,
                CTRL_CUR_PI_SHIFT);
    s_mc.command.current = (MCCurrentCommand_t){0,
                                                0,
                                                CTRL_SPD_IQ_LIMIT,
                                                CTRL_CUR_KP,
                                                CTRL_CUR_KI,
                                                CTRL_CUR_V_LIMIT,
                                                0,
                                                0};
    s_mc.command.speed =
        (MCSpeedCommand_t){0, CTRL_SPD_KP, CTRL_SPD_KI, CTRL_SPD_IQ_LIMIT};
    s_mc.command.vf = (MCVfCommand_t){OL_SPEED_REF, OL_VF_VOLTAGE, OL_TIMEOUT_MS};
    s_mc.closed_loop.current.phase = (FocPhaseCurrent_t){0, 0, 0};
    s_mc.closed_loop.current.dq = (FocDq_t){0, 0};
    s_mc.closed_loop.current.id_ref_active = 0;
    s_mc.closed_loop.current.iq_ref_active = 0;
    s_mc.closed_loop.output.voltage_ab = (FocAlphaBeta_t){0, 0};
    s_mc.closed_loop.output.voltage_dq = (FocDq_t){0, 0};
    s_mc.closed_loop.output.voltage_theta = 0U;
    s_mc.closed_loop.output.duty = (FocDuty_t){PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    s_mc.closed_loop.output.voltage_limited = 0U;
    MotorControlVf_Init();
    s_mc.check = (MotorControlCheck_t){0U, 1U, 1U, 0U};
    pwm_off();
}

/** @brief 主循环复制 Ozone 命令并完成限幅、模式切换和诊断 reset。 */
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t *command)
{
    MotorControlCommand_t next_command;
    uint8_t next_mode;

    if (command == 0)
    {
        return;
    }

    g_motor_diag.command_apply_count++;
    copy_command(command, &next_command);
    next_command.iq_limit = abs_limit(next_command.iq_limit, CTRL_CUR_REF_LIMIT);
    next_command.current_kp = foc_clamp_s16(next_command.current_kp, 0, 32767);
    next_command.current_ki = foc_clamp_s16(next_command.current_ki, 0, 32767);
    next_command.speed_kp = foc_clamp_s16(next_command.speed_kp, 0, 32767);
    next_command.speed_ki = foc_clamp_s16(next_command.speed_ki, 0, 32767);
    next_command.current_v_limit =
        abs_limit(next_command.current_v_limit, (int16_t)PWM_SVPWM_V_LIMIT);
    next_command.id_ref = clamp_ref(next_command.id_ref, CTRL_CUR_REF_LIMIT);
    next_command.iq_ref = clamp_ref(next_command.iq_ref, next_command.iq_limit);
    next_command.open_loop_speed_ref =
        clamp_s32_local(next_command.open_loop_speed_ref, CTRL_SPD_REF_LIMIT);
    if (next_command.speed_ref_rpm != 0)
    {
        next_command.speed_ref = rpm_to_speed_counts(next_command.speed_ref_rpm);
    }
    next_command.speed_ref =
        clamp_s32_local(next_command.speed_ref, CTRL_SPD_REF_LIMIT);
    next_command.vf_voltage = clamp_ref(next_command.vf_voltage, CTRL_CUR_V_LIMIT);

    next_mode = (next_command.enable != 0U) ? next_command.control_mode : MC_MODE_OFF;

    NVIC_DisableIRQ(ADC_IRQn);
    if (next_mode != s_mc.runtime.mode)
    {
        MotorControl_SpeedReset(&s_mc);
        MotorControl_CurrentReset(&s_mc);
        if (next_mode == MC_MODE_VF_OPEN_LOOP)
        {
            MotorControlVf_ResetForMode(next_mode);
        }
    }
    s_mc.runtime.enabled = (uint8_t)(next_command.enable != 0U);
    s_mc.runtime.mode = next_mode;
    apply_runtime_commands(&next_command);
    NVIC_EnableIRQ(ADC_IRQn);

    if ((s_mc.runtime.enabled != 0U) && (s_mc.runtime.mode == MC_MODE_VF_OPEN_LOOP))
    {
        curr_set_vf_voltage(next_command.vf_voltage);
    }
    else
    {
        curr_set_vf_voltage(0);
    }

    if (s_mc.runtime.enabled == 0U)
    {
        if ((s_mc.runtime.pwm_output != 0U) || (s_mc.runtime.state != MC_STATE_IDLE))
        {
            MotorControl_InternalEnterSafeState(&s_mc);
        }
        s_mc.runtime.state = MC_STATE_IDLE;
        s_mc.runtime.fault = MC_FAULT_NONE;
    }
}

/** @brief 主循环慢环，执行板级维护、安全检查和状态机更新。 */
void MotorControl_RunSlowLoop(void)
{
    s_mc.closed_loop.current.phase.u = curr_u();
    s_mc.closed_loop.current.phase.v = curr_v();
    s_mc.closed_loop.current.phase.w = curr_w();
    s_mc.check.current_ok = MotorControl_InternalCurrentOk(&s_mc);
    s_mc.check.pwm_off_safe = (pwm_is_off_safe() != 0U) ? 1U : 0U;
    s_mc.check.ma600_ok = 0U;
    s_mc.check.ready_closed_loop = 0U;

    if (s_mc.runtime.enabled == 0U)
    {
        g_motor_diag.slow_loop_count++;
        return;
    }

    if ((s_mc.runtime.mode == MC_MODE_VF_OPEN_LOOP) ||
        (s_mc.runtime.mode == MC_MODE_CURRENT) || (s_mc.runtime.mode == MC_MODE_SPEED))
    {
        s_mc.check.ma600_ok =
            ((s_mc.runtime.mode == MC_MODE_CURRENT) ||
             (s_mc.runtime.mode == MC_MODE_SPEED))
                ? (uint8_t)((s_mc.closed_loop.encoder.ok != 0U) ||
                            (s_mc.closed_loop.encoder.initialized == 0U))
                : 1U;
        s_mc.check.ready_closed_loop =
            (uint8_t)((s_mc.check.current_ok != 0U) &&
                      ((s_mc.runtime.mode == MC_MODE_VF_OPEN_LOOP) ||
                       (s_mc.closed_loop.encoder.ok != 0U) ||
                       (s_mc.closed_loop.encoder.initialized == 0U)));

        if (s_mc.check.ready_closed_loop != 0U)
        {
            if (s_mc.runtime.state != MC_STATE_CLOSED_LOOP)
            {
                if (s_mc.runtime.mode != MC_MODE_VF_OPEN_LOOP)
                {
                    MotorControl_SpeedReset(&s_mc);
                }
                MotorControl_CurrentReset(&s_mc);
            }
            s_mc.runtime.state = MC_STATE_CLOSED_LOOP;
            s_mc.runtime.fault = MC_FAULT_NONE;
        }
        else
        {
            const uint8_t should_enter_safe =
                (uint8_t)((s_mc.runtime.state != MC_STATE_FAULT) ||
                          (s_mc.runtime.pwm_output != 0U));
            s_mc.runtime.state = MC_STATE_FAULT;
            s_mc.runtime.fault = MC_FAULT_CURRENT;
            if ((s_mc.check.current_ok != 0U) &&
                ((s_mc.runtime.mode == MC_MODE_CURRENT) ||
                 (s_mc.runtime.mode == MC_MODE_SPEED)) &&
                (s_mc.check.ma600_ok == 0U))
            {
                s_mc.runtime.fault = MC_FAULT_ENCODER;
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
            (uint8_t)((s_mc.runtime.state != MC_STATE_FAULT) ||
                      (s_mc.runtime.pwm_output != 0U));
        s_mc.runtime.state = MC_STATE_FAULT;
        s_mc.runtime.fault = MC_FAULT_UNSUPPORTED_MODE;
        if (should_enter_safe != 0U)
        {
            MotorControl_InternalEnterSafeState(&s_mc);
        }
    }

    g_motor_diag.slow_loop_count++;
}

/** @brief ADC IRQ 快环入口，按控制模式运行 Current/Speed 或 VF 快环。 */
uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    const uint8_t sample_ready = bsp_adc_irq();
    if (sample_ready == 0U)
    {
        return 0U;
    }

    if ((s_mc.runtime.enabled != 0U) && (s_mc.runtime.state == MC_STATE_CLOSED_LOOP))
    {
        if (s_mc.runtime.mode == MC_MODE_VF_OPEN_LOOP)
        {
            MotorControlVf_RunFastLoop(&s_mc);
        }
        else if (s_mc.runtime.mode == MC_MODE_SPEED)
        {
            MotorControl_CurrentRunFastLoop(&s_mc, 1U);
        }
        else if (s_mc.runtime.mode == MC_MODE_CURRENT)
        {
            MotorControl_CurrentRunFastLoop(&s_mc, 0U);
        }
    }
    return 1U;
}

/** @brief 从 volatile 命令入口逐字段复制，避免快环直接读取 volatile 结构。 */
static void copy_command(const volatile MotorControlCommand_t *src,
                         MotorControlCommand_t *dst)
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

/** @brief 将完整外部命令拆成快环直接读取的小命令缓存。 */
static void apply_runtime_commands(const MotorControlCommand_t *command)
{
    s_mc.command.current.id_ref = command->id_ref;
    s_mc.command.current.iq_ref = command->iq_ref;
    s_mc.command.current.iq_limit = command->iq_limit;
    s_mc.command.current.current_kp = command->current_kp;
    s_mc.command.current.current_ki = command->current_ki;
    s_mc.command.current.current_v_limit = command->current_v_limit;
    s_mc.command.current.elec_zero_trim = command->elec_zero_trim;
    s_mc.command.current.voltage_theta_offset = command->voltage_theta_offset;

    s_mc.command.speed.speed_ref = command->speed_ref;
    s_mc.command.speed.speed_kp = command->speed_kp;
    s_mc.command.speed.speed_ki = command->speed_ki;
    s_mc.command.speed.iq_limit = command->iq_limit;

    s_mc.command.vf.open_loop_speed_ref = command->open_loop_speed_ref;
    s_mc.command.vf.vf_voltage = command->vf_voltage;
    s_mc.command.vf.open_loop_timeout_ms = command->open_loop_timeout_ms;
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
#endif
