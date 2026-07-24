#if 0
/*
 * Frozen legacy C implementation. The current firmware uses
 * Firmware/MotorControl/Core/output.cpp and split g_mc_* state objects.
 */
#include "BoardConfig.h"
#include "TuneConfig.h"
#include "foc_math.h"
#include "motor_control_internal.h"

#include "foc_pwm.h"
#include <stdint.h>

/** @brief 检查当前状态中的三相电流和 KCL 和是否在安全范围。 */
uint8_t MotorControl_InternalCurrentOk(MotorControlCState *mc)
{
    uint8_t ok = 1U;

    (void)mc;
#if (MOT_CHECK_CURR_CNT_LIMIT < 32767)
    ok = (uint8_t)(ok &&
                   (mc->closed_loop.current.phase.u >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                   (mc->closed_loop.current.phase.u <= MOT_CHECK_CURR_CNT_LIMIT));
    ok = (uint8_t)(ok &&
                   (mc->closed_loop.current.phase.v >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                   (mc->closed_loop.current.phase.v <= MOT_CHECK_CURR_CNT_LIMIT));
    ok = (uint8_t)(ok &&
                   (mc->closed_loop.current.phase.w >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                   (mc->closed_loop.current.phase.w <= MOT_CHECK_CURR_CNT_LIMIT));
#endif
#if (MOT_CHECK_SUM_CNT_LIMIT < 32767)
    {
        const int16_t sum = (int16_t)(mc->closed_loop.current.phase.u +
                                      mc->closed_loop.current.phase.v +
                                      mc->closed_loop.current.phase.w);
        ok = (uint8_t)(ok && (sum >= -MOT_CHECK_SUM_CNT_LIMIT) &&
                       (sum <= MOT_CHECK_SUM_CNT_LIMIT));
    }
#endif
    return ok;
}

/** @brief 获取当前电流环/诊断电压限幅，命令未设置时回退默认值。 */
int16_t MotorControl_InternalVoltageLimit(const MotorControlCState *mc)
{
    int16_t limit = mc->command.current.current_v_limit;

    if (limit <= 0)
    {
        limit = CTRL_CUR_V_LIMIT;
    }
    return limit;
}

/** @brief 关闭 PWM 并清空输出/速度 PI 状态。 */
void MotorControl_InternalEnterSafeState(MotorControlCState *mc)
{
    g_motor_diag.safe_state_count++;
    pwm_off();
    mc->runtime.pwm_output = 0U;
    mc->closed_loop.output.voltage_dq = (FocDq_t){0, 0};
    mc->closed_loop.output.voltage_ab = (FocAlphaBeta_t){0, 0};
    mc->closed_loop.output.voltage_theta = 0U;
    mc->closed_loop.speed.iq_ref = 0;
    mc->closed_loop.speed.err_rpm = 0;
    mc->closed_loop.output.duty = (FocDuty_t){PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    foc_pi_reset(&mc->closed_loop.speed.pi);
}

/** @brief 统一输出 dq 电压矢量，含限幅、反 Park、SVPWM 和 PWM 使能。 */
void MotorControl_InternalApplyVoltageVector(MotorControlCState *mc,
                                             int16_t vd,
                                             int16_t vq,
                                             uint16_t theta)
{
    const int16_t v_limit = MotorControl_InternalVoltageLimit(mc);

    mc->closed_loop.output.voltage_theta = theta;
    mc->closed_loop.output.voltage_dq.d = vd;
    mc->closed_loop.output.voltage_dq.q = vq;
    mc->closed_loop.output.voltage_limited =
        foc_limit_dq(&mc->closed_loop.output.voltage_dq, v_limit);
    mc->closed_loop.output.voltage_ab =
        foc_inv_park(mc->closed_loop.output.voltage_dq, theta);
    mc->closed_loop.output.duty = foc_svpwm(mc->closed_loop.output.voltage_ab,
                                            PWM_PERIOD,
                                            PWM_DUTY_MIN,
                                            PWM_DUTY_MAX);
    pwm_set_duty(mc->closed_loop.output.duty.u,
                 mc->closed_loop.output.duty.v,
                 mc->closed_loop.output.duty.w);
    (void)pwm_enable(1U);
    mc->runtime.pwm_output = 1U;
}
#endif
