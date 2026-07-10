#include "motor_control_internal.h"

#include "foc_pwm.h"

/** @brief 检查当前状态中的三相电流和 KCL 和是否在安全范围。 */
uint8_t MotorControl_InternalCurrentOk(MotorControlCState* mc)
{
    uint8_t ok = 1U;

    (void)mc;
#if (MOT_CHECK_CURR_CNT_LIMIT < 32767)
    ok = (uint8_t)(ok && (mc->current.u >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                  (mc->current.u <= MOT_CHECK_CURR_CNT_LIMIT));
    ok = (uint8_t)(ok && (mc->current.v >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                  (mc->current.v <= MOT_CHECK_CURR_CNT_LIMIT));
    ok = (uint8_t)(ok && (mc->current.w >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                  (mc->current.w <= MOT_CHECK_CURR_CNT_LIMIT));
#endif
#if (MOT_CHECK_SUM_CNT_LIMIT < 32767)
    {
        const int16_t sum = (int16_t)(mc->current.u + mc->current.v + mc->current.w);
        ok = (uint8_t)(ok && (sum >= -MOT_CHECK_SUM_CNT_LIMIT) &&
                      (sum <= MOT_CHECK_SUM_CNT_LIMIT));
    }
#endif
    return ok;
}

/** @brief 获取当前电流环/诊断电压限幅，命令未设置时回退默认值。 */
int16_t MotorControl_InternalVoltageLimit(const MotorControlCState* mc)
{
    int16_t limit = mc->command.current_v_limit;

    if (limit <= 0)
    {
        limit = CTRL_CUR_V_LIMIT;
    }
    return limit;
}

/** @brief 关闭 PWM 并清空输出/速度 PI 状态。 */
void MotorControl_InternalEnterSafeState(MotorControlCState* mc)
{
    mc->safe_state_count++;
    pwm_off();
    mc->pwm_output = 0U;
    mc->voltage_dq = (FocDq_t){0, 0};
    mc->voltage_ab = (FocAlphaBeta_t){0, 0};
    mc->voltage_theta = 0U;
    mc->speed_iq_ref = 0;
    mc->speed_err_rpm = 0;
    mc->duty = (FocDuty_t){PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    foc_pi_reset(&mc->speed_pi);
}

/** @brief 统一输出 dq 电压矢量，含限幅、反 Park、SVPWM 和 PWM 使能。 */
void MotorControl_InternalApplyVoltageVector(MotorControlCState* mc, int16_t vd, int16_t vq,
                                             uint16_t theta)
{
    const int16_t v_limit = MotorControl_InternalVoltageLimit(mc);

    mc->voltage_theta = theta;
    mc->voltage_dq.d = vd;
    mc->voltage_dq.q = vq;
    mc->voltage_limited = foc_limit_dq(&mc->voltage_dq, v_limit);
    mc->voltage_ab = foc_inv_park(mc->voltage_dq, theta);
    mc->duty = foc_svpwm(mc->voltage_ab, PWM_PERIOD, PWM_DUTY_MIN, PWM_DUTY_MAX);
    pwm_set_duty(mc->duty.u, mc->duty.v, mc->duty.w);
    (void)pwm_enable(1U);
    mc->pwm_output = 1U;
}
