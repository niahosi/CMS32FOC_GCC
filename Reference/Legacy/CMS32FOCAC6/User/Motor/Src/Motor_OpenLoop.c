/**
 * @file Motor_OpenLoop.c
 * @brief 电机开环检测实现。
 * @details VF/IF 只用于早期验证功率级、相序、电流采样方向和开环拖动能力。
 *          正式电流环和速度环不依赖本文件作为运行路径。
 */

#include "Motor_OpenLoop.h"
#include "Motor_Foc.h"
#include "Board_Analog.h"
#include "Board_PWM.h"

typedef struct
{
    int16_t iu;
    int16_t iv;
    int16_t iw;
} OpenLoopCurrent_t;

/* 开环检测独立维护角度和 PI，避免污染正式闭环 Motor_Loop 状态。 */
static uint16_t s_theta;
static int32_t s_theta_acc;
static volatile int32_t s_speed_ref_cmd;
static int16_t s_vf_voltage;
static int16_t s_if_id_ref;
static int16_t s_if_iq_ref;
static uint32_t s_ticks;
static uint32_t s_timeout_ticks;
static uint8_t s_current_over_count;

static Foc_PiController_t s_pi_id;
static Foc_PiController_t s_pi_iq;

static uint8_t IsAbsOverS16(int16_t value, int16_t limit);
static int16_t ClampRefS16(int16_t value, int16_t limit);
static void ClearPi(void);
static void ReadCurrent(OpenLoopCurrent_t* cur);
static uint8_t IsCurrentSafe(const OpenLoopCurrent_t* cur);
static uint16_t UpdateTheta(void);
static void OutputVoltage(int16_t vd, int16_t vq, uint16_t theta);
static MotorOpenLoopFault_t RunVf(void);
static MotorOpenLoopFault_t RunIf(void);

void Motor_OpenLoopInit(void)
{
    /* 每次进入 VF/IF 前清掉开环角度、超时计数和 IF 电流 PI 积分。 */
    s_theta = 0u;
    s_theta_acc = 0;
    s_speed_ref_cmd = 0;
    s_vf_voltage = MOTOR_VF_VOLTAGE_DEFAULT;
    s_if_id_ref = MOTOR_IF_ID_REF_DEFAULT;
    s_if_iq_ref = MOTOR_IF_IQ_REF_DEFAULT;
    s_ticks = 0u;
    s_timeout_ticks = (uint32_t)MOTOR_OL_TIMEOUT_MS_DEFAULT * 2u;
    s_current_over_count = 0u;
    ClearPi();
}

MotorOpenLoopFault_t Motor_OpenLoopRun(uint8_t ctrl_mode)
{
    /* ctrl_mode 只接受 VF/IF，其他模式直接返回无故障。 */
    if (ctrl_mode == MOTOR_CTRL_VF)
    {
        return RunVf();
    }

    if (ctrl_mode == MOTOR_CTRL_IF)
    {
        return RunIf();
    }

    return MOTOR_OPEN_LOOP_FAULT_NONE;
}

void Motor_OpenLoopSetSpeedRef(int32_t speed_ref)
{
    s_speed_ref_cmd = Foc_ClampS32(speed_ref, -MOTOR_SPEED_REF_LIMIT, MOTOR_SPEED_REF_LIMIT);
}

void Motor_OpenLoopSetVfVoltage(int16_t voltage)
{
    s_vf_voltage = Foc_ClampS16(voltage, -MOTOR_CURRENT_V_LIMIT, MOTOR_CURRENT_V_LIMIT);
}

void Motor_OpenLoopSetIfCurrentRef(int16_t id_ref, int16_t iq_ref)
{
    s_if_id_ref = ClampRefS16(id_ref, MOTOR_CURRENT_REF_LIMIT);
    s_if_iq_ref = ClampRefS16(iq_ref, MOTOR_CURRENT_REF_LIMIT);
}

void Motor_OpenLoopSetTimeoutMs(uint16_t timeout_ms)
{
    s_timeout_ticks = (uint32_t)timeout_ms * 2u;
}

void Motor_OpenLoopFillSnap(MotorOpenLoopSnap_t* snap)
{
    if (snap == 0)
    {
        return;
    }

    snap->theta = s_theta;
    snap->vf_voltage = s_vf_voltage;
    snap->if_id_ref = s_if_id_ref;
    snap->if_iq_ref = s_if_iq_ref;
    snap->current_over_count = s_current_over_count;
}

static uint8_t IsAbsOverS16(int16_t value, int16_t limit)
{
    return (uint8_t)((value > limit) || (value < -limit));
}

static int16_t ClampRefS16(int16_t value, int16_t limit)
{
    return Foc_ClampS16(value, (int16_t)-limit, limit);
}

static void ClearPi(void)
{
    Foc_PiInit(&s_pi_id, MOTOR_CURRENT_KP, MOTOR_CURRENT_KI, -MOTOR_CURRENT_V_LIMIT,
               MOTOR_CURRENT_V_LIMIT);
    Foc_PiInit(&s_pi_iq, MOTOR_CURRENT_KP, MOTOR_CURRENT_KI, -MOTOR_CURRENT_V_LIMIT,
               MOTOR_CURRENT_V_LIMIT);
    Foc_PiSetShift(&s_pi_id, MOTOR_CURRENT_PI_SHIFT);
    Foc_PiSetShift(&s_pi_iq, MOTOR_CURRENT_PI_SHIFT);
}

static void ReadCurrent(OpenLoopCurrent_t* cur)
{
    cur->iu = Board_GetIuCnt();
    cur->iv = Board_GetIvCnt();
    cur->iw = Board_GetIwCnt();
}

static uint8_t IsCurrentSafe(const OpenLoopCurrent_t* cur)
{
    if (IsAbsOverS16(cur->iu, MOTOR_CURRENT_SAFE_LIMIT) == 0u &&
        IsAbsOverS16(cur->iv, MOTOR_CURRENT_SAFE_LIMIT) == 0u &&
        IsAbsOverS16(cur->iw, MOTOR_CURRENT_SAFE_LIMIT) == 0u)
    {
        s_current_over_count = 0u;
        return 1u;
    }

    if (s_current_over_count < 255u)
    {
        s_current_over_count++;
    }

    return (uint8_t)(s_current_over_count < MOTOR_CURRENT_OVER_LIMIT);
}

static uint16_t UpdateTheta(void)
{
    int32_t step;

    /*
     * ol_speed_ref 单位沿用 sensor count/s，转换成每次开环调用的 16 bit 电角度步进。
     * MOTOR_SENSOR_DIR 只修正控制方向，不改变 MA600 原始角度缓存。
     */
    step = (s_speed_ref_cmd * MOTOR_OL_SPEED_TO_THETA_STEP +
            (1l << (MOTOR_OL_SPEED_TO_THETA_SHIFT - 1u))) >>
           MOTOR_OL_SPEED_TO_THETA_SHIFT;
    s_theta_acc += step;
    s_theta = (uint16_t)((uint32_t)s_theta_acc & 0xFFFFu);

    return (uint16_t)((int32_t)s_theta * (int32_t)MOTOR_SENSOR_DIR);
}

static void OutputVoltage(int16_t vd, int16_t vq, uint16_t theta)
{
    int16_t v_alpha;
    int16_t v_beta;
    uint16_t duty_u;
    uint16_t duty_v;
    uint16_t duty_w;

    /* VF/IF 最终都输出 dq 电压矢量，经 InvPark/SVPWM 写入三相 PWM。 */
    (void)Foc_LimitDq(&vd, &vq, MOTOR_CURRENT_V_LIMIT);
    Foc_InvParkTransform(vd, vq, theta, &v_alpha, &v_beta);
    Foc_Svpwm(v_alpha, v_beta, PWM_PERIOD, &duty_u, &duty_v, &duty_w);

    Board_SetPwmDuty(duty_u, duty_v, duty_w);
    Board_EnablePwmOutput(1u);
}

static MotorOpenLoopFault_t RunVf(void)
{
    OpenLoopCurrent_t cur;
    uint16_t theta;

    /*
     * VF 不使用电流 PI，只输出随 theta 旋转的电压矢量。
     * 因此它能验证功率输出，但不能证明真实电角度零点正确。
     */
    ReadCurrent(&cur);
    if (Motor_IsAngleSafe() == 0u)
    {
        Board_ForcePwmOff();
        return MOTOR_OPEN_LOOP_FAULT_ANGLE;
    }

    if (IsCurrentSafe(&cur) == 0u)
    {
        Board_ForcePwmOff();
        return MOTOR_OPEN_LOOP_FAULT_CURRENT;
    }

    s_ticks++;
    if ((s_timeout_ticks > 0u) && (s_ticks >= s_timeout_ticks))
    {
        Board_ForcePwmOff();
        return MOTOR_OPEN_LOOP_FAULT_TIMEOUT;
    }

    theta = UpdateTheta();
    OutputVoltage(s_vf_voltage, 0, theta);

    return MOTOR_OPEN_LOOP_FAULT_NONE;
}

static MotorOpenLoopFault_t RunIf(void)
{
    OpenLoopCurrent_t cur;
    int16_t i_alpha;
    int16_t i_beta;
    int16_t id;
    int16_t iq;
    int16_t vd;
    int16_t vq;
    uint16_t theta;

    /*
     * IF 使用开环 theta 做 Park/InvPark，中间加入 id/iq PI。
     * 它能验证电流采样和 PI 链路，但方向主要仍由 ol_speed_ref 决定。
     */
    ReadCurrent(&cur);
    if (Motor_IsAngleSafe() == 0u)
    {
        Board_ForcePwmOff();
        return MOTOR_OPEN_LOOP_FAULT_ANGLE;
    }

    if (IsCurrentSafe(&cur) == 0u)
    {
        Board_ForcePwmOff();
        return MOTOR_OPEN_LOOP_FAULT_CURRENT;
    }

    s_ticks++;
    if ((s_timeout_ticks > 0u) && (s_ticks >= s_timeout_ticks))
    {
        Board_ForcePwmOff();
        return MOTOR_OPEN_LOOP_FAULT_TIMEOUT;
    }

    theta = UpdateTheta();
    Foc_ClarkeTransform(cur.iu, cur.iv, cur.iw, &i_alpha, &i_beta);
    Foc_ParkTransform(i_alpha, i_beta, theta, &id, &iq);
    vd = Foc_PiUpdate(&s_pi_id, s_if_id_ref, id);
    vq = Foc_PiUpdate(&s_pi_iq, s_if_iq_ref, iq);
    OutputVoltage(vd, vq, theta);

    return MOTOR_OPEN_LOOP_FAULT_NONE;
}
