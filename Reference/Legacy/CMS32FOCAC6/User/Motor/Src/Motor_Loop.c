/**
 * @file Motor_Loop.c
 * @brief 电机闭环控制模块实现
 * @details 实现电流环、速度估算、速度环和闭环前对齐等功能。
 *          由 Motor_FastLoop() 调用，执行实际的 FOC 闭环计算。
 */

#include "Motor_Loop.h"
#include "Motor.h"
#include "Motor_Foc.h"
#include "Motor_OpenLoop.h"
#include "Board.h"
#include "Board_Analog.h"
#include "Board_PWM.h"

/*===========================================================================
 * 内部数据结构
 *===========================================================================*/

typedef struct
{
    int16_t iu;
    int16_t iv;
    int16_t iw;
    int16_t sum;
} MotorCurrent_t;

typedef struct
{
    uint8_t active;
    uint8_t output_ready;

    uint16_t angle_raw;
    uint16_t angle_elec;

    MotorCurrent_t cur;

    int16_t i_alpha;
    int16_t i_beta;
    int16_t id;
    int16_t iq;

    int16_t id_ref;
    int16_t iq_ref;
    int16_t vd;
    int16_t vq;
    int16_t kp;
    int16_t ki;
    int16_t v_limit;
    int16_t v_alpha;
    int16_t v_beta;
    uint8_t v_limited;

    uint16_t duty_u;
    uint16_t duty_v;
    uint16_t duty_w;

    uint32_t fast_count;
} MotorLoop_t;

/**
 * @brief 速度环内部状态。
 * @details 位置差分在低频节拍估速，速度 PI 输出 iq_ref 给电流环使用。
 */
typedef struct
{
    int32_t ref;
    int32_t fb_raw;
    int32_t fb_filt;
    int32_t prev_pos;
    int32_t pos_delta;
    int32_t integral;
    int32_t error;
    int16_t iq_ref;
    int16_t iq_limit;
    uint16_t div;
    uint32_t est_count;
    uint32_t loop_count;
} MotorSpeedLoop_t;

/*===========================================================================
 * 静态变量
 *===========================================================================*/

static MotorLoop_t s_loop;
static MotorSpeedLoop_t s_speed;

static Foc_PiController_t s_pi_id;
static Foc_PiController_t s_pi_iq;
static uint8_t s_current_over_count;

/* 控制参数（由 Motor.c 通过 setter 设置） */
static volatile int16_t s_id_ref_cmd;
static volatile int16_t s_iq_ref_cmd;
static volatile int32_t s_speed_ref_cmd;
static volatile int16_t s_iq_limit_cmd;
static volatile int16_t s_current_kp_cmd;
static volatile int16_t s_current_ki_cmd;
static volatile int16_t s_current_v_limit_cmd;

/* 对齐参数 */
static uint16_t s_align_ticks;
static uint8_t  s_align_phase;  /* 0=校准阶段, 1=对齐阶段 */

/*===========================================================================
 * 工具函数
 *===========================================================================*/

static uint8_t IsAbsOverS16(int16_t value, int16_t limit)
{
    return (uint8_t)((value > limit) || (value < -limit));
}

static uint8_t IsAbsOverS32(int32_t value, int32_t limit)
{
    return (uint8_t)((value > limit) || (value < -limit));
}

static int32_t ScaleDownS32(int32_t value, uint8_t shift)
{
    if (value >= 0)
    {
        return value >> shift;
    }
    return -((-value) >> shift);
}

static int16_t ClampRefS16(int16_t value, int16_t limit)
{
    return Foc_ClampS16(value, (int16_t)-limit, limit);
}

static int16_t ClampPiGainS16(int16_t value)
{
    return Foc_ClampS16(value, 0, 32767);
}

static int16_t ClampVoltageLimitS16(int16_t value)
{
    if (value < 0)
    {
        value = (int16_t)-value;
    }
    return Foc_ClampS16(value, 0, PWM_PERIOD / 2);
}

/*===========================================================================
 * 初始化/清除函数
 *===========================================================================*/

static void ClearPi(void)
{
    /* 电流环 PI 使用固定点参数，输出直接作为 vd/vq 命令幅值。 */
    Foc_PiInit(&s_pi_id, s_current_kp_cmd, s_current_ki_cmd, (int16_t)-s_current_v_limit_cmd,
               s_current_v_limit_cmd);
    Foc_PiInit(&s_pi_iq, s_current_kp_cmd, s_current_ki_cmd, (int16_t)-s_current_v_limit_cmd,
               s_current_v_limit_cmd);
    Foc_PiSetShift(&s_pi_id, MOTOR_CURRENT_PI_SHIFT);
    Foc_PiSetShift(&s_pi_iq, MOTOR_CURRENT_PI_SHIFT);

    s_loop.kp = s_current_kp_cmd;
    s_loop.ki = s_current_ki_cmd;
    s_loop.v_limit = s_current_v_limit_cmd;
}

static void ClearSpeedLoop(void)
{
    /* prev_pos 取当前累计位置，避免刚进入速度环时把历史位移当成速度。 */
    s_speed.ref = 0;
    s_speed.fb_raw = 0;
    s_speed.fb_filt = 0;
    s_speed.prev_pos = Motor_GetAnglePos();
    s_speed.pos_delta = 0;
    s_speed.integral = 0;
    s_speed.error = 0;
    s_speed.iq_ref = 0;
    s_speed.iq_limit = s_iq_limit_cmd;
    s_speed.div = 0;
    s_speed.est_count = 0;
    s_speed.loop_count = 0;
}

static void ClearLoop(void)
{
    s_loop.active = 0;
    s_loop.output_ready = 0;
    s_loop.angle_raw = 0;
    s_loop.angle_elec = 0;

    s_loop.cur.iu = 0;
    s_loop.cur.iv = 0;
    s_loop.cur.iw = 0;
    s_loop.cur.sum = 0;

    s_loop.i_alpha = 0;
    s_loop.i_beta = 0;
    s_loop.id = 0;
    s_loop.iq = 0;

    s_loop.id_ref = 0;
    s_loop.iq_ref = 0;
    s_loop.vd = 0;
    s_loop.vq = 0;
    s_loop.kp = s_current_kp_cmd;
    s_loop.ki = s_current_ki_cmd;
    s_loop.v_limit = s_current_v_limit_cmd;
    s_loop.v_alpha = 0;
    s_loop.v_beta = 0;
    s_loop.v_limited = 0u;
    s_loop.fast_count = 0;
    s_current_over_count = 0;

    ClearSpeedLoop();
    ClearPi();
}

/*===========================================================================
 * 电流采样和安全检查
 *===========================================================================*/

static void ReadCurrent(MotorCurrent_t* cur)
{
    cur->iu = Board_GetIuCnt();
    cur->iv = Board_GetIvCnt();
    cur->iw = Board_GetIwCnt();
    cur->sum = Board_GetIuvwSum();
}

static uint8_t IsRunCurrentSafe(const MotorCurrent_t* cur)
{
    /*
     * 运行中过流采用连续计数滤波。
     * 偶发单次尖峰不立即 fault，连续超过阈值才返回不安全。
     */
    if (IsAbsOverS16(cur->iu, MOTOR_CURRENT_SAFE_LIMIT) == 0u &&
        IsAbsOverS16(cur->iv, MOTOR_CURRENT_SAFE_LIMIT) == 0u &&
        IsAbsOverS16(cur->iw, MOTOR_CURRENT_SAFE_LIMIT) == 0u)
    {
        s_current_over_count = 0;
        return 1u;
    }

    if (s_current_over_count < 255u)
    {
        s_current_over_count++;
    }

    return (uint8_t)(s_current_over_count < MOTOR_CURRENT_OVER_LIMIT);
}

/*===========================================================================
 * 角度辅助函数
 *===========================================================================*/

static void SnapshotAngleToLoop(void)
{
    /* 快环使用 Motor 层缓存角度，不在这里发起 MA600 SPI 事务。 */
    s_loop.angle_raw = Motor_GetAngleRaw();
    s_loop.angle_elec = Motor_GetAngleElec();
}

/*===========================================================================
 * 速度估算和速度环
 *===========================================================================*/

static void UpdateSpeedEstimateFromPos(void)
{
    int32_t control_delta;
    int32_t diff;
    int32_t raw;

    /*
     * 静止时 MA600 会有少量抖动，不能用单次 angle_delta 直接乘 20 kHz。
     * 这里在速度环节拍做窗口差分，再加死区和 IIR 滤波。
     */
    s_speed.pos_delta = Motor_GetAnglePos() - s_speed.prev_pos;
    s_speed.prev_pos = Motor_GetAnglePos();

    control_delta = s_speed.pos_delta * (int32_t)MOTOR_SENSOR_DIR;
    if (IsAbsOverS32(control_delta, MOTOR_SPEED_POS_DEADBAND) == 0u)
    {
        raw = 0;
    }
    else
    {
        raw = control_delta * MOTOR_SPEED_EST_HZ;
    }

    s_speed.fb_raw = raw;
    diff = s_speed.fb_raw - s_speed.fb_filt;
    s_speed.fb_filt += ScaleDownS32(diff, MOTOR_SPEED_FILTER_SHIFT);

    if (IsAbsOverS32(s_speed.fb_filt, MOTOR_SPEED_ZERO_SNAP) == 0u)
    {
        s_speed.fb_filt = 0;
    }

    s_speed.est_count++;
}

static int16_t RunSpeedPi(void)
{
    int32_t error;
    int32_t error_scaled;
    int32_t integral_new;
    int32_t output_unclamped;
    int32_t output;
    int16_t limit;

    limit = s_speed.iq_limit;
    if (limit < 0)
    {
        limit = (int16_t)-limit;
    }
    if (limit > MOTOR_CURRENT_REF_LIMIT)
    {
        limit = MOTOR_CURRENT_REF_LIMIT;
    }

    s_speed.iq_limit = limit;

    /*
     * 速度环只生成 iq_ref，不直接写 PWM。
     * iq_limit 是速度环输出的硬约束，用于早期调试限制力矩。
     */
    error = s_speed.ref - s_speed.fb_filt;
    error = Foc_ClampS32(error, -MOTOR_SPEED_REF_LIMIT, MOTOR_SPEED_REF_LIMIT);
    s_speed.error = error;

    error_scaled = ScaleDownS32(error, MOTOR_SPEED_ERR_SHIFT);
    error_scaled = Foc_ClampS32(error_scaled, -32767, 32767);

    integral_new = s_speed.integral + error_scaled;
    integral_new = Foc_ClampS32(integral_new, -32767, 32767);

    output_unclamped = (int32_t)MOTOR_SPEED_KP * error_scaled +
                       (int32_t)MOTOR_SPEED_KI * integral_new;
    output = Foc_ClampS32(output_unclamped, (int32_t)-limit, (int32_t)limit);

    if ((output == output_unclamped) || (MOTOR_SPEED_KI == 0) ||
        ((output_unclamped > limit) && (error_scaled < 0)) ||
        ((output_unclamped < -limit) && (error_scaled > 0)))
    {
        s_speed.integral = integral_new;
    }

    output_unclamped = (int32_t)MOTOR_SPEED_KP * error_scaled +
                       (int32_t)MOTOR_SPEED_KI * s_speed.integral;
    output = Foc_ClampS32(output_unclamped, (int32_t)-limit, (int32_t)limit);

    s_speed.iq_ref = (int16_t)output;
    s_speed.loop_count++;
    return s_speed.iq_ref;
}

static void UpdateSpeedTaskIfDue(uint8_t ctrl_mode)
{
    /* 电流环默认 2 kHz，速度估算/速度 PI 默认再分频到约 500 Hz。 */
    s_speed.div++;
    if (s_speed.div < MOTOR_SPEED_LOOP_DIV)
    {
        return;
    }
    s_speed.div = 0;

    UpdateSpeedEstimateFromPos();
    if (ctrl_mode == MOTOR_CTRL_SPEED)
    {
        (void)RunSpeedPi();
    }
    else
    {
        s_speed.iq_ref = 0;
    }
}

/*===========================================================================
 * 正式闭环计算
 *===========================================================================*/

static MotorLoopFault_t RunClosedLoopCalc(uint8_t ctrl_mode, uint8_t ma600_ok)
{
    uint16_t vdc;

    /*
     * 正式闭环路径：
     * 三相电流 + MA600 电角度 -> Clarke/Park -> PI -> InvPark/SVPWM -> EPWM。
     */
    ReadCurrent(&s_loop.cur);
    SnapshotAngleToLoop();

    if (ma600_ok == 0)
    {
        s_loop.active = 0;
        s_loop.output_ready = 0;
        ClearPi();
        ClearSpeedLoop();
        return MOTOR_LOOP_FAULT_MA600;
    }

    if (Motor_IsAngleSafe() == 0u)
    {
        s_loop.active = 0;
        s_loop.output_ready = 0;
        ClearPi();
        ClearSpeedLoop();
        return MOTOR_LOOP_FAULT_ANGLE;
    }

    if (IsRunCurrentSafe(&s_loop.cur) == 0)
    {
        s_loop.active = 0;
        s_loop.output_ready = 0;
        ClearPi();
        ClearSpeedLoop();
        return MOTOR_LOOP_FAULT_CURRENT;
    }

    s_loop.active = 1;
    s_loop.fast_count++;

    /* Current feedback transform. */
    Foc_ClarkeTransform(s_loop.cur.iu, s_loop.cur.iv, s_loop.cur.iw, &s_loop.i_alpha,
                        &s_loop.i_beta);
    Foc_ParkTransform(s_loop.i_alpha, s_loop.i_beta, s_loop.angle_elec, &s_loop.id, &s_loop.iq);

    if (ctrl_mode == MOTOR_CTRL_SPEED)
    {
        /* 速度模式下，速度 PI 的输出作为 q 轴电流给定。 */
        s_speed.ref = s_speed_ref_cmd;
        s_speed.iq_limit = s_iq_limit_cmd;
        s_loop.id_ref = 0;
        s_loop.iq_ref = s_speed.iq_ref;
    }
    else if (ctrl_mode == MOTOR_CTRL_CURRENT)
    {
        /* 电流模式直接使用调试/上层下发的 id/iq 给定。 */
        s_loop.id_ref = s_id_ref_cmd;
        s_loop.iq_ref = s_iq_ref_cmd;
    }
    else
    {
        s_loop.id_ref = 0;
        s_loop.iq_ref = 0;
    }

    s_loop.vd = Foc_PiUpdate(&s_pi_id, s_loop.id_ref, s_loop.id);
    s_loop.vq = Foc_PiUpdate(&s_pi_iq, s_loop.iq_ref, s_loop.iq);
    s_loop.v_limited = Foc_LimitDq(&s_loop.vd, &s_loop.vq, s_loop.v_limit);

    /* Voltage command transform and PWM update. */
    Foc_InvParkTransform(s_loop.vd, s_loop.vq, s_loop.angle_elec, &s_loop.v_alpha, &s_loop.v_beta);

    vdc = PWM_PERIOD;
    Foc_Svpwm(s_loop.v_alpha, s_loop.v_beta, vdc, &s_loop.duty_u, &s_loop.duty_v, &s_loop.duty_w);

    if (s_loop.output_ready != 0)
    {
        Board_SetPwmDuty(s_loop.duty_u, s_loop.duty_v, s_loop.duty_w);
        Board_EnablePwmOutput(1);
    }
    else
    {
        Board_ForcePwmOff();
    }

    return MOTOR_LOOP_FAULT_NONE;
}

/*===========================================================================
 * 转子对齐
 *===========================================================================*/

uint8_t Motor_LoopRunAlign(void)
{
    int16_t vd;
    int16_t vq;
    int16_t v_alpha;
    int16_t v_beta;
    uint16_t duty_u;
    uint16_t duty_v;
    uint16_t duty_w;

    /*
     * 阶段 0：PWM 运行时零漂重校准
     * 以 50% 占空比运行 PWM（驱动关闭），让 ADC 在 PWM 开关噪声环境下
     * 重新测量零漂，消除校准和运行时 ADC 配置差异引起的 DC offset。
     */
    if (s_align_phase == 0u)
    {
        Board_SetPwmDuty(PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50);
        Board_EnablePwmOutput(1);

        Board_CalibCurrentOffsetPwm(MOTOR_ALIGN_CAL_SAMPLES);

        Board_ForcePwmOff();
        s_align_phase = 1u;
        s_align_ticks = 0u;
        return 0u;
    }

    /*
     * 阶段 1：转子对齐
     * 输出固定 d 轴电压，将转子拉到 theta=0 方向。
     */
    ReadCurrent(&s_loop.cur);

    if (IsRunCurrentSafe(&s_loop.cur) == 0)
    {
        Board_ForcePwmOff();
        return 1u;
    }

    s_align_ticks++;
    if (s_align_ticks >= MOTOR_ALIGN_TICKS)
    {
        Board_ForcePwmOff();
        return 1u;
    }

    /*
     * 对齐只使用固定 d 轴电压，不使用速度环/电流环。
     * 目的是给闭环测试一个可重复的初始电角度关系。
     */
    vd = MOTOR_ALIGN_VD;
    vq = 0;
    (void)Foc_LimitDq(&vd, &vq, s_current_v_limit_cmd);
    Foc_InvParkTransform(vd, vq, MOTOR_ALIGN_THETA, &v_alpha, &v_beta);
    Foc_Svpwm(v_alpha, v_beta, PWM_PERIOD, &duty_u, &duty_v, &duty_w);

    Board_SetPwmDuty(duty_u, duty_v, duty_w);
    Board_EnablePwmOutput(1);

    return 0u;
}

/*===========================================================================
 * 公共接口函数
 *===========================================================================*/

void Motor_LoopInit(void)
{
    s_id_ref_cmd = 0;
    s_iq_ref_cmd = 0;
    s_speed_ref_cmd = 0;
    s_iq_limit_cmd = MOTOR_SPEED_IQ_LIMIT_DEFAULT;
    s_current_kp_cmd = MOTOR_CURRENT_KP;
    s_current_ki_cmd = MOTOR_CURRENT_KI;
    s_current_v_limit_cmd = MOTOR_CURRENT_V_LIMIT;
    s_align_ticks = 0;
    s_align_phase = 0;
    ClearLoop();
    Motor_OpenLoopInit();
}

MotorLoopFault_t Motor_LoopRun(uint8_t ctrl_mode, uint8_t ma600_ok)
{
    MotorLoopFault_t fault;
    MotorOpenLoopFault_t ol_fault;

    /*
     * VF/IF 是开环检测路径，保留在 Motor_OpenLoop 中。
     * 正式电流环/速度环只走 RunClosedLoopCalc。
     */
    if ((ctrl_mode == MOTOR_CTRL_VF) || (ctrl_mode == MOTOR_CTRL_IF))
    {
        ol_fault = Motor_OpenLoopRun(ctrl_mode);
        if (ol_fault == MOTOR_OPEN_LOOP_FAULT_ANGLE)
        {
            return MOTOR_LOOP_FAULT_ANGLE;
        }
        if (ol_fault == MOTOR_OPEN_LOOP_FAULT_CURRENT)
        {
            return MOTOR_LOOP_FAULT_CURRENT;
        }
        if (ol_fault == MOTOR_OPEN_LOOP_FAULT_TIMEOUT)
        {
            return MOTOR_LOOP_FAULT_OL_TIMEOUT;
        }
        return MOTOR_LOOP_FAULT_NONE;
    }

    if (Motor_IsAngleSafe() != 0u)
    {
        UpdateSpeedTaskIfDue(ctrl_mode);
    }

    if ((s_loop.output_ready == 0) || (ctrl_mode == MOTOR_CTRL_OFF))
    {
        return MOTOR_LOOP_FAULT_NONE;
    }

    fault = RunClosedLoopCalc(ctrl_mode, ma600_ok);
    return fault;
}

void Motor_LoopSetSpeedRef(int32_t speed_ref)
{
    s_speed_ref_cmd = Foc_ClampS32(speed_ref, -MOTOR_SPEED_REF_LIMIT, MOTOR_SPEED_REF_LIMIT);
    s_speed.ref = s_speed_ref_cmd;
}

void Motor_LoopSetIqLimit(int16_t iq_limit)
{
    if (iq_limit < 0)
    {
        iq_limit = (int16_t)-iq_limit;
    }
    s_iq_limit_cmd = ClampRefS16(iq_limit, MOTOR_CURRENT_REF_LIMIT);
    s_speed.iq_limit = s_iq_limit_cmd;
}

void Motor_LoopSetCurrentRef(int16_t id_ref, int16_t iq_ref)
{
    s_id_ref_cmd = ClampRefS16(id_ref, MOTOR_CURRENT_REF_LIMIT);
    s_iq_ref_cmd = ClampRefS16(iq_ref, MOTOR_CURRENT_REF_LIMIT);
    s_loop.id_ref = s_id_ref_cmd;
    s_loop.iq_ref = s_iq_ref_cmd;
}

void Motor_LoopSetCurrentPi(int16_t kp, int16_t ki, int16_t v_limit)
{
    kp = ClampPiGainS16(kp);
    ki = ClampPiGainS16(ki);
    v_limit = ClampVoltageLimitS16(v_limit);

    if ((kp == s_current_kp_cmd) && (ki == s_current_ki_cmd) &&
        (v_limit == s_current_v_limit_cmd))
    {
        return;
    }

    s_current_kp_cmd = kp;
    s_current_ki_cmd = ki;
    s_current_v_limit_cmd = v_limit;
    ClearPi();
}

void Motor_LoopSetOutputReady(uint8_t ready)
{
    s_loop.output_ready = ready;
}

uint8_t Motor_LoopGetOutputReady(void)
{
    return s_loop.output_ready;
}

uint8_t Motor_LoopGetActive(void)
{
    return s_loop.active;
}

void Motor_LoopFillRunSnap(MotorRunSnap_t* snap)
{
    if (snap == 0)
    {
        return;
    }

    snap->speed_ref = s_speed_ref_cmd;
    snap->speed_fb = s_speed.fb_filt;
    snap->id_ref = s_loop.id_ref;
    snap->iq_ref = s_loop.iq_ref;
    snap->id = s_loop.id;
    snap->iq = s_loop.iq;
    snap->vd = s_loop.vd;
    snap->vq = s_loop.vq;
    snap->kp = s_loop.kp;
    snap->ki = s_loop.ki;
    snap->v_limit = s_loop.v_limit;
    snap->v_limited = s_loop.v_limited;
    snap->current_over_count = s_current_over_count;
}
