/**
 * @file Motor.c
 * @brief 电机控制模块实现。
 * @details 实现电机状态机、角度处理和对外接口。
 *          闭环控制已迁入 Motor_Loop，开环检测已迁入 Motor_OpenLoop。
 */

#include "Motor.h"
#include "Motor_Foc.h"
#include "Motor_Loop.h"
#include "Motor_OpenLoop.h"
#include "Motor_ZeroScanTest.h"

#include "Board.h"
#include "Board_Analog.h"
#include "Board_PWM.h"

typedef struct
{
    int16_t iu;
    int16_t iv;
    int16_t iw;
    int16_t sum;
} MotorCurrent_t;

/**
 * @brief Motor 层角度缓存。
 * @details raw/elec 为最近一次同步角度，delta/pos 用于速度估算和方向判断。
 */
typedef struct
{
    uint16_t raw;
    uint16_t elec;
    uint16_t prev_raw;
    int16_t delta;
    int32_t pos;
    uint8_t ok;
    uint8_t age;
    uint8_t inited;
} MotorAngle_t;

static volatile MotorState_t s_state = MOTOR_STATE_IDLE;
static volatile uint8_t s_enable;

static MotorCheck_t s_check;
static MotorCurrent_t s_check_cur;
static volatile MotorAngle_t s_angle;

static uint8_t s_ma600_samples;
static volatile uint16_t s_elec_zero = MOTOR_ELEC_ZERO;
static volatile MotorControlMode_t s_ctrl_mode;

static MotorFaultReason_t s_fault_reason;

static uint8_t IsInRangeS16(int16_t value, int16_t limit);
static void ClearCheck(void);
static void ClearAngle(void);
static void ClearFault(void);
static void ReadCurrent(MotorCurrent_t* cur);
static uint8_t IsCurrentSafe(const MotorCurrent_t* cur);
static void UpdateAngle(void);
static uint8_t IsAngleSafe(void);
static void UpdateSensorCheck(void);
static void EnterFault(MotorFaultReason_t reason);

void Motor_Init(void)
{
    /* Motor 只清内部状态；PWM/ADC/MA600 等硬件初始化由 Board_Init() 完成。 */
    s_state = MOTOR_STATE_IDLE;
    s_enable = 0;
    s_ctrl_mode = MOTOR_CTRL_OFF;
    ClearFault();
    ClearCheck();
    ClearAngle();
    Motor_LoopInit();
    Motor_ZeroScanTestInit();
}

void Motor_TASK(void)
{
    /*
     * 低频状态机负责“是否允许输出”，快环只负责“如何计算输出”。
     * 检查失败时统一收敛到 ForcePwmOff 或 FAULT。
     */
    if (s_enable == 0)
    {
        s_state = MOTOR_STATE_IDLE;
        Board_ForcePwmOff();
        ClearCheck();
        ClearAngle();
        Motor_LoopInit();
        return;
    }

    switch (s_state)
    {
        
        case MOTOR_STATE_IDLE:
            Board_ForcePwmOff();
            ClearCheck();
            ClearAngle();
            s_state = MOTOR_STATE_SENSOR_CHECK;
            break;

        case MOTOR_STATE_SENSOR_CHECK:
            UpdateSensorCheck();
            if (s_check.ready_closed_loop != 0)
            {
                /* 进入闭环前先重置控制器，再走一次对齐流程。 */
                Motor_LoopInit();
                s_state = MOTOR_STATE_ALIGN;
            }
            break;

        case MOTOR_STATE_ALIGN:
            UpdateSensorCheck();
            if (s_check.ma600_ok == 0)
            {
                EnterFault(MOTOR_FAULT_MA600_CHECK);
                break;
            }
            if (s_check.current_ok == 0)
            {
                EnterFault(MOTOR_FAULT_CURRENT_CHECK);
                break;
            }
            {
                uint8_t done = Motor_LoopRunAlign();
                if (done != 0u)
                {
                    s_state = MOTOR_STATE_CLOSED_LOOP;
                }
            }
            break;

        case MOTOR_STATE_CLOSED_LOOP:
            UpdateSensorCheck();
            if (s_check.ma600_ok == 0)
            {
                EnterFault(MOTOR_FAULT_MA600_CHECK);
                break;
            }
            if (s_check.current_ok == 0)
            {
                EnterFault(MOTOR_FAULT_CURRENT_CHECK);
                break;
            }

            if ((s_ctrl_mode != MOTOR_CTRL_OFF) && (s_check.ma600_ok != 0) &&
                (s_check.current_ok != 0))
            {
                Motor_LoopSetOutputReady(1);
            }
            else
            {
                Motor_LoopSetOutputReady(0);
            }
            if (Motor_LoopGetOutputReady() == 0)
            {
                Board_ForcePwmOff();
            }

            break;

        case MOTOR_STATE_FAULT:
            Board_ForcePwmOff();
            Motor_LoopSetOutputReady(0);
            break;

        default:
            EnterFault(MOTOR_FAULT_STATE);
            break;
    }
}

void Motor_FastLoop(void)
{
    static uint8_t div = 0;
    MotorLoopFault_t fault;

    /* 快环由 ADC 中断调用；这里避免阻塞操作、调试输出和复杂状态切换。 */
    if (s_enable == 0)
    {
        Board_ForcePwmOff();
        return;
    }

    if (s_state != MOTOR_STATE_CLOSED_LOOP)
    {
        return;
    }

    div++;
    if (div < MOTOR_FAST_LOOP_DIV)
    {
        return;
    }
    div = 0;

    fault = Motor_LoopRun((uint8_t)s_ctrl_mode, s_check.ma600_ok);
    if (fault == MOTOR_LOOP_FAULT_MA600)
    {
        EnterFault(MOTOR_FAULT_MA600_CHECK);
    }
    else if (fault == MOTOR_LOOP_FAULT_ANGLE)
    {
        EnterFault(MOTOR_FAULT_ANGLE_STALE);
    }
    else if (fault == MOTOR_LOOP_FAULT_CURRENT)
    {
        EnterFault(MOTOR_FAULT_RUN_CURRENT);
    }
    else if (fault == MOTOR_LOOP_FAULT_OL_TIMEOUT)
    {
        /* VF/IF 超时：关断 PWM 并切换到 OFF 模式 */
        Board_ForcePwmOff();
        s_ctrl_mode = MOTOR_CTRL_OFF;
        Motor_LoopSetOutputReady(0);
    }
}

void Motor_UpdateAngleFromIsr(void)
{
    uint16_t raw = Board_GetAngleRaw();
    uint8_t ok = Board_IsAngleOk();

    /* Board 层已经完成 SPI 读角；这里仅消费缓存并派生电角度/累计位置。 */
    s_angle.ok = (uint8_t)(ok && Board_IsAngleOk());

    if (s_angle.ok == 0u)
    {
        if (s_angle.age < 255u)
        {
            s_angle.age++;
        }
        return;
    }

    s_angle.raw = raw;
    s_angle.elec = Motor_GetElecAngle(raw);
    s_angle.age = 0u;

    if (s_angle.inited == 0u)
    {
        s_angle.prev_raw = raw;
        s_angle.delta = 0;
        s_angle.inited = 1u;
    }
    else
    {
        s_angle.delta = (int16_t)(raw - s_angle.prev_raw);
        s_angle.prev_raw = raw;
        s_angle.pos += s_angle.delta;
    }
}

void Motor_SetEnable(uint8_t enable)
{
    if (enable == 0)
    {
        /* 总使能关闭时立即切 OFF 并强制关断功率输出。 */
        s_enable = 0;
        s_ctrl_mode = MOTOR_CTRL_OFF;
        ClearFault();
        s_state = MOTOR_STATE_IDLE;

        Board_ForcePwmOff();

        ClearCheck();
        ClearAngle();
        Motor_LoopInit();
        return;
    }

    s_enable = 1;
    if (s_state == MOTOR_STATE_IDLE)
    {
        ClearFault();
    }
}

void Motor_SetControlMode(MotorControlMode_t mode)
{
    if (mode > MOTOR_CTRL_IF)
    {
        mode = MOTOR_CTRL_OFF;
    }

    if (s_ctrl_mode == mode)
    {
        return;
    }

    /* 模式切换时先关闭输出门控，避免新模式继承上一模式的输出状态。 */
    s_ctrl_mode = mode;
    Motor_LoopSetOutputReady(0);

    if ((mode == MOTOR_CTRL_VF) || (mode == MOTOR_CTRL_IF))
    {
        Motor_OpenLoopInit();
    }

    if (mode == MOTOR_CTRL_OFF)
    {
        Board_ForcePwmOff();
    }
}

void Motor_SetCurrentRef(int16_t id_ref, int16_t iq_ref)
{
    Motor_LoopSetCurrentRef(id_ref, iq_ref);
}

void Motor_SetCurrentPi(int16_t kp, int16_t ki, int16_t v_limit)
{
    Motor_LoopSetCurrentPi(kp, ki, v_limit);
}

void Motor_SetSpeedRef(int32_t speed_ref)
{
    Motor_LoopSetSpeedRef(speed_ref);
}

void Motor_SetIqLimit(int16_t iq_limit)
{
    Motor_LoopSetIqLimit(iq_limit);
}

void Motor_SetOlSpeedRef(int32_t speed_ref)
{
    Motor_OpenLoopSetSpeedRef(speed_ref);
}

void Motor_SetVfVoltage(int16_t voltage)
{
    Motor_OpenLoopSetVfVoltage(voltage);
}

void Motor_SetIfCurrentRef(int16_t id_ref, int16_t iq_ref)
{
    Motor_OpenLoopSetIfCurrentRef(id_ref, iq_ref);
}

void Motor_SetOlTimeoutMs(uint16_t timeout_ms)
{
    Motor_OpenLoopSetTimeoutMs(timeout_ms);
}

void Motor_SetElecZero(uint16_t zero)
{
    s_elec_zero = zero;
}

uint16_t Motor_GetAngleRaw(void)
{
    return s_angle.raw;
}

uint16_t Motor_GetAngleElec(void)
{
    return s_angle.elec;
}

int16_t Motor_GetAngleDelta(void)
{
    return s_angle.delta;
}

int32_t Motor_GetAnglePos(void)
{
    return s_angle.pos;
}

uint8_t Motor_IsAngleSafe(void)
{
    return IsAngleSafe();
}

uint8_t Motor_GetAngleAge(void)
{
    return s_angle.age;
}

void Motor_GetCheck(MotorCheck_t* check)
{
    if (check == 0)
    {
        return;
    }

    *check = s_check;
}

void Motor_GetRunSnap(MotorRunSnap_t* snap)
{
    volatile uint16_t duty_u;
    volatile uint16_t duty_v;
    volatile uint16_t duty_w;
    volatile uint8_t pwm_on;
    volatile uint8_t brake_on;

    if (snap == 0)
    {
        return;
    }

    /* 调试快照只读缓存，不触发任何硬件事务。 */
    snap->state = s_state;
    snap->ctrl_mode = s_ctrl_mode;
    snap->enable = s_enable;
    snap->angle_raw = s_angle.raw;
    snap->angle_elec = s_angle.elec;
    snap->angle_pos = s_angle.pos;
    snap->angle_delta = s_angle.delta;
    snap->fault_reason = s_fault_reason;

    Motor_LoopFillRunSnap(snap);

    Board_GetPwmDebug(&duty_u, &duty_v, &duty_w, &pwm_on, &brake_on);
    snap->duty_u = duty_u;  
    snap->duty_v = duty_v;
    snap->duty_w = duty_w;
    snap->pwm_output_on = pwm_on;
    snap->pwm_brake_on = brake_on;
}

uint16_t Motor_GetElecZero(void)
{
    return s_elec_zero;
}

uint16_t Motor_GetElecAngle(uint16_t sensor_angle)
{
#if MOTOR_SENSOR_DIR > 0
    /* 当前 MA600 四对极磁环与电机四对极匹配，传感器角按电角度使用。 */
    return (uint16_t)(sensor_angle + s_elec_zero);
#else
    /* SENSOR_DIR 为负时，电角度方向反转，同时叠加已标定的零点。 */
    return (uint16_t)(s_elec_zero - sensor_angle);
#endif
}

static uint8_t IsInRangeS16(int16_t value, int16_t limit)
{
    return (uint8_t)((value >= -limit) && (value <= limit));
}

static void ClearCheck(void)
{
    s_check.ma600_ok = 0;
    s_check.current_ok = 0;
    s_check.pwm_off_safe = 0;
    s_check.ready_closed_loop = 0;
    s_ma600_samples = 0;
}

static void ClearAngle(void)
{
    s_angle.raw = 0;
    s_angle.elec = 0;
    s_angle.prev_raw = 0;
    s_angle.delta = 0;
    s_angle.pos = 0;
    s_angle.ok = 0;
    s_angle.age = 255u;
    s_angle.inited = 0;
}

static void ClearFault(void)
{
    s_fault_reason = MOTOR_FAULT_NONE;
}

static void ReadCurrent(MotorCurrent_t* cur)
{
    cur->iu = Board_GetIuCnt();
    cur->iv = Board_GetIvCnt();
    cur->iw = Board_GetIwCnt();
    cur->sum = Board_GetIuvwSum();
}

static uint8_t IsCurrentSafe(const MotorCurrent_t* cur)
{
    return (uint8_t)(IsInRangeS16(cur->iu, MOTOR_CHECK_CURRENT_CNT_LIMIT) &&
                     IsInRangeS16(cur->iv, MOTOR_CHECK_CURRENT_CNT_LIMIT) &&
                     IsInRangeS16(cur->iw, MOTOR_CHECK_CURRENT_CNT_LIMIT) &&
                     IsInRangeS16(cur->sum, MOTOR_CHECK_SUM_CNT_LIMIT));
}

static void UpdateAngle(void)
{
    uint8_t ok = Board_UpdateAngle();
    uint16_t raw = Board_GetAngleRaw();

    s_angle.ok = (uint8_t)(ok && Board_IsAngleOk());

    if (s_angle.ok == 0u)
    {
        if (s_angle.age < 255u)
        {
            s_angle.age++;
        }
        return;
    }

    s_angle.raw = raw;
    s_angle.elec = Motor_GetElecAngle(raw);
    s_angle.age = 0u;

    if (s_angle.inited == 0u)
    {
        s_angle.prev_raw = raw;
        s_angle.delta = 0;
        s_angle.inited = 1u;
    }
    else
    {
        s_angle.delta = (int16_t)(raw - s_angle.prev_raw);
        s_angle.prev_raw = raw;
        s_angle.pos += s_angle.delta;
    }
}

static uint8_t IsAngleSafe(void)
{
    return (uint8_t)((s_angle.ok != 0u) && (s_angle.age <= MOTOR_ANGLE_MAX_AGE));
}

static void UpdateSensorCheck(void)
{
    /*
     * 进入闭环前同时检查 MA600、静态电流和 PWM 关闭安全态。
     * 这里不打开输出，只给状态机提供 ready_closed_loop 条件。
     */
    if ((s_angle.inited == 0u) || (s_angle.age > MOTOR_ANGLE_MAX_AGE))
    {
        UpdateAngle();
    }

    ReadCurrent(&s_check_cur);

    if (IsAngleSafe() != 0u)
    {
        if (s_ma600_samples < MOTOR_CHECK_MA600_SAMPLES)
        {
            s_ma600_samples++;
        }
    }
    else
    {
        s_ma600_samples = 0;
    }

    s_check.ma600_ok = (uint8_t)(s_ma600_samples >= MOTOR_CHECK_MA600_SAMPLES);
    s_check.current_ok = IsCurrentSafe(&s_check_cur);
    s_check.pwm_off_safe = Board_IsPwmOffSafe();
    s_check.ready_closed_loop =
        (uint8_t)(s_check.ma600_ok && s_check.current_ok && s_check.pwm_off_safe);
}

static void EnterFault(MotorFaultReason_t reason)
{
    if (s_fault_reason == MOTOR_FAULT_NONE)
    {
        s_fault_reason = reason;
    }

    s_state = MOTOR_STATE_FAULT;
    Motor_LoopSetOutputReady(0);
    Board_ForcePwmOff();
}
