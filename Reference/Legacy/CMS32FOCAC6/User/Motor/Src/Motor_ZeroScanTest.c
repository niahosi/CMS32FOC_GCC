/**
 * @file Motor_ZeroScanTest.c
 * @brief 临时电角度零点扫描测试实现。
 * @details 扫描不同 elec_zero_trim，并分别施加正/负 q 轴小电流。
 *          分数越小，说明当前零点下 d 轴串扰小、q 轴电流跟随更接近给定。
 */

#include "Motor_ZeroScanTest.h"

typedef struct
{
    MotorZeroScanTestState_t state;
    MotorZeroScanTestFault_t fault;

    int32_t start_trim;
    int32_t end_trim;
    int32_t step;
    int32_t trim;
    int32_t best_trim;
    int32_t score;
    int32_t best_score;
    int16_t iq_ref;
    MotorZeroScanTestPhase_t phase;

    uint16_t settle_ticks;
    uint16_t sample_ticks;
    uint16_t ticks;
    uint16_t samples;
    uint8_t v_limit_count;

    int32_t id_sum;
    int32_t iq_sum;
    int32_t id_abs_sum;
    int32_t iq_err_abs_sum;
    int32_t vq_abs_sum;
    int16_t id_avg;
    int16_t iq_avg;
    int16_t iq_err_avg;
    int16_t vq_avg;
    int16_t best_id_avg;
    int16_t best_iq_err_avg;
    int16_t best_vq_avg;
} ZeroScanTest_t;

#define ZERO_SCAN_TEST_DEFAULT_IQ_REF 40
#define ZERO_SCAN_TEST_DEFAULT_SETTLE_TICKS 100u
#define ZERO_SCAN_TEST_DEFAULT_SAMPLE_TICKS 200u
#define ZERO_SCAN_TEST_V_LIMIT_MAX 8u
#define ZERO_SCAN_TEST_SCORE_MAX 0x7FFFFFFFl

static ZeroScanTest_t s_test;

static int32_t AbsS16ToS32(int16_t value)
{
    int32_t v = value;
    return (v < 0) ? -v : v;
}

static int16_t ClampIqRef(int16_t iq_ref)
{
    int32_t ref = iq_ref;

    if (ref < 0)
    {
        ref = -ref;
    }
    if (ref == 0)
    {
        ref = ZERO_SCAN_TEST_DEFAULT_IQ_REF;
    }
    if (ref > MOTOR_CURRENT_REF_LIMIT)
    {
        ref = MOTOR_CURRENT_REF_LIMIT;
    }
    return (int16_t)ref;
}

static int16_t GetPhaseIqRef(void)
{
    if (s_test.phase == MOTOR_ZERO_SCAN_TEST_PHASE_NEG)
    {
        return (int16_t)(-s_test.iq_ref);
    }
    return s_test.iq_ref;
}

static void ClearSampleStats(void)
{
    s_test.ticks = 0u;
    s_test.samples = 0u;
    s_test.v_limit_count = 0u;
    s_test.id_sum = 0;
    s_test.iq_sum = 0;
    s_test.id_abs_sum = 0;
    s_test.iq_err_abs_sum = 0;
    s_test.vq_abs_sum = 0;
    s_test.id_avg = 0;
    s_test.iq_avg = 0;
    s_test.iq_err_avg = 0;
    s_test.vq_avg = 0;
}

static void ClearPointStats(void)
{
    s_test.score = 0;
    s_test.phase = MOTOR_ZERO_SCAN_TEST_PHASE_POS;
    ClearSampleStats();
}

static uint8_t IsBadParam(const volatile MotorZeroScanTestCmd_t* cmd)
{
    if (cmd == 0)
    {
        return 1u;
    }
    if (cmd->step == 0)
    {
        return 1u;
    }
    if (cmd->iq_ref == 0)
    {
        return 1u;
    }
    if ((cmd->end_trim > cmd->start_trim) && (cmd->step < 0))
    {
        return 1u;
    }
    if ((cmd->end_trim < cmd->start_trim) && (cmd->step > 0))
    {
        return 1u;
    }
    return 0u;
}

static uint8_t HasNextTrim(void)
{
    int32_t next = s_test.trim + s_test.step;

    if (s_test.step > 0)
    {
        return (uint8_t)(next <= s_test.end_trim);
    }
    return (uint8_t)(next >= s_test.end_trim);
}

static void EnterFault(MotorZeroScanTestFault_t fault)
{
    s_test.state = MOTOR_ZERO_SCAN_TEST_FAULT;
    s_test.fault = fault;
}

static void StartScan(const volatile MotorZeroScanTestCmd_t* cmd)
{
    s_test.start_trim = cmd->start_trim;
    s_test.end_trim = cmd->end_trim;
    s_test.step = cmd->step;
    s_test.trim = cmd->start_trim;
    s_test.best_trim = cmd->start_trim;
    s_test.best_score = ZERO_SCAN_TEST_SCORE_MAX;
    s_test.iq_ref = ClampIqRef(cmd->iq_ref);
    s_test.settle_ticks = cmd->settle_ticks;
    s_test.sample_ticks = cmd->sample_ticks;

    if (s_test.settle_ticks == 0u)
    {
        s_test.settle_ticks = ZERO_SCAN_TEST_DEFAULT_SETTLE_TICKS;
    }
    if (s_test.sample_ticks == 0u)
    {
        s_test.sample_ticks = ZERO_SCAN_TEST_DEFAULT_SAMPLE_TICKS;
    }

    s_test.fault = MOTOR_ZERO_SCAN_TEST_FAULT_NONE;
    ClearPointStats();
    s_test.state = MOTOR_ZERO_SCAN_TEST_WAIT_CLOSED_LOOP;
}

static uint8_t CheckRunCondition(void)
{
    MotorRunSnap_t run;

    Motor_GetRunSnap(&run);
    if (run.fault_reason != MOTOR_FAULT_NONE)
    {
        EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_MOTOR_FAULT);
        return 0u;
    }
    if (run.state != MOTOR_STATE_CLOSED_LOOP)
    {
        EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_NOT_CLOSED_LOOP);
        return 0u;
    }
    if (run.ctrl_mode != MOTOR_CTRL_CURRENT)
    {
        EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_NOT_CURRENT_MODE);
        return 0u;
    }
    return 1u;
}

static uint8_t IsReadyToSample(void)
{
    MotorRunSnap_t run;

    Motor_GetRunSnap(&run);
    if (run.fault_reason != MOTOR_FAULT_NONE)
    {
        EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_MOTOR_FAULT);
        return 0u;
    }
    if ((run.state == MOTOR_STATE_CLOSED_LOOP) && (run.ctrl_mode == MOTOR_CTRL_CURRENT))
    {
        return 1u;
    }
    return 0u;
}

static void SamplePoint(void)
{
    MotorRunSnap_t run;
    int32_t iq_err;

    Motor_GetRunSnap(&run);
    iq_err = (int32_t)GetPhaseIqRef() - (int32_t)run.iq;

    s_test.id_sum += run.id;
    s_test.iq_sum += run.iq;
    s_test.id_abs_sum += AbsS16ToS32(run.id);
    s_test.iq_err_abs_sum += AbsS16ToS32((int16_t)iq_err);
    s_test.vq_abs_sum += AbsS16ToS32(run.vq);
    s_test.samples++;

    if (run.v_limited != 0u)
    {
        if (s_test.v_limit_count < 255u)
        {
            s_test.v_limit_count++;
        }
    }
}

static void FinishTrim(void)
{
    if (s_test.score < s_test.best_score)
    {
        s_test.best_score = s_test.score;
        s_test.best_trim = s_test.trim;
        s_test.best_id_avg = s_test.id_avg;
        s_test.best_iq_err_avg = s_test.iq_err_avg;
        s_test.best_vq_avg = s_test.vq_avg;
    }

    if (HasNextTrim() == 0u)
    {
        s_test.state = MOTOR_ZERO_SCAN_TEST_DONE;
        return;
    }

    s_test.trim += s_test.step;
    ClearPointStats();
    s_test.state = MOTOR_ZERO_SCAN_TEST_SETTLE;
}

static void FinishPhase(void)
{
    int32_t id_abs_avg;
    int32_t iq_err_abs_avg;
    int32_t vq_abs_avg;
    int32_t id_avg;
    int32_t iq_avg;

    if (s_test.samples == 0u)
    {
        EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_BAD_PARAM);
        return;
    }

    id_abs_avg = s_test.id_abs_sum / (int32_t)s_test.samples;
    iq_err_abs_avg = s_test.iq_err_abs_sum / (int32_t)s_test.samples;
    vq_abs_avg = s_test.vq_abs_sum / (int32_t)s_test.samples;
    id_avg = s_test.id_sum / (int32_t)s_test.samples;
    iq_avg = s_test.iq_sum / (int32_t)s_test.samples;

    s_test.id_avg = (int16_t)id_avg;
    s_test.iq_avg = (int16_t)iq_avg;
    s_test.iq_err_avg = (int16_t)iq_err_abs_avg;
    s_test.vq_avg = (int16_t)vq_abs_avg;

    /*
     * 严格找零重点看两件事：
     * 1. d 轴电流是否接近 0，避免电流投影到励磁轴；
     * 2. q 轴电流是否跟随给定，避免只靠电压硬推。
     * vq 权重较低，只用于排除明显耗压的点。
     */
    s_test.score += id_abs_avg * 4 + iq_err_abs_avg * 4 + vq_abs_avg;

    if (s_test.v_limit_count > ZERO_SCAN_TEST_V_LIMIT_MAX)
    {
        EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_V_LIMIT);
        return;
    }

    if (s_test.phase == MOTOR_ZERO_SCAN_TEST_PHASE_POS)
    {
        s_test.phase = MOTOR_ZERO_SCAN_TEST_PHASE_NEG;
        ClearSampleStats();
        s_test.state = MOTOR_ZERO_SCAN_TEST_SETTLE;
        return;
    }

    FinishTrim();
}

void Motor_ZeroScanTestInit(void)
{
    s_test.state = MOTOR_ZERO_SCAN_TEST_IDLE;
    s_test.fault = MOTOR_ZERO_SCAN_TEST_FAULT_NONE;
    s_test.start_trim = 0;
    s_test.end_trim = 0;
    s_test.step = 0;
    s_test.trim = 0;
    s_test.best_trim = 0;
    s_test.score = 0;
    s_test.best_score = ZERO_SCAN_TEST_SCORE_MAX;
    s_test.iq_ref = ZERO_SCAN_TEST_DEFAULT_IQ_REF;
    s_test.phase = MOTOR_ZERO_SCAN_TEST_PHASE_POS;
    s_test.best_id_avg = 0;
    s_test.best_iq_err_avg = 0;
    s_test.best_vq_avg = 0;
    s_test.settle_ticks = ZERO_SCAN_TEST_DEFAULT_SETTLE_TICKS;
    s_test.sample_ticks = ZERO_SCAN_TEST_DEFAULT_SAMPLE_TICKS;
    ClearSampleStats();
}

void Motor_ZeroScanTestTask(const volatile MotorZeroScanTestCmd_t* cmd)
{
    if ((cmd == 0) || (cmd->enable == 0u))
    {
        s_test.state = MOTOR_ZERO_SCAN_TEST_IDLE;
        s_test.fault = MOTOR_ZERO_SCAN_TEST_FAULT_NONE;
        return;
    }

    if (cmd->stop_req != 0u)
    {
        EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_STOP_REQ);
        return;
    }

    if ((s_test.state == MOTOR_ZERO_SCAN_TEST_IDLE) && (cmd->start_req != 0u))
    {
        if (IsBadParam(cmd) != 0u)
        {
            EnterFault(MOTOR_ZERO_SCAN_TEST_FAULT_BAD_PARAM);
            return;
        }
        StartScan(cmd);
    }

    switch (s_test.state)
    {
        case MOTOR_ZERO_SCAN_TEST_WAIT_CLOSED_LOOP:
            if (IsReadyToSample() != 0u)
            {
                ClearSampleStats();
                s_test.state = MOTOR_ZERO_SCAN_TEST_SETTLE;
            }
            break;

        case MOTOR_ZERO_SCAN_TEST_SETTLE:
            if (CheckRunCondition() == 0u)
            {
                break;
            }
            s_test.ticks++;
            if (s_test.ticks >= s_test.settle_ticks)
            {
                ClearSampleStats();
                s_test.state = MOTOR_ZERO_SCAN_TEST_SAMPLE;
            }
            break;

        case MOTOR_ZERO_SCAN_TEST_SAMPLE:
            if (CheckRunCondition() == 0u)
            {
                break;
            }
            SamplePoint();
            if (s_test.samples >= s_test.sample_ticks)
            {
                FinishPhase();
            }
            break;

        case MOTOR_ZERO_SCAN_TEST_DONE:
        case MOTOR_ZERO_SCAN_TEST_FAULT:
        case MOTOR_ZERO_SCAN_TEST_IDLE:
        default:
            break;
    }
}

uint8_t Motor_ZeroScanTestIsActive(void)
{
    return (uint8_t)((s_test.state == MOTOR_ZERO_SCAN_TEST_WAIT_CLOSED_LOOP) ||
                     (s_test.state == MOTOR_ZERO_SCAN_TEST_SETTLE) ||
                     (s_test.state == MOTOR_ZERO_SCAN_TEST_SAMPLE));
}

int32_t Motor_ZeroScanTestGetTrim(void)
{
    if (s_test.state == MOTOR_ZERO_SCAN_TEST_DONE)
    {
        return s_test.best_trim;
    }
    return s_test.trim;
}

int16_t Motor_ZeroScanTestGetIqRef(void)
{
    if ((s_test.state == MOTOR_ZERO_SCAN_TEST_WAIT_CLOSED_LOOP) ||
        (s_test.state == MOTOR_ZERO_SCAN_TEST_SETTLE) ||
        (s_test.state == MOTOR_ZERO_SCAN_TEST_SAMPLE))
    {
        return GetPhaseIqRef();
    }
    return 0;
}

void Motor_ZeroScanTestFillSnap(MotorZeroScanTestSnap_t* snap)
{
    if (snap == 0)
    {
        return;
    }

    snap->state = s_test.state;
    snap->phase = s_test.phase;
    snap->iq_ref = GetPhaseIqRef();
    snap->trim = s_test.trim;
    snap->best_trim = s_test.best_trim;
    snap->score = s_test.score;
    snap->best_score = s_test.best_score;
    snap->id_avg = s_test.id_avg;
    snap->iq_avg = s_test.iq_avg;
    snap->iq_err_avg = s_test.iq_err_avg;
    snap->vq_avg = s_test.vq_avg;
    snap->best_id_avg = s_test.best_id_avg;
    snap->best_iq_err_avg = s_test.best_iq_err_avg;
    snap->best_vq_avg = s_test.best_vq_avg;
    snap->fault = s_test.fault;
}
