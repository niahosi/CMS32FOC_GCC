#include "App_Debug.h"
#include "Board.h"
#include "Board_Analog.h"
#include "Board_PWM.h"
#include "Motor_OpenLoop.h"
#include "Motor_ZeroScanTest.h"
#include "SEGGER_RTT.h"

/**
 * @file App_Debug.c
 * @brief 应用层调试快照实现。
 * @details 把 Board/Motor 分散的运行量汇总到 g_elementary，便于 Watch/JScope 观察。
 */

volatile AppCmd_t g_cmd = {
    {0u, MOTOR_CTRL_OFF},
    {0, 0, MOTOR_CURRENT_KP, MOTOR_CURRENT_KI, MOTOR_CURRENT_V_LIMIT},
    {0, MOTOR_SPEED_IQ_LIMIT_DEFAULT},
    {0},
    {MOTOR_OL_SPEED_REF_DEFAULT, MOTOR_VF_VOLTAGE_DEFAULT, MOTOR_IF_ID_REF_DEFAULT,
     MOTOR_IF_IQ_REF_DEFAULT, MOTOR_OL_TIMEOUT_MS_DEFAULT},
    {0u, 0u, 0u, 40, -8192, 8192, 512, 100u, 200u},
    {0u}};

volatile AppElementary_t g_elementary;

static uint16_t s_rtt_div;

void Debug_InitRtt(void)
{
    s_rtt_div = 0u;
}

void Debug_UpdateElementary(void)
{
    MotorCheck_t check;
    MotorRunSnap_t run;
    MotorOpenLoopSnap_t open_loop;
    MotorZeroScanTestSnap_t zero_scan;

    /*
     * 这里只读取各模块缓存，不触发 SPI/ADC/PWM 配置动作。
     * 因此可以放在主循环中频繁刷新，不影响 ADC 快环。
     */
    Motor_GetRunSnap(&run);

    g_elementary.motor.state = run.state;
    g_elementary.motor.ctrl_mode = run.ctrl_mode;
    g_elementary.motor.enable = run.enable;
    g_elementary.motor.fault_reason = run.fault_reason;

    g_elementary.angle.raw = run.angle_raw;
    g_elementary.angle.elec = run.angle_elec;
    g_elementary.angle.pos = run.angle_pos;
    g_elementary.angle.delta = run.angle_delta;

    g_elementary.current.iu = Board_GetIuCnt();
    g_elementary.current.iv = Board_GetIvCnt();
    g_elementary.current.iw = Board_GetIwCnt();
    g_elementary.current.sum = Board_GetIuvwSum();
    g_elementary.current.iu_raw_adc = Board_GetIuRawAdc();
    g_elementary.current.iv_raw_adc = Board_GetIvRawAdc();
    g_elementary.current.iw_raw_adc = Board_GetIwRawAdc();
    g_elementary.current.iu_raw_cnt = Board_GetIuRawCnt();
    g_elementary.current.iv_raw_cnt = Board_GetIvRawCnt();
    g_elementary.current.iw_raw_cnt = Board_GetIwRawCnt();
    g_elementary.current.raw_sum = Board_GetIuvwRawSum();
    g_elementary.current.adc_sync_count = Board_GetAdcSyncCount();
    g_elementary.current.adc_trigger_tick = Board_GetAdcTriggerTick();
    g_elementary.current.sample_dynamic_mode = Board_IsCurrentSampleDynamicEnabled();
    g_elementary.current.sample_pair =
        (AppCurrentSamplePair_t)Board_GetCurrentSamplePair();
    g_elementary.current.sample_center_tick = Board_GetCurrentSampleCenterTick();
    g_elementary.current.sample_diag_tick_a = Board_GetAdcTriggerTickA();
    g_elementary.current.sample_diag_tick_b = Board_GetAdcTriggerTickB();
    g_elementary.current.sample_window_u = Board_GetCurrentSampleWindowU();
    g_elementary.current.sample_window_v = Board_GetCurrentSampleWindowV();
    g_elementary.current.sample_window_w = Board_GetCurrentSampleWindowW();
    g_elementary.current.sample_hold = Board_GetCurrentSampleHold();
    g_elementary.current.sample_hold_count = Board_GetCurrentSampleHoldCount();
    g_elementary.current.sample_diag_stage = Board_GetCurrentSampleDiagStage();
    g_elementary.current.sample_diag_count = Board_GetCurrentSampleDiagCount();
    g_elementary.current.sample_a_first = Board_GetCurrentSampleAFirst();
    g_elementary.current.sample_a_second = Board_GetCurrentSampleASecond();
    g_elementary.current.sample_b_first = Board_GetCurrentSampleBFirst();
    g_elementary.current.sample_b_second = Board_GetCurrentSampleBSecond();
    g_elementary.current.sample_spread_first = Board_GetCurrentSampleSpreadFirst();
    g_elementary.current.sample_spread_second = Board_GetCurrentSampleSpreadSecond();

    g_elementary.foc.id_ref = run.id_ref;
    g_elementary.foc.iq_ref = run.iq_ref;
    g_elementary.foc.id = run.id;
    g_elementary.foc.iq = run.iq;
    g_elementary.foc.vd = run.vd;
    g_elementary.foc.vq = run.vq;
    g_elementary.foc.kp = run.kp;
    g_elementary.foc.ki = run.ki;
    g_elementary.foc.v_limit = run.v_limit;
    g_elementary.foc.v_limited = run.v_limited;

    g_elementary.pwm.duty_u = run.duty_u;
    g_elementary.pwm.duty_v = run.duty_v;
    g_elementary.pwm.duty_w = run.duty_w;
    g_elementary.pwm.output_on = run.pwm_output_on;
    g_elementary.pwm.brake_on = run.pwm_brake_on;

    g_elementary.speed.ref = run.speed_ref;
    g_elementary.speed.fb = run.speed_fb;

    Motor_GetCheck(&check);
    g_elementary.safety.sensor_ma600_ok = check.ma600_ok;
    g_elementary.safety.sensor_current_ok = check.current_ok;
    g_elementary.safety.current_over_count = run.current_over_count;

    Motor_OpenLoopFillSnap(&open_loop);
    g_elementary.open_loop.theta = open_loop.theta;
    g_elementary.open_loop.vf_voltage = open_loop.vf_voltage;
    g_elementary.open_loop.if_id_ref = open_loop.if_id_ref;
    g_elementary.open_loop.if_iq_ref = open_loop.if_iq_ref;
    g_elementary.open_loop.current_over_count = open_loop.current_over_count;

    Motor_ZeroScanTestFillSnap(&zero_scan);
    g_elementary.zero_scan_test.state = zero_scan.state;
    g_elementary.zero_scan_test.phase = zero_scan.phase;
    g_elementary.zero_scan_test.iq_ref = zero_scan.iq_ref;
    g_elementary.zero_scan_test.trim = zero_scan.trim;
    g_elementary.zero_scan_test.best_trim = zero_scan.best_trim;
    g_elementary.zero_scan_test.score = zero_scan.score;
    g_elementary.zero_scan_test.best_score = zero_scan.best_score;
    g_elementary.zero_scan_test.id_avg = zero_scan.id_avg;
    g_elementary.zero_scan_test.iq_avg = zero_scan.iq_avg;
    g_elementary.zero_scan_test.iq_err_avg = zero_scan.iq_err_avg;
    g_elementary.zero_scan_test.vq_avg = zero_scan.vq_avg;
    g_elementary.zero_scan_test.best_id_avg = zero_scan.best_id_avg;
    g_elementary.zero_scan_test.best_iq_err_avg = zero_scan.best_iq_err_avg;
    g_elementary.zero_scan_test.best_vq_avg = zero_scan.best_vq_avg;
    g_elementary.zero_scan_test.fault = zero_scan.fault;
}

void Debug_RttTask(void)
{
    if (g_cmd.debug.rtt_enable == 0u)
    {
        return;
    }

    s_rtt_div++;
    if (s_rtt_div < 50u)
    {
        return;
    }
    s_rtt_div = 0u;

    /*
     * RTT 输出只做低频状态摘要，不进入 ADC 中断。
     * 字段保持紧凑，便于脱离 Keil Debug 时观察运行趋势。
     */
    SEGGER_RTT_printf(0,
                      "st=%u,en=%u,mode=%u,idr=%d,iqr=%d,id=%d,iq=%d,vd=%d,vq=%d,lim=%u,duty=%u/%u/%u,ang=%u,pos=%ld,spd=%ld/%ld,zst=%u,zph=%u,ziq=%d,ztr=%ld,zbs=%ld,zsc=%ld,zie=%d,zf=%u,fault=%u,oc=%u\r\n",
                      (unsigned)g_elementary.motor.state,
                      (unsigned)g_elementary.motor.enable,
                      (unsigned)g_elementary.motor.ctrl_mode,
                      (int)g_elementary.foc.id_ref,
                      (int)g_elementary.foc.iq_ref,
                      (int)g_elementary.foc.id,
                      (int)g_elementary.foc.iq,
                      (int)g_elementary.foc.vd,
                      (int)g_elementary.foc.vq,
                      (unsigned)g_elementary.foc.v_limited,
                      (unsigned)g_elementary.pwm.duty_u,
                      (unsigned)g_elementary.pwm.duty_v,
                      (unsigned)g_elementary.pwm.duty_w,
                      (unsigned)g_elementary.angle.elec,
                      (long)g_elementary.angle.pos,
                      (long)g_elementary.speed.ref,
                      (long)g_elementary.speed.fb,
                      (unsigned)g_elementary.zero_scan_test.state,
                      (unsigned)g_elementary.zero_scan_test.phase,
                      (int)g_elementary.zero_scan_test.iq_ref,
                      (long)g_elementary.zero_scan_test.trim,
                      (long)g_elementary.zero_scan_test.best_trim,
                      (long)g_elementary.zero_scan_test.score,
                      (int)g_elementary.zero_scan_test.iq_err_avg,
                      (unsigned)g_elementary.zero_scan_test.fault,
                      (unsigned)g_elementary.motor.fault_reason,
                      (unsigned)g_elementary.safety.current_over_count);
}
