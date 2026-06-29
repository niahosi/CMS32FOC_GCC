/**
 * @file main.c
 * @brief CMS32FOCAC6 main entry.
 * @details 主循环负责低频调度和调试命令下发；ADC 中断负责同步采样后的快环入口。
 */

#include "App_Debug.h"
#include "Board.h"
#include "Board_Analog.h"
#include "Motor.h"
#include "Motor_ZeroScanTest.h"
#include "delay.h"

static void DebugApplyControl(void);

int main(void)
{
    /* 先初始化板级硬件安全态，再初始化 Motor 状态机和调试输出。 */
    Board_Init();
    Motor_Init();
    Debug_InitRtt();

    while (1)
    {
        /* Watch/JScope 写入 g_cmd 后，在这里统一转成 Motor 层 setter。 */
        DebugApplyControl();
        Motor_TASK();
        Motor_ZeroScanTestTask(&g_cmd.zero_scan_test);

        /* 调试快照放在主循环刷新，避免在 ADC 中断里做额外格式化或聚合。 */
        Debug_UpdateElementary();
        Debug_RttTask();
        m0_delay_us(200);
    }
}

void ADC_IRQHandler(void)
{
    if (Board_HandleAdcIrq() != 0)
    {
        /*
         * EPWM CMP0 触发的一组电流采样完成后，立即同步读取 MA600。
         * Motor_FastLoop() 使用同一 PWM 周期内的电流和角度缓存。
         */
        Board_UpdateAngleFast();
        Motor_UpdateAngleFromIsr();

        if (g_cmd.run.motor_enable == 0u)
        {
            Motor_SetEnable(0u);
        }
        else
        {
            Motor_FastLoop();
        }
    }

    NVIC_ClearPendingIRQ(ADC_IRQn);
}

static void DebugApplyControl(void)
{
    /*
     * 参数先于模式和使能下发，避免模式切换后的第一拍快环读到旧给定。
     */
    if (g_cmd.zero_scan_test.enable != 0u)
    {
        Motor_SetElecZero((uint16_t)(MOTOR_ELEC_ZERO + Motor_ZeroScanTestGetTrim()));
        Motor_SetCurrentRef(0, Motor_ZeroScanTestGetIqRef());
    }
    else
    {
        Motor_SetElecZero((uint16_t)(MOTOR_ELEC_ZERO + g_cmd.angle.elec_zero_trim));
        Motor_SetCurrentRef(g_cmd.current.id_ref, g_cmd.current.iq_ref);
    }

    Motor_SetCurrentPi(g_cmd.current.kp, g_cmd.current.ki, g_cmd.current.v_limit);
    Motor_SetIqLimit(g_cmd.speed.iq_limit);
    Motor_SetSpeedRef(g_cmd.speed.speed_ref);
    Motor_SetOlSpeedRef(g_cmd.open_loop.ol_speed_ref);
    Motor_SetVfVoltage(g_cmd.open_loop.vf_voltage);
    Motor_SetIfCurrentRef(g_cmd.open_loop.if_id_ref, g_cmd.open_loop.if_iq_ref);
    Motor_SetOlTimeoutMs(g_cmd.open_loop.ol_timeout_ms);
    /*
     * 模式与使能
     */
    Motor_SetControlMode((MotorControlMode_t)g_cmd.run.ctrl_mode);
    Motor_SetEnable(g_cmd.run.motor_enable);
}
