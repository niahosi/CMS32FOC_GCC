#include "MotorControl.h"
#include "Config.h"
#include "cms32m6510_platform.h" // IWYU pragma: keep
#include "foc_bsp.h"

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
    .open_loop_speed_ref = OL_SPEED_REF,
    .vf_voltage = OL_VF_VOLTAGE,
    .if_id_ref = OL_IF_ID_REF,
    .if_iq_ref = OL_IF_IQ_REF,
    .open_loop_timeout_ms = OL_TIMEOUT_MS,
    .elec_zero_trim = 0,
    .voltage_theta_offset = 0,
};

 volatile MotorControlWatch_t g_motor_watch;
// volatile uint32_t g_app_boot_stage;
// volatile uint32_t g_app_loop_count;
// volatile uint32_t g_app_adc_irq_count;
// volatile uint32_t g_app_adc_ready_count;

int main(void)
{
    bsp_init();
    MotorControl_Init();
    MotorControl_UpdateWatch(&g_motor_watch);
    bsp_start_adc_sync();

    while (1)
    {
        MotorControl_ApplyCommand(&g_motor_cmd);
        MotorControl_RunSlowLoop();
        MotorControl_UpdateWatch(&g_motor_watch);
    }
}

void ADC_IRQHandler(void)
{
    if (MotorControl_FastLoopFromAdcIrq() != 0U)
    {
        // g_app_adc_ready_count++;
    }
}
