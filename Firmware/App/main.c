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
    .iq_limit = MOTOR_SPEED_IQ_LIMIT_DEFAULT,
    .current_kp = MOTOR_CURRENT_KP,
    .current_ki = MOTOR_CURRENT_KI,
    .current_v_limit = MOTOR_CURRENT_V_LIMIT,
    .open_loop_speed_ref = MOTOR_OL_SPEED_REF_DEFAULT,
    .vf_voltage = MOTOR_VF_VOLTAGE_DEFAULT,
    .if_id_ref = MOTOR_IF_ID_REF_DEFAULT,
    .if_iq_ref = MOTOR_IF_IQ_REF_DEFAULT,
    .open_loop_timeout_ms = MOTOR_OL_TIMEOUT_MS_DEFAULT,
    .elec_zero_trim = 0,
};

volatile MotorControlWatch_t g_motor_watch;

int main(void)
{
    bsp_init();
    MotorControl_Init();

    while (1)
    {
        MotorControl_ApplyCommand(&g_motor_cmd);
        MotorControl_RunSlowLoop();
        MotorControl_UpdateWatch(&g_motor_watch);
    }
}

void ADC_IRQHandler(void)
{
    (void)MotorControl_FastLoopFromAdcIrq();
}
