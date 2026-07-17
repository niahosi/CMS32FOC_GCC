#include "motor_control_vf.h"

#include "BoardConfig.h"
#include "MotorControl.h"
#include "TuneConfig.h"
#include "foc_curr.h"
#include "motor_control_internal.h"
#include <stdint.h>

/** @brief VF 应急开环私有状态，独立于 Current/Speed 主线。 */
typedef struct
{
    uint32_t open_loop_ticks;
    uint32_t open_loop_reset_count;
    uint32_t open_loop_timeout_ticks;
    int32_t open_loop_theta_acc;
    uint16_t open_loop_theta;
} MotorControlVfState;

static MotorControlVfState s_vf;

static void reset_open_loop_theta(void);
static uint16_t update_open_loop_theta(const MCVfCommand_t* command);

/** @brief 初始化 VF 开环状态。 */
void MotorControlVf_Init(void)
{
    s_vf.open_loop_timeout_ticks = (uint32_t)OL_TIMEOUT_MS * 2U;
    s_vf.open_loop_ticks = 0U;
    s_vf.open_loop_theta = 0U;
    s_vf.open_loop_theta_acc = 0;
    s_vf.open_loop_reset_count = 0U;
}

/** @brief 仅在明确切入 VF 模式时重置开环角。 */
void MotorControlVf_ResetForMode(uint8_t mode)
{
    if (mode == MC_MODE_VF_OPEN_LOOP)
    {
        reset_open_loop_theta();
    }
}

/** @brief 运行 VF 应急开环快环，输出 q 轴开环电压并保留基础电流保护。 */
void MotorControlVf_RunFastLoop(MotorControlCState* mc)
{
    uint16_t theta;

    mc->current.u = curr_u();
    mc->current.v = curr_v();
    mc->current.w = curr_w();
    if (MotorControl_InternalCurrentOk(mc) == 0U)
    {
        mc->state = MC_STATE_FAULT;
        mc->fault = MC_FAULT_CURRENT;
        MotorControl_InternalEnterSafeState(mc);
        return;
    }

    s_vf.open_loop_timeout_ticks = (uint32_t)mc->vf_command.open_loop_timeout_ms * 2U;
    if ((s_vf.open_loop_timeout_ticks > 0U) &&
        (s_vf.open_loop_ticks >= s_vf.open_loop_timeout_ticks))
    {
        mc->state = MC_STATE_FAULT;
        mc->fault = MC_FAULT_OPEN_LOOP_TIMEOUT;
        MotorControl_InternalEnterSafeState(mc);
        return;
    }

    theta = update_open_loop_theta(&mc->vf_command);
    MotorControl_InternalApplyVoltageVector(mc, 0, mc->vf_command.vf_voltage, theta);
    if (++mc->speed_sample_div >= MC_SPEED_SAMPLE_DIV)
    {
        mc->speed_sample_div = 0U;
        if (MotorControl_InternalUpdateEncoderAngle(mc) != 0U)
        {
            (void)MotorControl_InternalUpdateEncoderSpeed(mc);
        }
    }
    mc->fast_loop_count++;
}

/** @brief 获取 VF 开环角。 */
uint16_t MotorControlVf_OpenLoopTheta(void)
{
    return s_vf.open_loop_theta;
}

/** @brief 获取 VF 开环角累计 tick。 */
uint32_t MotorControlVf_OpenLoopTicks(void)
{
    return s_vf.open_loop_ticks;
}

/** @brief 获取 VF 开环角重置次数。 */
uint32_t MotorControlVf_OpenLoopResetCount(void)
{
    return s_vf.open_loop_reset_count;
}

/** @brief 重置 VF 开环角并递增 reset 计数。 */
static void reset_open_loop_theta(void)
{
    s_vf.open_loop_ticks = 0U;
    s_vf.open_loop_theta = 0U;
    s_vf.open_loop_theta_acc = 0;
    s_vf.open_loop_reset_count++;
}

/** @brief 按命令 open_loop_speed_ref 推进 VF 开环角。 */
static uint16_t update_open_loop_theta(const MCVfCommand_t* command)
{
    int32_t step = (command->open_loop_speed_ref * (int32_t)OL_SPEED_TO_THETA_STEP +
                    (1L << (OL_SPEED_TO_THETA_SHIFT - 1U))) >>
                   OL_SPEED_TO_THETA_SHIFT;
    s_vf.open_loop_theta_acc += step;
    s_vf.open_loop_theta = (uint16_t)((uint32_t)s_vf.open_loop_theta_acc & 0xFFFFUL);
    s_vf.open_loop_ticks++;
    return (uint16_t)((int32_t)s_vf.open_loop_theta * (int32_t)MOT_SENSOR_DIR);
}
