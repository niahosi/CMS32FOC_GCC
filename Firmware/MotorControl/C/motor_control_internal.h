#pragma once

#include <stdint.h>

#include "Config.h"
#include "MotorControl.h"
#include "foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 控制层内部状态：空闲。 */
#define MC_STATE_IDLE 0U
/** @brief 控制层内部状态：闭环/诊断模式快环可运行。 */
#define MC_STATE_CLOSED_LOOP 3U
/** @brief 控制层内部状态：故障，PWM 应处于安全关闭。 */
#define MC_STATE_FAULT 4U

/** @brief 控制模式：关闭。 */
#define MC_MODE_OFF 0U
/** @brief 控制模式：电流环。 */
#define MC_MODE_CURRENT 1U
/** @brief 控制模式：速度环外环加电流环内环。 */
#define MC_MODE_SPEED 2U
/** @brief 诊断模式：VF 开环电压旋转。 */
#define MC_MODE_VF_OPEN_LOOP 3U
/** @brief 冻结模式：开环扫描对齐零位，cms32foc 主固件不支持。 */
#define MC_MODE_ALIGN_LOCK 4U
/** @brief 冻结模式：编码器角度下直接输出电压矢量，cms32foc 主固件不支持。 */
#define MC_MODE_ENCODER_VOLTAGE 5U

/** @brief 无故障。 */
#define MC_FAULT_NONE 0U
/** @brief 不支持的控制模式。 */
#define MC_FAULT_UNSUPPORTED_MODE 1U
/** @brief 电流采样超出安全范围。 */
#define MC_FAULT_CURRENT 2U
/** @brief VF 开环运行超时。 */
#define MC_FAULT_OPEN_LOOP_TIMEOUT 3U
/** @brief 编码器角度不可用或坏角过多。 */
#define MC_FAULT_ENCODER 4U

/** @brief 电流环执行频率，单位 Hz。 */
#define MC_CURRENT_HZ (PWM_FREQ_HZ / CTRL_FAST_LOOP_DIV)
/** @brief 每多少个电流环 tick 更新一次速度估算/速度环。 */
#define MC_SPEED_SAMPLE_DIV (MC_CURRENT_HZ / CTRL_SPD_EST_HZ)
/** @brief 每机械圈对应的电角 encoder count 数。 */
#define MC_SPEED_COUNTS_PER_REV ((int32_t)MOT_SENSOR_CPR * (int32_t)MOT_SENSOR_POLE_PAIRS)

/** @brief 电流快环直接使用的命令子集。 */
typedef struct
{
    int16_t id_ref;
    int16_t iq_ref;
    int16_t iq_limit;
    int16_t current_kp;
    int16_t current_ki;
    int16_t current_v_limit;
    int16_t elec_zero_trim;
    int16_t voltage_theta_offset;
} MCCurrentCommand_t;

/** @brief 速度估算/速度 PI 使用的命令子集。 */
typedef struct
{
    int32_t speed_ref;
    int16_t speed_kp;
    int16_t speed_ki;
    int16_t iq_limit;
} MCSpeedCommand_t;

/** @brief VF 应急开环使用的命令子集。 */
typedef struct
{
    int32_t open_loop_speed_ref;
    int16_t vf_voltage;
    uint16_t open_loop_timeout_ms;
} MCVfCommand_t;

/**
 * @brief C 控制层和 VF 应急开环模块共享的内部状态。
 *
 * 该结构不是公共 API，只在 MotorControl C 内部模块之间共享。
 * Current/Speed 主线、VF 开环、编码器、输出和 watch 共用同一份状态。
 */
typedef struct
{
    /** @brief 控制状态，见 MC_STATE_*。 */
    uint8_t state;
    /** @brief 故障原因，见 MC_FAULT_*。 */
    uint8_t fault;
    /** @brief 命令使能缓存。 */
    uint8_t enabled;
    /** @brief 当前控制模式，见 MC_MODE_*。 */
    uint8_t mode;
    /** @brief PWM 是否已请求输出。 */
    uint8_t pwm_output;
    /** @brief 主循环复制命令次数。 */
    uint32_t command_apply_count;
    /** @brief 慢环执行次数。 */
    uint32_t slow_loop_count;
    /** @brief 有效快环执行次数。 */
    uint32_t fast_loop_count;
    /** @brief 速度环/编码器 reset 次数，用于诊断闭环是否反复重入。 */
    uint32_t speed_reset_count;
    /** @brief 进入 PWM 安全态次数，用于诊断 iq_cmd 是否被安全路径清零。 */
    uint32_t safe_state_count;
    /** @brief 速度环 PI 更新次数。 */
    uint32_t speed_loop_count;
    /** @brief 速度给定落入死区导致速度环清零的次数。 */
    uint32_t speed_deadband_count;
    /** @brief 快环分频计数，用于降低电流环执行频率。 */
    uint16_t current_loop_div;
    /** @brief 速度估算/速度环分频计数。 */
    uint16_t speed_sample_div;
    /** @brief 最近接受的 MA600 raw。 */
    uint16_t encoder_raw;
    /** @brief 闭环使用的 16-bit 电角度。 */
    uint16_t encoder_elec;
    /** @brief 上一次速度估算使用的 raw。 */
    uint16_t encoder_prev_raw;
    /** @brief 本次速度估算 raw 差分。 */
    int16_t encoder_delta;
    /** @brief 累计 raw 位置。 */
    int32_t encoder_pos;
    /** @brief 最近一次接受 raw 的步进。 */
    int16_t encoder_raw_step;
    /** @brief 最近一次拒绝 raw 的步进。 */
    int16_t encoder_reject_step;
    /** @brief 拒绝前上一帧 raw。 */
    uint16_t encoder_reject_prev_raw;
    /** @brief 最近一次拒绝的 raw。 */
    uint16_t encoder_reject_raw;
    /** @brief raw 坏角拒绝计数。 */
    uint32_t encoder_reject_count;
    /** @brief 坏角后即时重读计数。 */
    uint32_t encoder_retry_count;
    /** @brief 即时重读成功接受计数。 */
    uint32_t encoder_retry_accept_count;
    /** @brief 最近一次即时重读 raw。 */
    uint16_t encoder_retry_raw;
    /** @brief 因读取失败或坏角而保持上一角度的次数。 */
    uint32_t encoder_hold_count;
    /** @brief 速度估算拒绝异常 raw 差分的次数。 */
    uint32_t speed_reject_count;
    /** @brief 最近一次被速度估算拒绝的 raw 差分。 */
    int16_t speed_reject_delta;
    /** @brief 当前选用速度反馈，编码器电角 count/s。 */
    int32_t speed_fb;
    /** @brief 速度环内部斜坡后的目标，编码器电角 count/s。 */
    int32_t speed_ref_active;
    /** @brief raw 差分速度反馈，编码器电角 count/s。 */
    int32_t speed_fb_diff;
    /** @brief 速度环 rpm 误差。 */
    int16_t speed_err_rpm;
    /** @brief 速度环斜率限制前的 q 轴电流目标。 */
    int16_t speed_iq_target;
    /** @brief 速度目标产生的 q 轴电流前馈。 */
    int16_t speed_iq_ff;
    /** @brief 速度环输出的 q 轴电流给定。 */
    int16_t speed_iq_ref;
    /** @brief 模式切入后速度估算空白窗口计数。 */
    uint8_t speed_startup_blank;
    /** @brief 角度缓存年龄。 */
    uint8_t encoder_age;
    /** @brief 编码器角度是否可用于闭环。 */
    uint8_t encoder_ok;
    /** @brief 编码器状态是否已收到首帧 raw。 */
    uint8_t encoder_initialized;
    /** @brief 速度环 PI。 */
    FocPi_t speed_pi;
    /** @brief d 轴电流 PI。 */
    FocPi_t current_pi_d;
    /** @brief q 轴电流 PI。 */
    FocPi_t current_pi_q;
    /** @brief 电流快环直接读取的命令缓存。 */
    MCCurrentCommand_t current_command;
    /** @brief 速度估算/速度 PI 直接读取的命令缓存。 */
    MCSpeedCommand_t speed_command;
    /** @brief VF 应急开环直接读取的命令缓存。 */
    MCVfCommand_t vf_command;
    /** @brief 最新三相电流。 */
    FocPhaseCurrent_t current;
    /** @brief Park 后 dq 电流。 */
    FocDq_t current_dq;
    /** @brief 当前实际 d 轴给定。 */
    int16_t id_ref_active;
    /** @brief 当前实际 q 轴给定。 */
    int16_t iq_ref_active;
    /** @brief 输出电压 alpha/beta。 */
    FocAlphaBeta_t voltage_ab;
    /** @brief 输出电压 dq。 */
    FocDq_t voltage_dq;
    /** @brief 输出电压使用的 16-bit 电角度。 */
    uint16_t voltage_theta;
    /** @brief SVPWM duty 输出。 */
    FocDuty_t duty;
    /** @brief 输出电压是否被矢量限幅。 */
    uint8_t voltage_limited;
    /** @brief 慢环安全检查结果。 */
    MotorControlCheck_t check;
} MotorControlCState;

/** @brief 检查内部状态中的三相电流是否处于安全范围。 */
uint8_t MotorControl_InternalCurrentOk(MotorControlCState *mc);
/** @brief 清空编码器、速度反馈和坏角诊断状态。 */
void MotorControl_EncoderReset(MotorControlCState *mc);
/** @brief 快环读取/校验/接受一次编码器角度，带坏角即时重读。 */
uint8_t MotorControl_InternalUpdateEncoderAngle(MotorControlCState *mc);
/** @brief 按当前编码器 raw 更新速度反馈。 */
uint8_t MotorControl_InternalUpdateEncoderSpeed(MotorControlCState *mc);
/** @brief 清空电流 PI 和当前电流给定斜坡状态。 */
void MotorControl_CurrentReset(MotorControlCState *mc);
/** @brief 清空速度 PI、速度估算和编码器状态。 */
void MotorControl_SpeedReset(MotorControlCState *mc);
/** @brief 运行 Current/Speed 主线快环；Current 也更新速度观察，Speed 额外运行速度 PI。 */
void MotorControl_CurrentRunFastLoop(MotorControlCState *mc, uint8_t speed_mode);
/** @brief 获取当前电流环/诊断电压限幅，命令未设置时回退默认值。 */
int16_t MotorControl_InternalVoltageLimit(const MotorControlCState *mc);
/** @brief 输出 dq 电压矢量，统一执行限幅、反 Park、SVPWM 和 PWM 使能。 */
void MotorControl_InternalApplyVoltageVector(MotorControlCState *mc,
                                             int16_t vd,
                                             int16_t vq,
                                             uint16_t theta);
/** @brief 进入安全态，关闭 PWM 并清零输出相关状态。 */
void MotorControl_InternalEnterSafeState(MotorControlCState *mc);
/** @brief 将编码器电角 count/s 转为 rpm 观察单位。 */
int16_t MotorControl_InternalSpeedCountsToRpm(int32_t speed);
/** @brief 填充 Current/Speed/VF 主线 watch 快照。 */
void MotorControl_WatchFill(const MotorControlCState *mc, MotorControlWatch_t *out);
/** @brief 将主线 watch 快照逐字段写入 volatile 目标。 */
void MotorControl_WatchCopyToVolatile(volatile MotorControlWatch_t *dst,
                                      const MotorControlWatch_t *src);

#ifdef __cplusplus
}
#endif
