#pragma once

#include <stdint.h>

#include "BoardConfig.h"
#include "Config.h"
#include "MotorControl.h"
#include "TuneConfig.h"
#include "foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 控制层内部状态原始值；g_motor.runtime.state 直接使用该枚举，方便 Ozone
 * 显示状态名。 */
typedef enum
{
    /** @brief 空闲。 */
    MC_STATE_IDLE = 0U,
    /** @brief 闭环/诊断模式快环可运行。 */
    MC_STATE_CLOSED_LOOP = 3U,
    /** @brief 故障，PWM 应处于安全关闭。 */
    MC_STATE_FAULT = 4U,
} MotorControlStateRaw_t;

/** @brief 控制模式原始值；g_motor.runtime.mode 直接使用该枚举，公共命令仍用 uint8_t。
 */
typedef enum
{
    /** @brief 关闭。 */
    MC_MODE_OFF = 0U,
    /** @brief 电流环。 */
    MC_MODE_CURRENT = 1U,
    /** @brief 速度环外环加电流环内环。 */
    MC_MODE_SPEED = 2U,
    /** @brief 诊断模式：VF 开环电压旋转。 */
    MC_MODE_VF_OPEN_LOOP = 3U,
    /** @brief 冻结模式：开环扫描对齐零位，cms32foc 主固件不支持。 */
    MC_MODE_ALIGN_LOCK = 4U,
    /** @brief 冻结模式：编码器角度下直接输出电压矢量，cms32foc 主固件不支持。 */
    MC_MODE_ENCODER_VOLTAGE = 5U,
    /** @brief 位置环外环加速度环、电流环。 */
    MC_MODE_POSITION = 6U,
} MotorControlModeRaw_t;

/** @brief 故障原因原始值；g_motor.runtime.fault 直接使用该枚举，方便 Ozone 显示故障名。
 */
typedef enum
{
    /** @brief 无故障。 */
    MC_FAULT_NONE = 0U,
    /** @brief 不支持的控制模式。 */
    MC_FAULT_UNSUPPORTED_MODE = 1U,
    /** @brief 电流采样超出安全范围。 */
    MC_FAULT_CURRENT = 2U,
    /** @brief VF 开环运行超时。 */
    MC_FAULT_OPEN_LOOP_TIMEOUT = 3U,
    /** @brief 编码器角度读取不可用。 */
    MC_FAULT_ENCODER = 4U,
} MotorControlFaultRaw_t;

/** @brief 电流环执行频率，单位 Hz。 */
#define MC_CURRENT_HZ (PWM_FREQ_HZ / CTRL_FAST_LOOP_DIV)
/** @brief 每多少个电流环 tick 更新一次速度估算/速度环。 */
#define MC_SPEED_SAMPLE_DIV (MC_CURRENT_HZ / CTRL_SPD_EST_HZ)
/** @brief 每机械圈对应的电角 encoder count 数。 */
#define MC_SPEED_COUNTS_PER_REV ((int32_t)MOT_SENSOR_CPR * (int32_t)MOT_POLE_PAIRS)

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

/** @brief 位置 PI/P 使用的命令子集。 */
typedef struct
{
    int32_t position_ref;
    int16_t position_kp;
    uint8_t position_err_shift;
    int16_t speed_limit_rpm;
    int32_t deadband_counts;
} MCPositionCommand_t;

/** @brief VF 应急开环使用的命令子集。 */
typedef struct
{
    int32_t open_loop_speed_ref;
    int16_t vf_voltage;
    uint16_t open_loop_timeout_ms;
} MCVfCommand_t;

/** @brief 慢环状态机当前运行状态。 */
typedef struct
{
    /** @brief 控制状态。 */
    MotorControlStateRaw_t state;
    /** @brief 故障原因。 */
    MotorControlFaultRaw_t fault;
    /** @brief 命令使能缓存。 */
    uint8_t enabled;
    /** @brief 当前控制模式。 */
    MotorControlModeRaw_t mode;
    /** @brief PWM 是否已请求输出。 */
    uint8_t pwm_output;
} MCRuntimeState_t;

/** @brief 慢环安全检查快照，只用于内部状态和 Ozone 观察。 */
typedef struct
{
    /** @brief MA600 角度缓存是否可用于闭环。 */
    uint8_t ma600_ok;
    /** @brief 三相电流是否在硬限范围内。 */
    uint8_t current_ok;
    /** @brief PWM 关闭时功率级是否处于安全态。 */
    uint8_t pwm_off_safe;
    /** @brief 当前模式是否满足进入闭环快环的条件。 */
    uint8_t ready_closed_loop;
} MotorControlCheck_t;

/** @brief ADC 快环直接读取的命令缓存。 */
typedef struct
{
    /** @brief 电流快环直接读取的命令缓存。 */
    MCCurrentCommand_t current;
    /** @brief 速度估算/速度 PI 直接读取的命令缓存。 */
    MCSpeedCommand_t speed;
    /** @brief 位置环直接读取的命令缓存。 */
    MCPositionCommand_t position;
    /** @brief VF 应急开环直接读取的命令缓存。 */
    MCVfCommand_t vf;
} MCCommandCache_t;

/** @brief 编码器闭环状态，只放会参与角度/速度/位置计算的数据。 */
typedef struct
{
    /** @brief 最近接受的 MA600 raw。 */
    uint16_t raw;
    /** @brief 闭环使用的 16-bit 电角度。 */
    uint16_t elec;
    /** @brief 上一次速度估算使用的 raw。 */
    uint16_t prev_raw;
    /** @brief 本次速度估算 raw 差分。 */
    int16_t delta;
    /** @brief 累计 raw 位置。 */
    int32_t pos;
    /** @brief 角度缓存年龄。 */
    uint8_t age;
    /** @brief 编码器角度是否可用于闭环。 */
    uint8_t ok;
    /** @brief 编码器状态是否已收到首帧 raw。 */
    uint8_t initialized;
} MCEncoderState_t;

/** @brief PWM 输出链路状态，保存本次 FOC 输出结果。 */
typedef struct
{
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
} MCOutputState_t;

/** @brief Ozone 诊断字段，不参与闭环控制决策。 */
typedef struct
{
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
    /** @brief 位置环更新次数。 */
    uint32_t position_loop_count;
    /** @brief 速度给定落入死区导致速度环清零的次数。 */
    uint32_t speed_deadband_count;
    /** @brief 最近一次接受 raw 的步进。 */
    int16_t encoder_raw_step;
    /** @brief 最近一次拒绝 raw 的步进。 */
    int16_t encoder_reject_step;
    /** @brief 拒绝前上一帧 raw。 */
    uint16_t encoder_reject_prev_raw;
    /** @brief 最近一次拒绝的 raw。 */
    uint16_t encoder_reject_raw;
    /** @brief 20 kHz 位置累计拒绝明显异常 raw 跳变的次数。 */
    uint32_t encoder_reject_count;
    /** @brief 坏角后即时重读计数；当前生产 C++ 路径不再即时重读，仅保留 ABI/调试兼容。
     */
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
} MotorControlDiag_t;

/** @brief 将编码器电角 count/s 转为 rpm 观察单位。 */
int16_t MotorControl_InternalSpeedCountsToRpm(int32_t speed);

#ifdef __cplusplus
}
#endif
