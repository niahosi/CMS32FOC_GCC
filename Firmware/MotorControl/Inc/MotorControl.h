#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ozone/主循环写入的电机控制命令。
 *
 * 该结构由主循环复制到控制层内部缓存，ADC 中断快环只读取缓存副本。
 * Current/Speed 主线只依赖闭环相关字段；VF/Align/EncoderVoltage 字段保留为诊断入口。
 */
typedef struct
{
    /** @brief 非 0 允许控制状态机进入指定控制模式。 */
    uint8_t enable;
    /** @brief 控制模式：1 Current, 2 Speed, 3 VF, 4 Align, 5 EncoderVoltage。 */
    uint8_t control_mode;
    /** @brief d 轴电流给定，内部 ADC count 缩放单位。 */
    int16_t id_ref;
    /** @brief q 轴电流给定，内部 ADC count 缩放单位。 */
    int16_t iq_ref;
    /** @brief 速度给定，单位为编码器电角 count/s。 */
    int32_t speed_ref;
    /** @brief rpm 速度给定入口；非 0 时覆盖 speed_ref。 */
    int16_t speed_ref_rpm;
    /** @brief q 轴电流命令限幅，内部 ADC count 缩放单位。 */
    int16_t iq_limit;
    /** @brief 电流环比例增益，定点 PI 参数。 */
    int16_t current_kp;
    /** @brief 电流环积分增益，定点 PI 参数。 */
    int16_t current_ki;
    /** @brief 速度环比例增益，rpm 误差输入。 */
    int16_t speed_kp;
    /** @brief 速度环积分增益，rpm 误差输入。 */
    int16_t speed_ki;
    /** @brief 电流环输出电压限幅，SVPWM count 单位。 */
    int16_t current_v_limit;
    /** @brief 诊断模式：VF 开环速度命令，内部开环角步进单位。 */
    int32_t open_loop_speed_ref;
    /** @brief 诊断模式：VF 开环 q 轴电压，SVPWM count 单位。 */
    int16_t vf_voltage;
    /** @brief 预留 IF 诊断 d 轴给定。 */
    int16_t if_id_ref;
    /** @brief 预留 IF 诊断 q 轴给定。 */
    int16_t if_iq_ref;
    /** @brief VF 开环超时时间，ms；0 表示不超时。 */
    uint16_t open_loop_timeout_ms;
    /** @brief 电角度零位临时修正，叠加到 MOT_ELEC_ZERO。 */
    int16_t elec_zero_trim;
    /** @brief 输出电压角度偏置，用于相位提前/滞后诊断。 */
    int16_t voltage_theta_offset;
} MotorControlCommand_t;

/** @brief 慢环安全检查快照。 */
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

/**
 * @brief Current/Speed 主线 Ozone watch。
 *
 * 只保留闭环调试必须字段。诊断模式、坏角重试、MA600 对比速度等字段见
 * MotorControlDiagWatch_t。Current/Speed 运行不依赖诊断 watch。
 */
typedef struct
{
    /** @brief 控制状态：0 idle, 3 closed-loop, 4 fault。 */
    uint8_t state;
    /** @brief 当前实际控制模式。 */
    uint8_t control_mode;
    /** @brief 故障原因枚举，见 motor_control_internal.h 中 MC_FAULT_*。 */
    uint8_t fault_reason;
    /** @brief 命令使能缓存。 */
    uint8_t enable;
    /** @brief 慢环运行次数。 */
    uint32_t slow_loop_count;
    /** @brief 有效快环运行次数。 */
    uint32_t fast_loop_count;
    /** @brief PWM/ADC 同步采样次数。 */
    uint32_t adc_sample_count;
    /** @brief 最近一次 MA600 原始角度。 */
    uint16_t encoder_raw;
    /** @brief 闭环使用的电角度，16-bit 周期角。 */
    uint16_t encoder_elec;
    /** @brief 速度估算采样间隔内 raw 增量。 */
    int16_t encoder_delta;
    /** @brief 累计 raw 位置，允许跨 16-bit 回绕。 */
    int32_t encoder_pos;
    /** @brief 角度缓存年龄，超过阈值会判为不可用。 */
    uint8_t encoder_age;
    /** @brief 当前编码器角度是否可用于闭环。 */
    uint8_t encoder_ok;
    /** @brief 逻辑 U 相电流，内部 ADC count 缩放单位。 */
    int16_t iu_cnt;
    /** @brief 逻辑 V 相电流，内部 ADC count 缩放单位。 */
    int16_t iv_cnt;
    /** @brief 逻辑 W 相电流，内部 ADC count 缩放单位。 */
    int16_t iw_cnt;
    /** @brief 三相逻辑电流和，用于 KCL 检查。 */
    int16_t i_sum;
    /** @brief 实际送入电流环的 d 轴给定。 */
    int16_t id_ref;
    /** @brief 实际送入电流环的 q 轴给定。 */
    int16_t iq_ref;
    /** @brief 速度给定，编码器电角 count/s。 */
    int32_t speed_ref;
    /** @brief 速度给定，rpm 观察单位。 */
    int16_t speed_ref_rpm;
    /** @brief 当前选用的速度反馈，编码器电角 count/s。 */
    int32_t speed_fb;
    /** @brief 当前选用的速度反馈，rpm 观察单位。 */
    int16_t speed_fb_rpm;
    /** @brief 速度环 rpm 误差。 */
    int16_t speed_err_rpm;
    /** @brief 速度环输出的 q 轴电流命令。 */
    int16_t speed_iq_cmd;
    /** @brief 速度 PI 积分项，定点内部值。 */
    int32_t speed_pi_integral;
    /** @brief Park 后 d 轴电流反馈。 */
    int16_t id;
    /** @brief Park 后 q 轴电流反馈。 */
    int16_t iq;
    /** @brief 电流 PI 输出 d 轴电压，SVPWM count 单位。 */
    int16_t vd;
    /** @brief 电流 PI 输出 q 轴电压，SVPWM count 单位。 */
    int16_t vq;
    /** @brief 本拍输出电压使用的电角度。 */
    uint16_t voltage_theta;
    /** @brief 电压矢量是否被限幅。 */
    uint8_t v_limited;
    /** @brief U 相 PWM duty count。 */
    uint16_t duty_u;
    /** @brief V 相 PWM duty count。 */
    uint16_t duty_v;
    /** @brief W 相 PWM duty count。 */
    uint16_t duty_w;
    /** @brief PWM 层是否处于安全状态。 */
    uint8_t pwm_safe;
    /** @brief PWM 是否正在输出。 */
    uint8_t pwm_running;
    /** @brief 慢环安全检查快照。 */
    MotorControlCheck_t check;
} MotorControlWatch_t;

/**
 * @brief 诊断模式和辅助观测 watch。
 *
 * 这里承接 VF/Align/EncoderVoltage、坏角重试、MA600 speed 对比和命令镜像。
 * 不应让 Current/Speed 主线依赖这些字段才能运行。
 */
typedef struct
{
    /** @brief 开环角重置次数；VF 运行中不应持续增加。 */
    uint32_t open_loop_reset_count;
    /** @brief 开环角累计 tick。 */
    uint32_t open_loop_ticks;
    /** @brief 诊断模块生成的开环角，16-bit 周期角。 */
    uint16_t open_loop_theta;
    /** @brief 实际输出电压角。 */
    uint16_t voltage_theta;
    /** @brief 最近一次 raw 接受步进。 */
    int16_t encoder_raw_step;
    /** @brief 最近一次被拒绝 raw 的步进。 */
    int16_t encoder_reject_step;
    /** @brief 拒绝前的上一帧 raw。 */
    uint16_t encoder_reject_prev_raw;
    /** @brief 最近一次被拒绝的 raw。 */
    uint16_t encoder_reject_raw;
    /** @brief raw 跳变拒绝累计次数。 */
    uint32_t encoder_reject_count;
    /** @brief 坏角后即时重读尝试次数。 */
    uint32_t encoder_retry_count;
    /** @brief 即时重读成功并接受的次数。 */
    uint32_t encoder_retry_accept_count;
    /** @brief 最近一次即时重读 raw。 */
    uint16_t encoder_retry_raw;
    /** @brief Align 扫描是否完成。 */
    uint8_t align_done;
    /** @brief Align 扫描 tick。 */
    uint32_t align_ticks;
    /** @brief Align 目标电角度。 */
    uint16_t align_theta;
    /** @brief Align 采样时的 raw。 */
    uint16_t align_raw;
    /** @brief Align 得到的电角零位 trim。 */
    int16_t align_zero_trim;
    /** @brief Align 采样时的编码器电角度。 */
    uint16_t align_encoder_elec;
    /** @brief Align 扫描阶段。 */
    uint8_t align_stage;
    /** @brief 当前样本相对首次样本的 trim 拉偏。 */
    int16_t align_pull_delta;
    /** @brief Align trim 采样数。 */
    uint16_t align_sample_count;
    /** @brief Align trim 差值累计和。 */
    int32_t align_delta_sum;
    /** @brief raw 差分测速反馈，编码器电角 count/s。 */
    int32_t speed_fb_diff;
    /** @brief raw 差分测速反馈，rpm。 */
    int16_t speed_fb_diff_rpm;
    /** @brief MA600 speed frame 换算后的反馈，编码器电角 count/s。 */
    int32_t speed_fb_ma600;
    /** @brief MA600 speed frame 换算后的反馈，rpm。 */
    int16_t speed_fb_ma600_rpm;
    /** @brief MA600 speed 原始 signed 16-bit 输出。 */
    int16_t ma600_speed_raw;
    /** @brief 当前编译选择的速度反馈来源。 */
    uint8_t speed_fb_source;
    /** @brief 命令复制次数。 */
    uint32_t command_apply_count;
    /** @brief 命令 enable 镜像。 */
    uint8_t command_enable;
    /** @brief 命令 control_mode 镜像。 */
    uint8_t command_control_mode;
    /** @brief 命令 VF 电压镜像。 */
    int16_t command_vf_voltage;
    /** @brief 命令开环速度镜像。 */
    int32_t command_open_loop_speed_ref;
    /** @brief 命令 rpm 速度给定镜像。 */
    int16_t command_speed_ref_rpm;
    /** @brief 命令 q 轴电流限幅镜像。 */
    int16_t command_iq_limit;
    /** @brief 命令电压限幅镜像。 */
    int16_t command_current_v_limit;
    /** @brief 命令输出电压角偏置镜像。 */
    int16_t command_voltage_theta_offset;
    /** @brief 命令速度环 kp 镜像。 */
    int16_t command_speed_kp;
    /** @brief 命令速度环 ki 镜像。 */
    int16_t command_speed_ki;
} MotorControlDiagWatch_t;

/** @brief 初始化 C 控制层状态和 PI 参数，保持 PWM 关闭。 */
void MotorControl_Init(void);
/** @brief 从 volatile 命令入口复制并限幅命令，只在主循环调用。 */
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command);
/** @brief 慢环安全检查和状态机更新，只在主循环调用。 */
void MotorControl_RunSlowLoop(void);
/** @brief ADC IRQ 快环入口；返回 1 表示本次形成有效控制采样。 */
uint8_t MotorControl_FastLoopFromAdcIrq(void);
/** @brief 获取主线 watch 快照。 */
void MotorControl_GetWatch(MotorControlWatch_t* out);
/** @brief 将主线 watch 快照写入 volatile Ozone 变量。 */
void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out);
/** @brief 获取诊断 watch 快照。 */
void MotorControl_GetDiagWatch(MotorControlDiagWatch_t* out);
/** @brief 将诊断 watch 快照写入 volatile Ozone 变量。 */
void MotorControl_UpdateDiagWatch(volatile MotorControlDiagWatch_t* out);

#ifdef __cplusplus
}
#endif
