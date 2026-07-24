#pragma once

#include <stdint.h>

/*
 * ScrewAxis 是 App 层的机械轴控制入口。
 *
 * 这个头文件故意保持 C ABI：
 * - main.c 可以直接 include；
 * - Ozone 可以稳定观察 g_screw_axis_cmd / g_screw_axis_watch；
 * - C++ 实现细节留在 screw_axis.cpp 内部，不从公共接口泄漏出来。
 *
 * 当前主固件保留双端回零/行程标定，并提供 mm_x100 物理位置调试命令。
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * 回零细分状态。
 *
 * 这些数值会暴露给 Ozone/watch。改枚举顺序或数值时，要同步文档和调试脚本；
 * 不要只把它当成普通 C enum 随手重排。
 */
typedef enum
{
    /** 未运行，等待 home_enable 触发。 */
    ScrewHomeState_Idle = 0U,
    /** 正向伸出，寻找最大外端/堵转点。 */
    ScrewHomeState_SeekOuter = 1U,
    /** 快速负向收回，寻找最里面机械端点。 */
    ScrewHomeState_FastRetract = 2U,
    /** 正向退离内端，释放机械压紧。 */
    ScrewHomeState_FastBackoff = 3U,
    /** 低速负向精找内端零位。 */
    ScrewHomeState_SlowRetract = 4U,
    /** 记录内端零位后最终正向退离。 */
    ScrewHomeState_FinalBackoff = 5U,
    /** 回零完成，电机停止，零位和行程有效。 */
    ScrewHomeState_Done = 6U,
    /** 回零失败或底层电机控制报错。 */
    ScrewHomeState_Fault = 7U,
} ScrewHomeState_t;

/**
 * 轴级总状态。
 *
 * home_state 描述回零内部走到哪一步；state 描述整个轴对外处于什么状态。
 * 调试时先看 state，再看 home_state。
 */
typedef enum
{
    /** 未回零或空闲。 */
    ScrewAxisState_Idle = 0U,
    /** 正在执行双端回零。 */
    ScrewAxisState_Homing = 1U,
    /** 双端回零完成，零位和行程有效。 */
    ScrewAxisState_Ready = 2U,
    /** 轴层或底层 MotorControl 故障。 */
    ScrewAxisState_Fault = 3U,
} ScrewAxisState_t;

/** 位置环激励信号类型。数值直接暴露给 Ozone。 */
typedef enum
{
    /** 不生成测试轨迹，使用 position_enable 的 target_mm_x100。 */
    ScrewAxisTestMode_Off = 0U,
    /** 固定频率正弦位置轨迹。 */
    ScrewAxisTestMode_Sine = 1U,
    /** 固定频率方波位置阶跃轨迹。 */
    ScrewAxisTestMode_Step = 2U,
    /** 频率随时间线性变化的正弦 chirp 轨迹。 */
    ScrewAxisTestMode_Sweep = 3U,
} ScrewAxisTestMode_t;

/**
 * 轴级故障原因。
 *
 * 这里只记录 ScrewAxis 能直接判断的机械/流程故障。
 * 底层电流、编码器、PWM 等故障继续看 MotorControl 的 g_motor.runtime/g_motor.check。
 */
typedef enum
{
    /** 无故障。 */
    ScrewAxisFault_None = 0U,
    /** 底层 MotorControl 已进入 fault。 */
    ScrewAxisFault_Motor = 1U,
    /** 正向找外端超时。 */
    ScrewAxisFault_HomeSeekOuterTimeout = 2U,
    /** 从外端快速收回找内端超时。 */
    ScrewAxisFault_HomeFastRetractTimeout = 3U,
    /** 低速精找内端超时。 */
    ScrewAxisFault_HomeSlowRetractTimeout = 4U,
} ScrewAxisFault_t;

/**
 * 轴级命令入口。
 *
 * 只保留这一套 Ozone 写入口，避免 home/position 多套命令互相抢优先级。
 * 目标使用 mm_x100 物理位置：0.50 mm 写 50，1.00 mm 写 100。
 */
typedef struct
{
    /** 非 0 时立即停止当前轴命令。 */
    uint8_t stop;
    /** 回零使能；置 1 后开始回零，完成或故障后固件自动清 0。 */
    uint8_t home_enable;
    /** 回零快速收回速度，机械 rpm；写正数也会按负向执行，写 0 时使用默认值。 */
    int16_t home_speed_rpm;
    /** 回零精找收回速度，机械 rpm；写正数也会按负向执行，写 0 时使用默认值。 */
    int16_t home_slow_speed_rpm;
    /** 回零正向退离速度，机械 rpm；写 0 时使用默认值，不影响外端粗找速度。 */
    int16_t home_backoff_speed_rpm;
    /** 回零找端阶段超时，单位 ms；写 0 时使用默认值。 */
    uint16_t home_timeout_ms;
    /** 回零 q 轴电流限幅；写 0 时使用默认值。 */
    int16_t home_iq_limit;
    /** 物理位置模式使能；置 1 后修改 target_mm_x100 会直接更新目标。 */
    uint8_t position_enable;
    /** 正弦位置测试使能；置 1 后自动生成 target_mm_x100。 */
    uint8_t sine_enable;
    /** 目标物理位置，单位 mm_x100；0.50 mm 写 50，10.00 mm 写 1000。 */
    int32_t target_mm_x100;
    /** 正弦中心位置，单位 mm_x100；写 500 表示围绕 5.00 mm 往返。 */
    int32_t sine_center_mm_x100;
    /** 正弦幅值，单位 mm_x100；写 200 表示 +/-2.00 mm。 */
    int32_t sine_amplitude_mm_x100;
    /** 正弦周期，单位 ms；写 2000 表示 2 秒一个完整来回周期。 */
    uint16_t sine_period_ms;
    /** 位置模式速度限幅，机械 rpm；写 0 时使用默认值。 */
    int16_t position_speed_limit_rpm;
    /** 位置模式 q 轴电流限幅；写 0 时使用默认值。 */
    int16_t position_iq_limit;
    /** 位置环 P 增益；写 0 时使用默认值。 */
    int16_t position_kp;
    /** 位置误差右移位数；写 0 时使用默认值。 */
    uint8_t position_err_shift;
    /** 位置到位死区，单位 encoder count；写 0 时使用默认值。 */
    int32_t position_deadband_counts;
    /** 测试轨迹使能；回零完成后置 1 即开始输出 test_mode 轨迹。 */
    uint8_t test_enable;
    /** 测试轨迹类型，取值为 ScrewAxisTestMode_t。 */
    uint8_t test_mode;
    /** 测试轨迹中心位置，单位 mm_x100。 */
    int32_t test_center_mm_x100;
    /** 测试轨迹幅值，单位 mm_x100。 */
    int32_t test_amplitude_mm_x100;
    /** Sine/Step 固定频率，单位 mHz；500 表示 0.5 Hz。 */
    uint16_t test_frequency_mhz;
    /** Sweep 起始频率，单位 mHz。 */
    uint16_t test_sweep_start_frequency_mhz;
    /** Sweep 结束频率，单位 mHz。 */
    uint16_t test_sweep_end_frequency_mhz;
    /** Sweep 单次扫描时长，单位 ms；扫完后保持结束频率，重新使能可重新扫描。 */
    uint32_t test_sweep_duration_ms;
} ScrewAxisCommand_t;

/**
 * 轴级汇总观察入口。
 *
 * watch 是给 Ozone/调试读取的状态快照，不是控制命令入口。
 * pos_mm_x100 / target_mm_x100 / error_mm_x100 是最直观的物理位置数值：
 * 50 表示 0.50 mm，也表示当前丝杠的一圈。
 */
typedef struct
{
    /** 当前轴级状态，取值为 ScrewAxisState_t。 */
    uint8_t state;
    /** 已经找到并记录零位。 */
    uint8_t homed;
    /** 本轮或最近一轮轴动作发生过错误。 */
    uint8_t fault_seen;
    /** 轴级故障原因，取值为 ScrewAxisFault_t。 */
    uint8_t fault_reason;
    /** 正在回零。 */
    uint8_t moving;
    int32_t zero_encoder_pos;
    int32_t outer_encoder_pos;
    int32_t travel_counts;
    int32_t pos_counts;
    int32_t lead_mm_x100;
    int32_t counts_per_rev;
    int32_t nominal_travel_mm_x100;
    int32_t travel_mm_x100;
    int32_t pos_mm_x100;
    int32_t target_mm_x100;
    int32_t error_mm_x100;
    int32_t sine_target_mm_x100;
    uint16_t sine_phase;
    uint8_t sine_active;
    int32_t target_counts;
    int32_t absolute_position_ref;
    uint8_t position_active;
    /** 当前轴层给 MotorControl 的速度命令，机械 rpm。 */
    int16_t speed_cmd_rpm;
    /** 回零详细阶段，取值为 ScrewHomeState_t。 */
    uint8_t home_state;
    /** 当前测试轨迹类型，取值为 ScrewAxisTestMode_t。 */
    uint8_t test_mode;
    /** 正在生成测试轨迹。 */
    uint8_t test_active;
    /** 当前测试轨迹目标，单位 mm_x100。 */
    int32_t test_target_mm_x100;
    /** 当前测试轨迹相位，uint16_t 全周期。 */
    uint16_t test_phase;
    /** 当前测试轨迹瞬时频率，单位 mHz。 */
    uint16_t test_frequency_mhz;
} ScrewAxisWatch_t;

/** 轴级命令入口。 */
extern volatile ScrewAxisCommand_t g_screw_axis_cmd;
/** 轴级汇总观察入口。 */
extern volatile ScrewAxisWatch_t g_screw_axis_watch;

/** 初始化螺杆轴回零状态机并关闭速度命令。 */
void ScrewAxis_Init(void);
/** 主循环调用，推进回零状态机。 */
void ScrewAxis_Run(void);
/** ADC 有效采样后调用，用于给回零状态机提供毫秒时间基。 */
void ScrewAxis_OnAdcSample(void);

#ifdef __cplusplus
}
#endif
