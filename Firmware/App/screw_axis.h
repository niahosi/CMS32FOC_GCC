#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 回零状态。数值会暴露给 Ozone/watch，调整时要同步上位观察脚本。 */
typedef enum
{
    /** 未运行，等待 start_seq 变化。 */
    ScrewHomeState_Idle = 0U,
    /** 快速负向回退，寻找机械端点/堵转点。 */
    ScrewHomeState_FastRetract = 1U,
    /** 正向退离端点，释放机械压紧。 */
    ScrewHomeState_FastBackoff = 2U,
    /** 低速负向精找端点。 */
    ScrewHomeState_SlowRetract = 3U,
    /** 记录零位后最终正向退离。 */
    ScrewHomeState_FinalBackoff = 4U,
    /** 回零完成，电机停止，零位有效。 */
    ScrewHomeState_Done = 5U,
    /** 回零失败或底层电机控制报错。 */
    ScrewHomeState_Fault = 6U,
} ScrewHomeState_t;

/**
 * Ozone/上层应用写入的回零命令。
 *
 * 修改参数后递增 start_seq 触发新一轮回零。负向回退速度会在内部归一化：
 * speed_rpm/slow_speed_rpm 写正数也会按负向执行。
 */
typedef struct
{
    /** 启动序号；每次递增触发一次新回零。 */
    uint16_t start_seq;
    /** 非 0 时立即停止回零并关闭速度命令。 */
    uint8_t stop;
    /** 快速回退速度，机械 rpm，内部强制为负向。 */
    int16_t speed_rpm;
    /** 精找回退速度，机械 rpm，内部强制为负向。 */
    int16_t slow_speed_rpm;
    /** 离开端点的正向退离速度，机械 rpm。 */
    int16_t backoff_speed_rpm;
    /** 快速/低速找端点阶段超时，单位 ms。 */
    uint16_t timeout_ms;
    /** 回零期间 q 轴电流限幅，沿用 MotorControl 缩放单位。 */
    int16_t iq_limit;
} ScrewHomeCommand_t;

/** 回零流程状态快照，只保留上层长期需要看的稳定字段。 */
typedef struct
{
    /** 回零流程正在运行。 */
    uint8_t busy;
    /** 已经找到并记录零位。 */
    uint8_t homed;
    /** 本轮或最近一轮回零发生过错误。 */
    uint8_t fault_seen;
    /** 当前状态，取值为 ScrewHomeState_t。 */
    uint8_t state;
    /** 当前执行到的 start_seq。 */
    uint16_t active_seq;
    /** 记录零位时的编码器原始计数。 */
    int32_t zero_encoder_pos;
    /** 当前位置相对零位的编码器计数；未回零时为 0。 */
    int32_t pos_counts;
} ScrewHomeStatus_t;

typedef ScrewHomeStatus_t ScrewHomeWatch_t;

/** 回零命令入口，当前主固件唯一的 ScrewAxis command 全局变量。 */
extern volatile ScrewHomeCommand_t g_screw_home_cmd;
/** 回零观察入口，当前主固件唯一的 ScrewAxis watch 全局变量。 */
extern volatile ScrewHomeWatch_t g_screw_home_watch;

/** 初始化螺杆轴回零状态机并关闭速度命令。 */
void ScrewAxis_Init(void);
/** 主循环调用，推进回零状态机。 */
void ScrewAxis_Run(void);
/** ADC 有效采样后调用，用于给回零状态机提供毫秒时间基。 */
void ScrewAxis_OnAdcSample(void);

#ifdef __cplusplus
}
#endif
