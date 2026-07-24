#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ozone/主循环写入的电机控制命令。
 *
 * 该结构由主循环复制到控制层内部缓存，ADC 中断快环只读取缓存副本。
 * Current/Speed/Position 主线只依赖闭环相关字段；VF 字段保留为应急开环入口。
 */
typedef struct
{
    /** @brief 非 0 允许控制状态机进入指定控制模式。 */
    uint8_t enable;
    /** @brief 控制模式：1 Current, 2 Speed, 3 VF, 6 Position；4/5 已冻结为不支持。 */
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
    /** @brief 位置目标，单位为编码器累计 count，对应 g_motor.encoder.pos。 */
    int32_t position_ref;
    /** @brief 位置环比例系数，输入为 encoder count 误差，输出为 mechanical rpm。 */
    int16_t position_kp;
    /** @brief 位置误差缩放右移位数。 */
    uint8_t position_err_shift;
    /** @brief 位置模式速度限幅，mechanical rpm。 */
    int16_t position_speed_limit_rpm;
    /** @brief 位置误差小于该 count 时输出 0 rpm。 */
    int32_t position_deadband_counts;
    /** @brief 电流环输出电压限幅，SVPWM count 单位。 */
    int16_t current_v_limit;
    /** @brief VF 应急开环速度命令，内部开环角步进单位。 */
    int32_t open_loop_speed_ref;
    /** @brief VF 应急开环 q 轴电压，SVPWM count 单位。 */
    int16_t vf_voltage;
    /** @brief 冻结字段：预留 IF 诊断 d 轴给定，主固件不使用。 */
    int16_t if_id_ref;
    /** @brief 冻结字段：预留 IF 诊断 q 轴给定，主固件不使用。 */
    int16_t if_iq_ref;
    /** @brief VF 开环超时时间，ms；0 表示不超时。 */
    uint16_t open_loop_timeout_ms;
    /** @brief 电角度零位临时修正，叠加到 MOT_ELEC_ZERO。 */
    int16_t elec_zero_trim;
    /** @brief Current/Speed 输出电压角度偏置，用于相位提前/滞后调试。 */
    int16_t voltage_theta_offset;
} MotorControlCommand_t;

/**
 * @brief Ozone/主循环写入的电机控制命令入口。
 *
 * 该变量是当前主固件公共调试入口，定义在 C 控制层实现中，不放在 main.c。
 */
extern volatile MotorControlCommand_t g_mc_cmd;

/** @brief 初始化 C 控制层状态和 PI 参数，保持 PWM 关闭。 */
void MotorControl_Init(void);
/** @brief 从 volatile 命令入口复制并限幅命令，只在主循环调用。 */
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t *command);
/** @brief 慢环安全检查和状态机更新，只在主循环调用。 */
void MotorControl_RunSlowLoop(void);
/** @brief ADC IRQ 快环入口；返回 1 表示本次形成有效控制采样。 */
uint8_t MotorControl_FastLoopFromAdcIrq(void);
#ifdef __cplusplus
}
#endif
