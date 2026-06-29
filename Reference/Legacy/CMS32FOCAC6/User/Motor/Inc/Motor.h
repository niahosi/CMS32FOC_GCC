/**
 * @file Motor.h
 * @brief 电机控制模块接口
 * @details 提供电机状态机、闭环 FOC 控制和故障保护等功能。
 *          此模块负责管理电机的整个运行生命周期，包括：
 *          - 传感器检查和安全验证
 *          - 闭环 FOC 控制骨架
 *          - 故障保护和状态监控
 */

#pragma once
#include <stdint.h>
#include "Config.h"

/**
 * @brief 电机状态枚举
 * @details 定义电机运行状态机的各个状态
 */
typedef enum
{
    MOTOR_STATE_IDLE = 0,     /**< 空闲状态，不输出 */
    MOTOR_STATE_SENSOR_CHECK, /**< 传感器检查状态 */
    MOTOR_STATE_ALIGN,        /**< 转子对齐状态 */
    MOTOR_STATE_CLOSED_LOOP,  /**< 闭环 FOC 运行状态 */
    MOTOR_STATE_FAULT         /**< 故障锁定状态 */
} MotorState_t;

typedef enum
{
    MOTOR_CTRL_OFF = 0,     /**< 关闭控制输出 */
    MOTOR_CTRL_CURRENT = 1, /**< 电流环模式 */
    MOTOR_CTRL_SPEED = 2,   /**< 速度环模式 */
    MOTOR_CTRL_VF = 3,      /**< VF 开环模式 */
    MOTOR_CTRL_IF = 4       /**< IF 开环模式 */
} MotorControlMode_t;

typedef enum
{
    MOTOR_FAULT_NONE = 0,      /**< 无故障 */
    MOTOR_FAULT_MA600_CHECK,   /**< MA600 检查失败 */
    MOTOR_FAULT_CURRENT_CHECK, /**< 电流零点/静态检查失败 */
    MOTOR_FAULT_ANGLE_STALE,   /**< 角度缓存过期 */
    MOTOR_FAULT_RUN_CURRENT,   /**< 运行中过流 */
    MOTOR_FAULT_STATE          /**< 状态机异常 */
} MotorFaultReason_t;

/**
 * @brief 电机检查结果结构体
 * @details 存储传感器和安全检查的结果
 */
typedef struct
{
    uint8_t ma600_ok;
    uint8_t current_ok;
    uint8_t pwm_off_safe;
    uint8_t ready_closed_loop;
} MotorCheck_t;

/**
 * @brief 精简运行快照结构体
 * @details 只保留高频调试最常看的字段，用于 Watch 窗口快速观测。
 */
typedef struct
{
    MotorState_t state;
    MotorControlMode_t ctrl_mode;
    uint8_t enable;
    uint16_t angle_raw;
    uint16_t angle_elec;
    int32_t angle_pos;
    int16_t angle_delta;
    int32_t speed_ref;
    int32_t speed_fb;
    int16_t id_ref;
    int16_t iq_ref;
    int16_t id;
    int16_t iq;
    int16_t vd;
    int16_t vq;
    int16_t kp;
    int16_t ki;
    int16_t v_limit;
    uint8_t v_limited;
    uint16_t duty_u;
    uint16_t duty_v;
    uint16_t duty_w;
    uint8_t pwm_output_on;
    uint8_t pwm_brake_on;
    MotorFaultReason_t fault_reason;
    uint8_t current_over_count;
} MotorRunSnap_t;

/**
 * @brief 初始化电机控制模块。
 * @details 清空状态机、角度缓存、诊断模块和闭环模块，上电后调用一次。
 */
void Motor_Init(void);

/**
 * @brief 电机低频任务入口。
 * @details 在主循环中调用，负责状态机、安全检查、对齐和输出门控。
 */
void Motor_TASK(void);

/**
 * @brief 电机快环入口。
 * @details 在 ADC/PWM 同步采样后调用，按分频执行电流环、速度环或开环控制。
 * @note 中断路径内禁止阻塞、打印和动态内存。
 */
void Motor_FastLoop(void);

/**
 * @brief 从中断上下文更新电机角度缓存。
 * @details 使用 Board 层已经更新的 MA600 缓存，计算原始角度、电角度和累计位置。
 */
void Motor_UpdateAngleFromIsr(void);

/**
 * @brief Setter: 设置电机总使能。
 * @param[in] enable 1 表示请求运行，0 表示关断并回到空闲。
 */
void Motor_SetEnable(uint8_t enable);

/**
 * @brief Setter: 设置控制模式。
 * @param[in] mode 控制模式，超出范围会被限制为 MOTOR_CTRL_OFF。
 */
void Motor_SetControlMode(MotorControlMode_t mode);

/**
 * @brief Setter: 设置 d/q 轴电流给定。
 * @param[in] id_ref d 轴电流给定，单位为 ADC count。
 * @param[in] iq_ref q 轴电流给定，单位为 ADC count。
 */
void Motor_SetCurrentRef(int16_t id_ref, int16_t iq_ref);

/**
 * @brief Setter: 设置电流环 PI 参数。
 * @param[in] kp 比例系数。
 * @param[in] ki 积分系数。
 * @param[in] v_limit d/q 电压输出限幅，单位 PWM count。
 */
void Motor_SetCurrentPi(int16_t kp, int16_t ki, int16_t v_limit);

/**
 * @brief Setter: 设置速度环目标速度。
 * @param[in] speed_ref 速度给定，单位为 sensor counts/s。
 */
void Motor_SetSpeedRef(int32_t speed_ref);

/**
 * @brief Setter: 设置速度环输出 iq 限幅。
 * @param[in] iq_limit iq 限幅正值，单位为 ADC count。
 */
void Motor_SetIqLimit(int16_t iq_limit);

/**
 * @brief Setter: 设置开环目标速度。
 * @param[in] speed_ref 开环速度给定，单位为 sensor counts/s。
 */
void Motor_SetOlSpeedRef(int32_t speed_ref);

/**
 * @brief Setter: 设置 VF 开环电压幅值。
 * @param[in] voltage VF 电压幅值，单位为 PWM count。
 */
void Motor_SetVfVoltage(int16_t voltage);

/**
 * @brief Setter: 设置 IF 开环 d/q 轴电流给定。
 * @param[in] id_ref d 轴电流给定，单位为 ADC count。
 * @param[in] iq_ref q 轴电流给定，单位为 ADC count。
 */
void Motor_SetIfCurrentRef(int16_t id_ref, int16_t iq_ref);

/**
 * @brief Setter: 设置 VF/IF 开环超时时间。
 * @param[in] timeout_ms 超时时间，单位 ms。
 */
void Motor_SetOlTimeoutMs(uint16_t timeout_ms);

/**
 * @brief Getter: 获取当前缓存的 MA600 原始角度。
 * @return 原始角度 count，范围 0~65535。
 */
uint16_t Motor_GetAngleRaw(void);

/**
 * @brief Getter: 获取当前 FOC 电角度。
 * @return 电角度 count，范围 0~65535。
 */
uint16_t Motor_GetAngleElec(void);

/**
 * @brief Getter: 获取最近两次角度差值。
 * @return 原始角度增量，已按 int16_t 自动处理跨零。
 */
int16_t Motor_GetAngleDelta(void);

/**
 * @brief Getter: 获取解缠后的累计位置。
 * @return 累计位置 count。
 */
int32_t Motor_GetAnglePos(void);

/**
 * @brief Getter: 判断角度缓存是否可用于闭环。
 * @return 1 表示角度有效，0 表示角度不可用。
 */
uint8_t Motor_IsAngleSafe(void);

/**
 * @brief Getter: 获取角度缓存年龄。
 * @return 距离最近一次成功更新的失败/老化计数。
 */
uint8_t Motor_GetAngleAge(void);

/**
 * @brief Getter: 获取电机进入闭环前的安全检查结果。
 * @param[out] check 检查结果输出指针，传入 0 时函数直接返回。
 */
void Motor_GetCheck(MotorCheck_t* check);

/**
 * @brief Getter: 获取电机运行快照。
 * @param[out] snap 运行快照输出指针，传入 0 时函数直接返回。
 */
void Motor_GetRunSnap(MotorRunSnap_t* snap);

/**
 * @brief 根据传感器原始角度计算 FOC 电角度。
 * @param[in] sensor_angle MA600 原始角度 count。
 * @return 修正方向和零点后的电角度 count。
 */
uint16_t Motor_GetElecAngle(uint16_t sensor_angle);

/**
 * @brief Setter: 设置电角度零点。
 * @param[in] zero 电角度零点偏移 count。
 */
void Motor_SetElecZero(uint16_t zero);

/**
 * @brief Getter: 获取当前电角度零点。
 * @return 电角度零点偏移 count。
 */
uint16_t Motor_GetElecZero(void);
