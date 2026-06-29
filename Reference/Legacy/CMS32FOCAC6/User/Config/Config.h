/**
 * @file Config.h
 * @brief FOCAC6 项目可调参数入口。
 * @details 这里只集中电机、传感器、控制环、保护阈值和开环默认值。
 *          引脚复用、ADC 通道、EPWM 通道、寄存器位等底层实现细节仍留在 Board/Motor/App 模块内。
 */

#pragma once

/* Board PWM --------------------------------------------------------------- */

/** @brief PWM 目标频率，单位 Hz。 */
#define PWM_FREQ_HZ 20000U
/** @brief 中心对齐 PWM 周期，Period = 64000000 / (2 * 20000) = 1600。 */
#define PWM_PERIOD 1600U
/** @brief 50% 占空比计数。 */
#define PWM_DUTY_50 800U
/** @brief 调试阶段最小占空比限制。 */
#define PWM_DUTY_MIN 200U
/** @brief 调试阶段最大占空比限制。 */
#define PWM_DUTY_MAX 1400U
/** @brief 死区时间 tick 数，当前约 1 us。 */
#define PWM_DEADTIME_TICKS 64U
/** @brief EPWM CMP0 触发 ADC 的默认计数点。650 */
#define PWM_ADC_TRIGGER_TICK_DEFAULT 650

/* Board analog ------------------------------------------------------------ */

/** @brief ADC 参考电压，单位 V。 */
#define ADC_VREF_V 3.6f
/** @brief ADC 满量程计数。 */
#define ADC_COUNTS 4096.0f
/** @brief 采样电阻阻值，单位 ohm。 */
#define SHUNT_OHM 0.08f
/** @brief PGA 增益。 */
#define PGA_GAIN 2.0f
/** @brief ADC count 到电流 A 的理论换算系数。 */
#define ADC_TO_AMP (ADC_VREF_V / ADC_COUNTS / SHUNT_OHM / PGA_GAIN)
/** @brief 静态零漂校准默认采样次数。 */
#define BOARD_CURRENT_OFFSET_SAMPLES 1024U
/** @brief 运行时 CMP0 同步采样 U/V 两相，W 相由 -U-V 重构。 */
#define CURRENT_SAMPLE_UV 0U
/** @brief 运行时 CMP0 同步采样 U/W 两相，V 相由 -U-W 重构。 */
#define CURRENT_SAMPLE_UW 1U
/** @brief 运行时 CMP0 同步采样 V/W 两相，U 相由 -V-W 重构。 */
#define CURRENT_SAMPLE_VW 2U
/** @brief 测试模式：运行时 CMP0 顺序采样 U/V/W 三相，不重构第三相。 */
#define CURRENT_SAMPLE_UVW_DIAG 3U
/** @brief 固定邻域双点采样开关，0 保持当前单点行为。 */
#define CURRENT_SAMPLE_MULTI_ENABLE 1U
/** @brief 动态选择低边窗口最大的两相采样，第三相由 KCL 重构。 */
#define CURRENT_SAMPLE_DYNAMIC_ENABLE 1U
/** @brief 双点采样与中心点的偏移 tick。 */
#define CURRENT_SAMPLE_MULTI_DELTA_TICK 40U
/** @brief 双点采样两点差值超过该值时，不直接取均值。 */
#define CURRENT_SAMPLE_MULTI_SPREAD_LIMIT_CNT 40
/** @brief 单相低边窗口小于该值时，不参与动态两相选择。 */
#define CURRENT_SAMPLE_MIN_WINDOW_TICK (PWM_DEADTIME_TICKS + 32U)
/** @brief 运行时电流采样相选择，当前使用VW。 */
#define CURRENT_SAMPLE_PAIR CURRENT_SAMPLE_VW

/* Motor and sensor -------------------------------------------------------- */

#define MOTOR_POLE_PAIRS 4u
#define MOTOR_SENSOR_POLE_PAIRS 4u
#define MOTOR_SENSOR_COUNTS_PER_REV 65536ul
#define MOTOR_POS_COUNTS_PER_MOTOR_REV (MOTOR_SENSOR_COUNTS_PER_REV * MOTOR_SENSOR_POLE_PAIRS)

/** @brief 传感器电角度倍数。 */
#define MOTOR_SENSOR_ELEC 1u
/** @brief 传感器方向，-1 表示从电机背后看顺时针转动时角度递减。 */
#define MOTOR_SENSOR_DIR (-1)
/** @brief 电角度零点偏移。 */
#define MOTOR_ELEC_ZERO 9577u //0u //38778u

/* Motor safety and alignment --------------------------------------------- */

/** @brief 电流采样值检查限值（ADC count）。 */
#define MOTOR_CHECK_CURRENT_CNT_LIMIT 32767
/** @brief 三相电流和检查限值（ADC count）。 */
#define MOTOR_CHECK_SUM_CNT_LIMIT 32767
/** @brief MA600 传感器检查采样次数。 */
#define MOTOR_CHECK_MA600_SAMPLES 10u
/** @brief MA600 角度缓存允许的最大年龄。 */
#define MOTOR_ANGLE_MAX_AGE 4u

/** @brief 闭环前对齐 d 轴电压幅值。 */
#define MOTOR_ALIGN_VD 120
/** @brief 闭环前对齐目标电角度。 */
#define MOTOR_ALIGN_THETA 0u
/** @brief 闭环前对齐持续 tick 数（约 300ms @2kHz）。 */
#define MOTOR_ALIGN_TICKS 600u
/** @brief PWM 运行时零漂重校准采样次数。 */
#define MOTOR_ALIGN_CAL_SAMPLES 128u

/* Current loop ------------------------------------------------------------ */

/** @brief 电流环给定限幅。 */
#define MOTOR_CURRENT_REF_LIMIT 500
/** @brief 电流环输出电压限幅。 */
#define MOTOR_CURRENT_V_LIMIT 500
/** @brief 电流环 PI 定点右移位数。 */
#define MOTOR_CURRENT_PI_SHIFT 3u
/** @brief 电流环比例系数。 */
#define MOTOR_CURRENT_KP 4
/** @brief 电流环积分系数。 */
#define MOTOR_CURRENT_KI 0
/** @brief 运行电流保护阈值。 */
#define MOTOR_CURRENT_SAFE_LIMIT 400
/** @brief 连续过流计数阈值。 */
#define MOTOR_CURRENT_OVER_LIMIT 4u
/** @brief 快环分频，20 kHz ADC/PWM sync / 10 = about 2 kHz current loop. */
#define MOTOR_FAST_LOOP_DIV 2u

/* Speed loop -------------------------------------------------------------- */

/** @brief 速度环分频，2 kHz current loop / 4 = about 500 Hz speed loop. */
#define MOTOR_SPEED_LOOP_DIV 4u
/** @brief 速度估算频率。 */
#define MOTOR_SPEED_EST_HZ 500l
/** @brief 速度估算位置死区。 */
#define MOTOR_SPEED_POS_DEADBAND 16l
/** @brief 速度反馈归零吸附阈值。 */
#define MOTOR_SPEED_ZERO_SNAP 200l
/** @brief 速度环比例系数。 */
#define MOTOR_SPEED_KP 1
/** @brief 速度环积分系数。 */
#define MOTOR_SPEED_KI 0
/** @brief 速度误差缩放右移位数。 */
#define MOTOR_SPEED_ERR_SHIFT 10u
/** @brief 速度反馈滤波右移位数。 */
#define MOTOR_SPEED_FILTER_SHIFT 4u
/** @brief 速度给定限幅。 */
#define MOTOR_SPEED_REF_LIMIT 200000l
/** @brief 速度环默认 iq 限幅。 */
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT 20

/* Open loop VF/IF --------------------------------------------------------- */

/** @brief VF/IF 开环调试默认速度给定，单位 sensor counts/s，约 36 rpm 机械。 */
#define MOTOR_OL_SPEED_REF_DEFAULT 400l
/** @brief VF 开环默认电压幅值。 */
#define MOTOR_VF_VOLTAGE_DEFAULT 160
/** @brief IF 开环默认 q 轴电流给定。 */
#define MOTOR_IF_IQ_REF_DEFAULT 80
/** @brief IF 开环默认 d 轴电流给定。 */
#define MOTOR_IF_ID_REF_DEFAULT 0
/** @brief VF/IF 开环默认超时时间，单位 ms。 */
#define MOTOR_OL_TIMEOUT_MS_DEFAULT 30000u
/** @brief 开环速度到电角度步进的定点乘数。 */
#define MOTOR_OL_SPEED_TO_THETA_STEP 131l
/** @brief 开环速度到电角度步进的定点右移位数。 */
#define MOTOR_OL_SPEED_TO_THETA_SHIFT 8u
