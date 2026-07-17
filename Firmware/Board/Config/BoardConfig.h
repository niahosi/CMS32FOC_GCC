/**
 * @file BoardConfig.h
 * @brief 板级和电机硬件基线配置。
 * @details 这里放不常变的 PWM 频率/周期、ADC/PGA 换算、电机和传感器基础参数。
 */

#pragma once

/* Board PWM --------------------------------------------------------------- */

/** @brief PWM 目标频率，单位 Hz。 */
#define PWM_FREQ_HZ 20000U
/** @brief 中心对齐 PWM 周期，Period = 64000000 / (2 * 20000) = 1600。 */
#define PWM_PERIOD 1600U
/** @brief 50% 占空比计数。 */
#define PWM_DUTY_50 800U
/** @brief 高调制测试阶段最小占空比限制，先保留约一个死区 tick 的极窄脉冲余量。 */
#define PWM_DUTY_MIN 32U
/** @brief 高调制测试阶段最大占空比限制。 */
#define PWM_DUTY_MAX 1568U
/** @brief 当前对称 duty guard 下 SVPWM 线性电压幅值上限，约等于 (800 - guard) / 0.866。 */
#define PWM_SVPWM_V_LIMIT (((PWM_DUTY_50 - PWM_DUTY_MIN) * 1000U) / 866U)
/** @brief EPWM CMP0 触发 ADC 的默认计数点。 */
#define PWM_ADC_TRIGGER_TICK_DEFAULT 650U
/** @brief 死区时间 tick 数，当前约 0.5 us。 */
#define PWM_DEADTIME_TICKS 32U

/* Phase mapping ----------------------------------------------------------- */

#define MOT_PHASE_MAP_UVW 0U
#define MOT_PHASE_MAP_UWV 1U
#define MOT_PHASE_MAP_VUW 2U
#define MOT_PHASE_MAP_VWU 3U
#define MOT_PHASE_MAP_WUV 4U
#define MOT_PHASE_MAP_WVU 5U

/** @brief 控制算法 U/V/W 到 EPWM 物理 U/V/W 的相序映射。 */
#define MOT_PWM_PHASE_MAP MOT_PHASE_MAP_UVW
/** @brief 物理采样 U/V/W 到控制算法 U/V/W 的相序映射。 */
#define MOT_CURR_PHASE_MAP MOT_PHASE_MAP_UVW
/** @brief 电流采样符号，1 保持，-1 整体反向。 */
#define MOT_CURR_SIGN (-1)

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

/* Motor and sensor -------------------------------------------------------- */

#define MOT_POLE_PAIRS 4u
#define MOT_SENSOR_POLE_PAIRS 4u
#define MOT_SENSOR_CPR 65536ul
#define MOT_POS_COUNTS_PER_REV (MOT_SENSOR_CPR * MOT_SENSOR_POLE_PAIRS)

/** @brief MA600 磁环角到电机电角度的倍数：转子极对数 / 磁环极对数。 */
#define MOT_SENSOR_ELEC (MOT_POLE_PAIRS / MOT_SENSOR_POLE_PAIRS)
/** @brief 传感器方向，使编码器电角度与控制电角度正方向一致。 */
// #define MOT_SENSOR_DIR (1)
#define MOT_SENSOR_DIR (1)
/** @brief 电角度零点偏移。 */
#define MOT_ELEC_ZERO -13478 //-24000//9577u //0u //38778u

/* MA600 SPI ---------------------------------------------------------------- */

/** @brief MA600 SSP 分频 M，SSPCLK = PCLK / ((M + 1) * N)。 */
#define MA600_SSP_CLK_M 7U
/** @brief MA600 SSP 分频 N；当前 64 MHz / ((7 + 1) * 2) = 4 MHz。 */
#define MA600_SSP_CLK_N 2U

/* Board UART --------------------------------------------------------------- */

/** @brief 开发期 UART bring-up 开关；打开后会在延时窗口后复用 P06/P07。 */
#define BOARD_UART_ENABLE 1U
/** @brief 开发期 UART 波特率；先用 9600 验证链路稳定性，再逐步提高。 */
#define BOARD_UART_BAUD 2000000U // 921600U
/** @brief 上电后保留 SWD 连接窗口，再禁用 SWD 并把 P06/P07 切成 UART。 */
#define BOARD_UART_SW_RELEASE_DELAY_MS 20000U
/** @brief P06/RXD 输入电平模式：1=TTL 输入，0=施密特输入；当前用施密特抗边沿噪声。 */
#define BOARD_UART_RX_TTL_INPUT 0U
/** @brief UART END 访问结束写入；访问 UART 寄存器后再碰其他外设前必须写 END。 */
#define BOARD_UART_USE_END_LOCK 1U
/** @brief UART 调试期优先保证 RX 及时响应；数值越小优先级越高。 */
#define BOARD_UART_IRQ_PRIORITY 0U
/** @brief UART 调试期暂时降低 ADC 快环优先级，避免单字节 RX buffer 溢出。 */
#define BOARD_UART_ADC_IRQ_PRIORITY 1U
