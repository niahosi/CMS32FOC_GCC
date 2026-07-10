/**
 * @file foc_curr.h
 * @brief 三相电流采样、零漂和 PWM/ADC 同步接口。
 */

#pragma once

#include <stdint.h>
#include "Config.h" // IWYU pragma: keep

/** @brief 初始化 ADC、ADCLDO 和三相电流 PGA。 */
void curr_init(void);

/**
 * @brief 在 PWM 关闭状态下采集静态电流零漂。
 * @param samples 每相平均采样次数。
 */
void curr_calib(uint16_t samples);

/** @brief 启动 PWM 触发 ADC 同步采样。 */
void curr_sync_init(void);

/** @brief PWM duty 更新后刷新下一拍采样窗口和触发点。 */
void curr_sync_timing(void);

/**
 * @brief 传入当前 VF 电压命令，用于高调制区切换采样策略。
 * @param vf_voltage VF q 轴电压命令，SVPWM count 单位。
 */
void curr_set_vf_voltage(int16_t vf_voltage);

/**
 * @brief ADC 中断采样入口。
 * @return 1 表示双点采样已解析出有效三相电流，0 表示仍在等待或样本无效。
 */
uint8_t curr_irq(void);

/** @brief 获取逻辑 U/V/W 三相电流 count。 */
int16_t curr_u(void);
int16_t curr_v(void);
int16_t curr_w(void);
int16_t curr_sum(void);

/** @brief 获取未扣零漂的 U/V/W ADC 原始码值。 */
uint16_t curr_raw_adc_u(void);
uint16_t curr_raw_adc_v(void);
uint16_t curr_raw_adc_w(void);

/** @brief 获取 PWM/ADC 同步采样计数；每次有效解析后递增。 */
uint32_t curr_sync_count(void);
