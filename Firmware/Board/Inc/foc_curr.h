/**
 * @file foc_curr.h
 * @brief 三相电流采样、零漂和 PWM/ADC 同步接口。
 */

#pragma once

#include <stdint.h>
#include "Config.h" // IWYU pragma: keep

/** @brief 初始化 ADC、ADCLDO 和三相电流 PGA。 */
void curr_init(void);

/** @brief 执行一次软件触发三相原始 ADC 采样。 */
void curr_sample_raw(void);

/** @brief 在 PWM 关闭状态下采集静态电流零漂。 */
void curr_calib(uint16_t samples);

/** @brief 在 PWM 运行配置下重新采集电流零漂。 */
void curr_calib_pwm(uint16_t samples);

/** @brief 根据最新 raw ADC 值更新三相电流缓存。 */
void curr_update(void);

/** @brief 启动 PWM 触发 ADC 同步采样。 */
void curr_sync_init(void);

/** @brief PWM duty 更新后刷新下一拍采样窗口和触发点。 */
void curr_sync_timing(void);

/** @brief ADC 中断采样入口。 */
uint8_t curr_irq(void);

/** @brief 获取逻辑 U/V/W 三相电流 count。 */
int16_t curr_u(void);
int16_t curr_v(void);
int16_t curr_w(void);
int16_t curr_sum(void);

/** @brief 获取物理采样通道 U/V/W 电流 count。 */
int16_t curr_raw_u(void);
int16_t curr_raw_v(void);
int16_t curr_raw_w(void);
int16_t curr_raw_sum(void);

/** @brief 获取未扣零漂的 U/V/W ADC 原始码值。 */
uint16_t curr_raw_adc_u(void);
uint16_t curr_raw_adc_v(void);
uint16_t curr_raw_adc_w(void);

/** @brief 获取 PWM/ADC 同步采样计数。 */
uint32_t curr_sync_count(void);

/** @brief 获取当前采样策略状态。 */
uint8_t curr_multi_enabled(void);
uint8_t curr_dynamic_enabled(void);
uint8_t curr_pair(void);
uint8_t curr_is_hold(void);
uint16_t curr_hold_count(void);
uint16_t curr_center_tick(void);
uint16_t curr_window_u(void);
uint16_t curr_window_v(void);
uint16_t curr_window_w(void);
uint8_t curr_three_shunt_active(void);
uint16_t curr_window_common(void);
uint32_t curr_sample_switch_count(void);
uint32_t curr_sample_fallback_count(void);
uint16_t curr_sample_pair_hold_left(void);
uint32_t curr_iv_spike_count(void);
uint32_t curr_iw_spike_count(void);
uint16_t curr_iv_max_step(void);
uint16_t curr_iw_max_step(void);

/** @brief 获取双点采样诊断值。 */
uint8_t curr_diag_stage(void);
uint8_t curr_diag_count(void);
int16_t curr_diag_a0(void);
int16_t curr_diag_a1(void);
int16_t curr_diag_b0(void);
int16_t curr_diag_b1(void);
int16_t curr_diag_spread0(void);
int16_t curr_diag_spread1(void);
