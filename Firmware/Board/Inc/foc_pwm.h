/**
 * @file foc_pwm.h
 * @brief 三相 EPWM 输出和功率级安全态接口。
 */

#pragma once

#include <stdint.h>
#include "Config.h" // IWYU pragma: keep

/** @brief 初始化三相互补 PWM，并保持功率输出关闭。 */
void pwm_init(void);

/** @brief 设置三相 PWM 占空比计数。 */
void pwm_set_duty(uint16_t u, uint16_t v, uint16_t w);

/** @brief 设置单点 ADC 触发 tick。 */
void pwm_set_adc_trigger(uint16_t tick);

/** @brief 设置双点 ADC 触发 tick。 */
void pwm_set_adc_triggers(uint16_t a, uint16_t b);

/** @brief 获取 ADC 触发中心 tick。 */
uint16_t pwm_adc_trigger(void);

/** @brief 获取双点采样 A 触发 tick。 */
uint16_t pwm_adc_trigger_a(void);

/** @brief 获取双点采样 B 触发 tick。 */
uint16_t pwm_adc_trigger_b(void);

/** @brief 强制关闭功率级输出。 */
void pwm_off(void);

/** @brief 设置功率级输出使能，返回实际输出状态。 */
uint8_t pwm_enable(uint8_t en);

/** @brief 判断 PWM 是否处于安全关闭状态。 */
uint8_t pwm_is_off_safe(void);

/** @brief 判断 PWM 是否处于运行输出状态。 */
uint8_t pwm_is_running(void);

/** @brief 判断 PWM 当前是否安全。 */
uint8_t pwm_is_safe(void);

/**
 * @brief 获取 PWM 调试快照。
 *
 * 参数使用 volatile 指针，方便诊断固件直接写入 Watch 变量。
 */
void pwm_snapshot(volatile uint16_t* u, volatile uint16_t* v, volatile uint16_t* w,
                  volatile uint8_t* out, volatile uint8_t* brake);
