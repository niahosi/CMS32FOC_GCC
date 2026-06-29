/**
 * @file Board_PWM.h
 * @brief 板级 EPWM 输出接口。
 * @details 提供三相互补 PWM 初始化、占空比设置、输出使能和安全状态读取。
 */

#pragma once

#include <stdint.h>
#include "Config.h"

/**
 * @brief 初始化三相 EPWM。
 * @details 配置互补输出、死区、ADC 触发点和输出安全关断状态。
 */
void Board_InitPwm(void);

/**
 * @brief Setter: 设置 EPWM CMP0 的 ADC 触发点。
 * @param[in] tick 触发点计数，超过 PWM 周期时会限制到 PWM 周期。
 * @note 默认值来自 Config.h，调试采样点时可临时通过 Watch 调整。
 */
void Board_SetAdcTriggerTick(uint16_t tick);

/**
 * @brief Getter: 获取当前 ADC 触发点。
 * @return EPWM CMP0 触发点计数。
 */
uint16_t Board_GetAdcTriggerTick(void);

/**
 * @brief Getter: 获取邻域采样较早触发点。
 * @return 双点模式下为中心点加 delta，单点模式下等于中心点。
 */
uint16_t Board_GetAdcTriggerTickA(void);

/**
 * @brief Getter: 获取邻域采样较晚触发点。
 * @return 双点模式下为中心点减 delta，单点模式下等于中心点。
 */
uint16_t Board_GetAdcTriggerTickB(void);

/**
 * @brief Setter: 直接设置两个 ADC 比较触发点。
 * @param[in] tick_a 第一个触发点，下降计数段较早发生的点通常数值更大。
 * @param[in] tick_b 第二个触发点。
 */
void Board_SetAdcTriggerTicks(uint16_t tick_a, uint16_t tick_b);

/**
 * @brief Getter: 获取 PWM 调试状态。
 * @param[out] duty_u U 相占空比计数。
 * @param[out] duty_v V 相占空比计数。
 * @param[out] duty_w W 相占空比计数。
 * @param[out] output_on 1 表示功率输出已打开。
 * @param[out] brake_on 1 表示软件刹车处于打开状态。
 */
void Board_GetPwmDebug(volatile uint16_t* duty_u, volatile uint16_t* duty_v,
                       volatile uint16_t* duty_w, volatile uint8_t* output_on,
                       volatile uint8_t* brake_on);

/**
 * @brief Getter: 判断 PWM 是否处于安全关闭状态。
 * @return 1 表示刹车打开且输出关闭，0 表示不满足安全关闭条件。
 */
uint8_t Board_IsPwmOffSafe(void);

/**
 * @brief Getter: 判断 PWM 是否处于运行输出状态。
 * @return 1 表示刹车关闭且输出打开，0 表示未处于运行输出状态。
 */
uint8_t Board_IsPwmRunOk(void);

/**
 * @brief Getter: 判断 PWM 当前是否安全。
 * @return 1 表示 PWM 安全，0 表示 PWM 状态不安全。
 */
uint8_t Board_IsPwmSafe(void);

/**
 * @brief Setter: 设置三相 PWM 占空比。
 * @param[in] duty_u U 相占空比计数。
 * @param[in] duty_v V 相占空比计数。
 * @param[in] duty_w W 相占空比计数。
 */
void Board_SetPwmDuty(uint16_t duty_u, uint16_t duty_v, uint16_t duty_w);

/**
 * @brief 强制关闭 PWM 功率输出。
 * @details 关闭驱动使能、打开软件刹车、关闭 EPWM 输出并回到 50% 占空比。
 */
void Board_ForcePwmOff(void);

/**
 * @brief Setter: 设置 PWM 功率输出使能。
 * @param[in] enable 1 表示打开输出，0 表示强制关闭输出。
 * @return 1 表示输出已打开，0 表示输出已关闭。
 */
uint8_t Board_EnablePwmOutput(uint8_t enable);
