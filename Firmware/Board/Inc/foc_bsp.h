/**
 * @file foc_bsp.h
 * @brief FOC 控制板 BSP 总入口。
 */

#pragma once

#include <stdint.h>

/**
 * @brief 初始化板级硬件。
 *
 * 初始化顺序保持硬件 bring-up 已验证逻辑：时钟、P16 安全态、MA600、
 * ADC/PGA、PWM、静态电流零漂和 PWM 触发 ADC 同步采样。
 */
void bsp_init(void);

/**
 * @brief 启动 PWM 触发 ADC 同步采样和 ADC 中断。
 *
 * 必须在 MotorControl_Init() 之后调用，避免 ADC IRQ 在控制对象初始化前进入快环。
 */
void bsp_start_adc_sync(void);

/**
 * @brief ADC 中断中的板级快环入口。
 * @return 1 表示完成一组有效电流采样，0 表示本次中断未形成有效样本。
 */
uint8_t bsp_adc_irq(void);

/**
 * @brief 更新 MA600 角度缓存。
 * @return 1 表示更新成功，0 表示 SPI 超时或读取失败。
 */
uint8_t bsp_update_angle(void);

/**
 * @brief 使用短超时路径更新 MA600 角度缓存。
 * @return 1 表示更新成功，0 表示 SPI 超时或读取失败。
 */
uint8_t bsp_update_angle_fast(void);

/**
 * @brief 使用 32-bit 帧更新 MA600 角度和 speed 缓存。
 * @return 1 表示更新成功，0 表示 SPI 超时或读取失败。
 */
uint8_t bsp_update_angle_speed_fast(void);

/** @brief 获取最近一次缓存的 MA600 原始角度。 */
uint16_t bsp_angle_raw(void);

/** @brief 获取最近一次缓存的 MA600 speed 原始 signed 16-bit 输出。 */
int16_t bsp_angle_speed_raw(void);

/** @brief 判断最近一次 MA600 缓存更新是否成功。 */
uint8_t bsp_angle_ok(void);

/** @brief 获取 MA600 缓存年龄。 */
uint8_t bsp_angle_age(void);

/** @brief 直接读取一次 MA600 原始角度。 */
uint16_t bsp_read_angle(void);
