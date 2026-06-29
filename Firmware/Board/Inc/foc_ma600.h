/**
 * @file foc_ma600.h
 * @brief MA600 磁编码器 SPI 驱动接口。
 */

#pragma once

#include <stdint.h>

/** @brief 初始化 MA600 使用的 SSP/SPI 外设和引脚。 */
void ma600_init(void);

/** @brief 写 MA600 寄存器。 */
void ma600_write_reg(uint32_t addr, uint32_t value);

/** @brief 读取 MA600 SFR 寄存器。 */
uint32_t ma600_read_reg(uint32_t addr);

/** @brief 兼容旧调试流程的连续角度读取接口。 */
uint32_t ma600_read_data(uint32_t addr);

/** @brief 直接读取一次 MA600 原始角度。 */
uint16_t ma600_read_angle(void);

/** @brief 更新角度缓存。 */
uint8_t ma600_update(void);

/** @brief 使用短超时路径更新角度缓存。 */
uint8_t ma600_update_fast(void);

/** @brief 获取角度缓存。 */
uint16_t ma600_raw(void);
uint8_t ma600_ok(void);
uint8_t ma600_age(void);
