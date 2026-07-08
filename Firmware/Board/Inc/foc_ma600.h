/**
 * @file foc_ma600.h
 * @brief MA600 磁编码器 SPI 驱动接口。
 */

#pragma once

#include <stdint.h>

/** @brief 初始化 MA600 使用的 SSP/SPI 外设和引脚。 */
void ma600_init(void);

/** @brief 写 MA600 RAM 寄存器；不用于 ADC 中断快环。 */
void ma600_write_reg(uint32_t addr, uint32_t value);

/** @brief 读取 MA600 SFR 寄存器。 */
uint32_t ma600_read_reg(uint32_t addr);

/**
 * @brief 触发 MA600 NVM block store，供诊断模块手动调用。
 * @warning NVM 写入有寿命和误配置风险，默认不要在调速过程中触发。
 */
void ma600_store_nvm_block(uint8_t block);

/** @brief 兼容旧调试流程的连续角度读取接口。 */
uint32_t ma600_read_data(uint32_t addr);

/** @brief 直接读取一次 MA600 原始角度。 */
uint16_t ma600_read_angle(void);

/** @brief 更新角度缓存。 */
uint8_t ma600_update(void);

/** @brief 使用短超时路径更新角度缓存。 */
uint8_t ma600_update_fast(void);

/** @brief 使用 32-bit 帧同时更新角度和 MA600 speed 缓存。 */
uint8_t ma600_update_speed_fast(void);

/** @brief 获取角度缓存。 */
uint16_t ma600_raw(void);
/** @brief 获取 MA600 speed 原始 signed 16-bit 输出。 */
int16_t ma600_speed_raw(void);
/** @brief 判断最近一次角度缓存更新是否成功。 */
uint8_t ma600_ok(void);
/** @brief 获取角度缓存年龄。 */
uint8_t ma600_age(void);
