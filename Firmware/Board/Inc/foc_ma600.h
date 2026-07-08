/**
 * @file foc_ma600.h
 * @brief MA600 磁编码器 SPI 驱动接口。
 */

#pragma once

#include <stdint.h>

/** @brief 初始化 MA600 使用的 SSP/SPI 外设和引脚。 */
void ma600_init(void);

/**
 * @brief 处理 Ozone/JLink 在线写入的 MA600 BCT 调参请求。
 *
 * 修改 g_ma600_bct_cmd / g_ma600_etx_cmd / g_ma600_ety_cmd 后，
 * 将 g_ma600_bct_apply 置 1，主循环会调用该接口写入 RAM 寄存器并回读。
 *
 * 32 点表调试：
 * - 单点写：g_ma600_corr_index / g_ma600_corr_value / g_ma600_corr_apply = 1
 * - 整表写：填 g_ma600_corr_table_cmd[32]，再置 g_ma600_corr_table_apply = 1
 * - 存 NVM：g_ma600_nvm_block_cmd = 1 后置 g_ma600_nvm_store_apply = 1
 */
void ma600_service_config(void);

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

/** @brief 使用 32-bit 帧同时更新角度和 MA600 speed 缓存。 */
uint8_t ma600_update_speed_fast(void);

/** @brief 获取角度缓存。 */
uint16_t ma600_raw(void);
/** @brief 获取 MA600 speed 原始 signed 16-bit 输出。 */
int16_t ma600_speed_raw(void);
uint8_t ma600_ok(void);
uint8_t ma600_age(void);
