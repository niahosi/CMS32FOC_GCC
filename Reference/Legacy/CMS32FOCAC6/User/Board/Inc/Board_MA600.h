/**
 * @file Board_MA600.h
 * @brief MA600 磁编码器板级驱动接口。
 * @details 提供 MA600 SPI 初始化、寄存器读写和角度缓存更新。
 */

#pragma once
#include "cgc.h"
#include "common.h"
#include "gpio.h"
#include "ssp.h"

/**
 * @brief 初始化 MA600 使用的 SPI 主机模式。
 */
void SPI_Master_Mode(void);

/**
 * @brief 向 MA600 寄存器写入 8 bit 数据。
 * @param[in] addr 寄存器地址。
 * @param[in] buf 写入数据。
 */
void MA600_SPI_Write(uint32_t addr, uint32_t buf);

/**
 * @brief 读取 MA600 连续角度数据。
 * @param[in] addr 兼容旧接口的地址参数，当前不使用。
 * @return 16 bit 原始角度数据。
 */
uint32_t MA600_SPI_Read_Data(uint32_t addr);

/**
 * @brief 读取 MA600 SFR 寄存器。
 * @param[in] cmd 寄存器地址。
 * @return 寄存器读回值。
 */
uint32_t MA600_SPI_Read_SFR(uint32_t cmd);

/**
 * @brief 直接读取一次 MA600 原始角度。
 * @return 原始角度 count，范围 0~65535。
 */
uint16_t MA600_ReadAngle(void);

/**
 * @brief 更新 MA600 角度缓存。
 * @return 1 表示更新成功，0 表示读取失败。
 */
uint8_t MA600_UpdateCache(void);

/**
 * @brief 快速更新 MA600 角度缓存。
 * @return 1 表示更新成功，0 表示读取失败。
 */
uint8_t MA600_UpdateCacheFast(void);

/**
 * @brief Getter: 获取 MA600 缓存原始角度。
 * @return 原始角度 count，范围 0~65535。
 */
uint16_t MA600_GetRaw(void);

/**
 * @brief Getter: 判断 MA600 最近一次缓存是否有效。
 * @return 1 表示有效，0 表示无效。
 */
uint8_t MA600_IsOk(void);

/**
 * @brief Getter: 获取 MA600 缓存年龄。
 * @return 距离最近一次成功更新的失败/老化计数。
 */
uint8_t MA600_GetAge(void);
