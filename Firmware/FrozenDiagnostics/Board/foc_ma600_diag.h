/**
 * @file foc_ma600_diag.h
 * @brief MA600 在线调参和 NVM 诊断接口。
 */

#pragma once

#include <stdint.h>

/** @brief MA600 CORR 残差表点数。 */
#define MA600_DIAG_CORR_COUNT 32u

/** @brief 上电时按 TuneConfig 默认值写入 MA600 RAM BCT/ET 配置。 */
void ma600_diag_init_defaults(void);
/** @brief 主循环低速服务入口，处理 Ozone 在线调参请求。 */
void ma600_diag_service(void);
/** @brief 返回诊断写寄存器是否正在进行，普通读角路径据此避让。 */
uint8_t ma600_diag_busy(void);

/** @brief MA600 调参写操作 busy 标志。 */
extern volatile uint8_t g_ma600_cfg_busy;
/** @brief 待写入的 BCT 强度，0..255，只写 RAM。 */
extern volatile uint8_t g_ma600_bct_cmd;
/** @brief 待写入 ETX 轴削弱标志。 */
extern volatile uint8_t g_ma600_etx_cmd;
/** @brief 待写入 ETY 轴削弱标志。 */
extern volatile uint8_t g_ma600_ety_cmd;
/** @brief 置 1 触发 BCT/ET RAM 写入，服务完成后清 0。 */
extern volatile uint8_t g_ma600_bct_apply;
/** @brief 最近一次请求写入的 BCT 值。 */
extern volatile uint8_t g_ma600_bct_write;
/** @brief 最近一次回读的 BCT 值。 */
extern volatile uint8_t g_ma600_bct_read;
/** @brief 最近一次请求写入的 ET 寄存器低位。 */
extern volatile uint8_t g_ma600_et_write;
/** @brief 最近一次回读的 ET 寄存器值。 */
extern volatile uint8_t g_ma600_et_read;
/** @brief BCT/ET 写入和回读校验结果。 */
extern volatile uint8_t g_ma600_config_ok;
/** @brief 单点 CORR 写入索引，0..31。 */
extern volatile uint8_t g_ma600_corr_index;
/** @brief 单点 CORR 写入值。 */
extern volatile uint8_t g_ma600_corr_value;
/** @brief 置 1 触发单点 CORR RAM 写入，服务完成后清 0。 */
extern volatile uint8_t g_ma600_corr_apply;
/** @brief 整表 CORR 待写入值。 */
extern volatile uint8_t g_ma600_corr_table_cmd[MA600_DIAG_CORR_COUNT];
/** @brief 整表或单点 CORR 回读值。 */
extern volatile uint8_t g_ma600_corr_table_read[MA600_DIAG_CORR_COUNT];
/** @brief 置 1 触发 32 点 CORR RAM 写入，服务完成后清 0。 */
extern volatile uint8_t g_ma600_corr_table_apply;
/** @brief 最近一次 CORR 写入索引。 */
extern volatile uint8_t g_ma600_corr_last_index;
/** @brief 最近一次 CORR 写入值。 */
extern volatile uint8_t g_ma600_corr_last_write;
/** @brief 最近一次 CORR 回读值。 */
extern volatile uint8_t g_ma600_corr_last_read;
/** @brief 整表 CORR 已成功写入点数。 */
extern volatile uint8_t g_ma600_corr_write_count;
/** @brief 整表 CORR 首个失败索引，0xFF 表示未失败。 */
extern volatile uint8_t g_ma600_corr_table_fail_index;
/** @brief 整表 CORR 失败点写入值。 */
extern volatile uint8_t g_ma600_corr_table_fail_write;
/** @brief 整表 CORR 失败点回读值。 */
extern volatile uint8_t g_ma600_corr_table_fail_read;
/** @brief CORR 写入和回读校验结果。 */
extern volatile uint8_t g_ma600_corr_ok;
/** @brief NVM 存储 block：0 配置区，1 CORR 表区。 */
extern volatile uint8_t g_ma600_nvm_block_cmd;
/** @brief 置 1 触发 NVM store，服务完成后清 0。 */
extern volatile uint8_t g_ma600_nvm_store_apply;
/** @brief NVM store 后读取的 MA600 status。 */
extern volatile uint8_t g_ma600_nvm_status;
/**
 * @brief NVM store 结果。
 * @warning 默认不要触发 NVM store；确认 RAM 配置长期稳定后再手动使用。
 */
extern volatile uint8_t g_ma600_nvm_ok;
