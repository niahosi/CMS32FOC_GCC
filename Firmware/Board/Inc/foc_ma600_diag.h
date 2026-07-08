/**
 * @file foc_ma600_diag.h
 * @brief MA600 在线调参和 NVM 诊断接口。
 */

#pragma once

#include <stdint.h>

#define MA600_DIAG_CORR_COUNT 32u

void ma600_diag_init_defaults(void);
void ma600_diag_service(void);
uint8_t ma600_diag_busy(void);

extern volatile uint8_t g_ma600_cfg_busy;
extern volatile uint8_t g_ma600_bct_cmd;
extern volatile uint8_t g_ma600_etx_cmd;
extern volatile uint8_t g_ma600_ety_cmd;
extern volatile uint8_t g_ma600_bct_apply;
extern volatile uint8_t g_ma600_bct_write;
extern volatile uint8_t g_ma600_bct_read;
extern volatile uint8_t g_ma600_et_write;
extern volatile uint8_t g_ma600_et_read;
extern volatile uint8_t g_ma600_config_ok;
extern volatile uint8_t g_ma600_corr_index;
extern volatile uint8_t g_ma600_corr_value;
extern volatile uint8_t g_ma600_corr_apply;
extern volatile uint8_t g_ma600_corr_table_cmd[MA600_DIAG_CORR_COUNT];
extern volatile uint8_t g_ma600_corr_table_read[MA600_DIAG_CORR_COUNT];
extern volatile uint8_t g_ma600_corr_table_apply;
extern volatile uint8_t g_ma600_corr_last_index;
extern volatile uint8_t g_ma600_corr_last_write;
extern volatile uint8_t g_ma600_corr_last_read;
extern volatile uint8_t g_ma600_corr_write_count;
extern volatile uint8_t g_ma600_corr_table_fail_index;
extern volatile uint8_t g_ma600_corr_table_fail_write;
extern volatile uint8_t g_ma600_corr_table_fail_read;
extern volatile uint8_t g_ma600_corr_ok;
extern volatile uint8_t g_ma600_nvm_block_cmd;
extern volatile uint8_t g_ma600_nvm_store_apply;
extern volatile uint8_t g_ma600_nvm_status;
extern volatile uint8_t g_ma600_nvm_ok;
