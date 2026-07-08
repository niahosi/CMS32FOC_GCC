#include "foc_ma600_diag.h"

#include "Config.h"
#include "delay.h"
#include "foc_ma600.h"

#define MA600_REG_BCT 0x02u
#define MA600_REG_ET 0x03u
#define MA600_REG_STATUS 0x1Au
#define MA600_REG_CORR_BASE 0x20u
#define MA600_ETX_MASK 0x01u
#define MA600_ETY_MASK 0x02u
#define MA600_STATUS_ERROR_MASK 0x87u
#define MA600_DIAG_RETRY 3u

volatile uint8_t g_ma600_cfg_busy;
volatile uint8_t g_ma600_bct_cmd = (uint8_t)MOT_ENCODER_SIDE_BCT;
volatile uint8_t g_ma600_etx_cmd = (uint8_t)MOT_ENCODER_SIDE_ETX;
volatile uint8_t g_ma600_ety_cmd = (uint8_t)MOT_ENCODER_SIDE_ETY;
volatile uint8_t g_ma600_bct_apply;
volatile uint8_t g_ma600_bct_write;
volatile uint8_t g_ma600_bct_read;
volatile uint8_t g_ma600_et_write;
volatile uint8_t g_ma600_et_read;
volatile uint8_t g_ma600_config_ok;
volatile uint8_t g_ma600_corr_index;
volatile uint8_t g_ma600_corr_value;
volatile uint8_t g_ma600_corr_apply;
volatile uint8_t g_ma600_corr_table_cmd[MA600_DIAG_CORR_COUNT];
volatile uint8_t g_ma600_corr_table_read[MA600_DIAG_CORR_COUNT];
volatile uint8_t g_ma600_corr_table_apply;
volatile uint8_t g_ma600_corr_last_index;
volatile uint8_t g_ma600_corr_last_write;
volatile uint8_t g_ma600_corr_last_read;
volatile uint8_t g_ma600_corr_write_count;
volatile uint8_t g_ma600_corr_table_fail_index;
volatile uint8_t g_ma600_corr_table_fail_write;
volatile uint8_t g_ma600_corr_table_fail_read;
volatile uint8_t g_ma600_corr_ok;
volatile uint8_t g_ma600_nvm_block_cmd = 1U;
volatile uint8_t g_ma600_nvm_store_apply;
volatile uint8_t g_ma600_nvm_status;
volatile uint8_t g_ma600_nvm_ok;

static uint8_t apply_side_bct(uint8_t bct, uint8_t etx, uint8_t ety);
static uint8_t apply_corr_point(uint8_t index, uint8_t value);
static uint8_t apply_corr_table(void);
static uint8_t store_nvm_block(uint8_t block);
static uint8_t write_verify_reg(uint8_t addr, uint8_t value);

void ma600_diag_init_defaults(void)
{
#if (MOT_ENCODER_SIDE_BCT_EN != 0U)
    g_ma600_config_ok =
        apply_side_bct((uint8_t)MOT_ENCODER_SIDE_BCT,
                       (uint8_t)MOT_ENCODER_SIDE_ETX,
                       (uint8_t)MOT_ENCODER_SIDE_ETY);
#else
    g_ma600_bct_write = 0U;
    g_ma600_bct_read = (uint8_t)ma600_read_reg(MA600_REG_BCT);
    g_ma600_et_write = 0U;
    g_ma600_et_read = (uint8_t)ma600_read_reg(MA600_REG_ET);
    g_ma600_config_ok = 1U;
#endif
}

void ma600_diag_service(void)
{
    uint8_t bct;
    uint8_t etx;
    uint8_t ety;

    if (g_ma600_bct_apply != 0U)
    {
        bct = g_ma600_bct_cmd;
        etx = (g_ma600_etx_cmd != 0U) ? 1U : 0U;
        ety = (g_ma600_ety_cmd != 0U) ? 1U : 0U;
        if ((etx != 0U) && (ety != 0U))
        {
            ety = 0U;
            g_ma600_ety_cmd = 0U;
        }

        g_ma600_cfg_busy = 1U;
        g_ma600_config_ok = apply_side_bct(bct, etx, ety);
        g_ma600_cfg_busy = 0U;
        g_ma600_bct_apply = 0U;
    }

    if (g_ma600_corr_apply != 0U)
    {
        g_ma600_cfg_busy = 1U;
        g_ma600_corr_ok = apply_corr_point(g_ma600_corr_index, g_ma600_corr_value);
        g_ma600_cfg_busy = 0U;
        g_ma600_corr_apply = 0U;
    }

    if (g_ma600_corr_table_apply != 0U)
    {
        g_ma600_cfg_busy = 1U;
        g_ma600_corr_ok = apply_corr_table();
        g_ma600_cfg_busy = 0U;
        g_ma600_corr_table_apply = 0U;
    }

    if (g_ma600_nvm_store_apply != 0U)
    {
        g_ma600_cfg_busy = 1U;
        g_ma600_nvm_ok = store_nvm_block(g_ma600_nvm_block_cmd);
        g_ma600_cfg_busy = 0U;
        g_ma600_nvm_store_apply = 0U;
    }
}

uint8_t ma600_diag_busy(void)
{
    return g_ma600_cfg_busy;
}

static uint8_t apply_side_bct(uint8_t bct, uint8_t etx, uint8_t ety)
{
    const uint8_t et =
        (uint8_t)(((etx != 0U) ? MA600_ETX_MASK : 0U) |
                  ((ety != 0U) ? MA600_ETY_MASK : 0U));

    g_ma600_bct_write = bct;
    g_ma600_et_write = et;
    g_ma600_config_ok = 0U;

    if (write_verify_reg(MA600_REG_BCT, bct) == 0U)
    {
        return 0U;
    }
    g_ma600_bct_read = (uint8_t)ma600_read_reg(MA600_REG_BCT);

    if (write_verify_reg(MA600_REG_ET, et) == 0U)
    {
        return 0U;
    }
    g_ma600_et_read = (uint8_t)ma600_read_reg(MA600_REG_ET);

    return (uint8_t)((g_ma600_bct_read == g_ma600_bct_write) &&
                     ((g_ma600_et_read & (MA600_ETX_MASK | MA600_ETY_MASK)) ==
                      g_ma600_et_write));
}

static uint8_t apply_corr_point(uint8_t index, uint8_t value)
{
    if (index >= MA600_DIAG_CORR_COUNT)
    {
        g_ma600_corr_last_index = index;
        g_ma600_corr_last_write = value;
        g_ma600_corr_last_read = 0U;
        return 0U;
    }

    g_ma600_corr_last_index = index;
    g_ma600_corr_last_write = value;
    if (write_verify_reg((uint8_t)(MA600_REG_CORR_BASE + index), value) == 0U)
    {
        g_ma600_corr_last_read =
            (uint8_t)ma600_read_reg((uint8_t)(MA600_REG_CORR_BASE + index));
        if (g_ma600_corr_last_read != value)
        {
            return 0U;
        }
    }
    else
    {
        g_ma600_corr_last_read =
            (uint8_t)ma600_read_reg((uint8_t)(MA600_REG_CORR_BASE + index));
    }
    g_ma600_corr_table_read[index] = g_ma600_corr_last_read;
    return (uint8_t)(g_ma600_corr_last_read == value);
}

static uint8_t apply_corr_table(void)
{
    uint8_t i;

    g_ma600_corr_write_count = 0U;
    g_ma600_corr_table_fail_index = 0xFFU;
    g_ma600_corr_table_fail_write = 0U;
    g_ma600_corr_table_fail_read = 0U;
    for (i = 0U; i < MA600_DIAG_CORR_COUNT; i++)
    {
        if (apply_corr_point(i, g_ma600_corr_table_cmd[i]) == 0U)
        {
            g_ma600_corr_table_fail_index = i;
            g_ma600_corr_table_fail_write = g_ma600_corr_last_write;
            g_ma600_corr_table_fail_read = g_ma600_corr_last_read;
            return 0U;
        }
        g_ma600_corr_write_count++;
    }

    return 1U;
}

static uint8_t store_nvm_block(uint8_t block)
{
    if (block > 1U)
    {
        g_ma600_nvm_status = (uint8_t)ma600_read_reg(MA600_REG_STATUS);
        return 0U;
    }

    ma600_store_nvm_block(block);
    g_ma600_nvm_status = (uint8_t)ma600_read_reg(MA600_REG_STATUS);
    return (uint8_t)((g_ma600_nvm_status & MA600_STATUS_ERROR_MASK) == 0U);
}

static uint8_t write_verify_reg(uint8_t addr, uint8_t value)
{
    uint8_t retry;

    for (retry = 0U; retry < MA600_DIAG_RETRY; retry++)
    {
        ma600_write_reg(addr, value);
        m0_delay_us(1000);
        if ((uint8_t)ma600_read_reg(addr) == value)
        {
            return 1U;
        }
    }

    return 0U;
}
