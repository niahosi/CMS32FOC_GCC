#include "foc_ma600.h"
#include "Config.h"
#include "cgc.h"
#include "common.h"
#include "delay.h"
#include "gpio.h"
#include "ssp.h"

/**
 * @file foc_ma600.c
 * @brief MA600 SPI 磁编码器驱动实现。
 */

#define MA600_TIMEOUT 100000u
#define MA600_REG_READ_CMD 0xD2u
#define MA600_REG_WRITE_UNLOCK 0xEAu
#define MA600_REG_WRITE_KEY 0x54u
#define MA600_REG_STORE_KEY 0x55u
#define MA600_REG_BCT 0x02u
#define MA600_REG_ET 0x03u
#define MA600_REG_STATUS 0x1Au
#define MA600_REG_MTSP 0x1Cu
#define MA600_REG_CORR_BASE 0x20u
#define MA600_MTSP_SPEED 0x80u
#define MA600_MTSP_MULTITURN 0x00u
#define MA600_ETX_MASK 0x01u
#define MA600_ETY_MASK 0x02u
#define MA600_STATUS_ERROR_MASK 0x87u
#define MA600_CORR_COUNT 32u
#define MA600_DUMMY_BYTE 0x00u
#define MA600_MAX_AGE 255u
#define MA600_INIT_RETRY 3u

typedef struct
{
    volatile uint16_t raw;
    volatile int16_t speed;
    volatile uint8_t ok;
    volatile uint8_t age;
} Ma600Cache;

static Ma600Cache s_enc = {
    .raw = 0U,
    .ok = 0U,
    .age = MA600_MAX_AGE,
};

static void spi_pins_init(void);
static void spi_init(void);
static void cs_low(void);
static void cs_high(void);
static uint32_t xfer(uint32_t tx);
static uint8_t xfer8(uint8_t tx, uint8_t* rx);
static uint8_t wait_idle(uint32_t timeout);
static void configure_side_bct(void);
static uint8_t apply_side_bct(uint8_t bct, uint8_t etx, uint8_t ety);
static uint8_t apply_corr_point(uint8_t index, uint8_t value);
static uint8_t apply_corr_table(void);
static uint8_t store_nvm_block(uint8_t block);
static uint8_t write_verify_reg(uint8_t addr, uint8_t value);
static void cache_fail(void);

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
volatile uint8_t g_ma600_corr_table_cmd[MA600_CORR_COUNT];
volatile uint8_t g_ma600_corr_table_read[MA600_CORR_COUNT];
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

void ma600_init(void)
{
    uint8_t retry;
    const uint8_t mtsp =
#if (MOT_ENCODER_MTSP_SPEED_EN != 0U)
        MA600_MTSP_SPEED;
#else
        MA600_MTSP_MULTITURN;
#endif

    /* MA600 使用手动 CS 控制，先配置 SSP，再配置复用引脚并启动外设。 */
    spi_init();
    spi_pins_init();
    SSP_Start();

    configure_side_bct();

    for (retry = 0u; retry < MA600_INIT_RETRY; retry++)
    {
        ma600_write_reg(MA600_REG_MTSP, mtsp);
        m0_delay_us(10000);
        if ((uint8_t)ma600_read_reg(MA600_REG_MTSP) == mtsp)
        {
            break;
        }
    }
}

void ma600_service_config(void)
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

static void cs_low(void)
{
    /* MA600 的 CS 为低有效，拉低后开始一帧 SPI 事务。 */
    SSP_MasterClearCS();
}

static void cs_high(void)
{
    /* 拉高 CS 结束本次事务，MA600 在后续帧返回对应数据。 */
    SSP_MasterSetCS();
}

static uint32_t xfer(uint32_t tx)
{
    uint32_t timeout = MA600_TIMEOUT;

    /* 先等发送 FIFO 可写，再写入一个 8 bit 数据。 */
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    SSP_SendData(tx);

    timeout = MA600_TIMEOUT;
    /* SPI 全双工：发送一个字节后，需要等待接收 FIFO 中出现同一帧返回字节。 */
    while (!SSP_GetRNEFlag())
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    return SSP_GetData();
}

void ma600_write_reg(uint32_t addr, uint32_t buf)
{
    /*
     * MA600 写寄存器需要 unlock/key、地址数据帧和 dummy 提交帧。
     * 该接口只用于配置阶段，不放入 ADC 中断快环。
     */
    cs_low();
    xfer(MA600_REG_WRITE_UNLOCK);
    xfer(MA600_REG_WRITE_KEY);
    cs_high();

    m0_delay_us(10);

    /* 第二帧写入目标寄存器地址和数据。 */
    cs_low();
    xfer(addr);
    xfer(buf);
    cs_high();

    m0_delay_us(10);

    /* 第三帧发送 dummy，提交并完成写入事务。 */
    cs_low();
    xfer(MA600_DUMMY_BYTE);
    xfer(MA600_DUMMY_BYTE);
    cs_high();
}

uint32_t ma600_read_data(uint32_t addr)
{
    uint32_t data;

    (void)addr;

    /* 兼容旧例程命名的直接读角接口，当前 addr 参数不参与连续角度读取。 */
    cs_low();
    data = (xfer(MA600_DUMMY_BYTE) << 8);
    data |= xfer(MA600_DUMMY_BYTE);
    cs_high();

    return data;
}

uint32_t ma600_read_reg(uint32_t cmd)
{
    uint32_t data;

    cs_low();
    xfer(MA600_REG_READ_CMD);
    xfer(cmd);
    cs_high();

    m0_delay_us(10);

    cs_low();
    xfer(MA600_DUMMY_BYTE);
    data = xfer(MA600_DUMMY_BYTE);
    cs_high();

    return data;
}

uint16_t ma600_read_angle(void)
{
    uint8_t high;
    uint8_t low;

    /* 连续发送两个 dummy byte，MA600 同步返回 16 bit 原始磁场角。 */
    cs_low();
    high = (uint8_t)xfer(MA600_DUMMY_BYTE);
    low = (uint8_t)xfer(MA600_DUMMY_BYTE);
    cs_high();

    return ((uint16_t)high << 8) | low;
}

uint8_t ma600_update(void)
{
    uint8_t high = 0u;
    uint8_t low = 0u;
    uint8_t ok;

    if (g_ma600_cfg_busy != 0U)
    {
        return 0U;
    }

    /* 普通缓存更新带完整超时保护，适合主循环或非实时路径调用。 */
    cs_low();
    ok = xfer8(MA600_DUMMY_BYTE, &high);
    ok = (uint8_t)(ok && xfer8(MA600_DUMMY_BYTE, &low));
    cs_high();

    if (ok != 0u)
    {
        s_enc.raw = ((uint16_t)high << 8) | low;
        s_enc.ok = 1u;
        s_enc.age = 0u;
    }
    else
    {
        cache_fail();
    }

    return ok;
}

uint8_t ma600_update_fast(void)
{
    uint8_t high = 0u;
    uint8_t low = 0u;
    uint32_t timeout;

    if (g_ma600_cfg_busy != 0U)
    {
        return 0U;
    }

    /*
     * 快速路径用于 ADC 中断后同步读角。
     * 保持逻辑直接，超时立即释放 CS 并标记缓存无效。
     */
    cs_low();

    timeout = 10000u;
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            cs_high();
            cache_fail();
            return 0u;
        }
    }
    SSP_SendData(MA600_DUMMY_BYTE);

    timeout = 10000u;
    while (!SSP_GetRNEFlag())
    {
        if (--timeout == 0u)
        {
            cs_high();
            cache_fail();
            return 0u;
        }
    }
    high = (uint8_t)SSP_GetData();

    timeout = 10000u;
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            cs_high();
            cache_fail();
            return 0u;
        }
    }
    SSP_SendData(MA600_DUMMY_BYTE);

    timeout = 10000u;
    while (!SSP_GetRNEFlag())
    {
        if (--timeout == 0u)
        {
            cs_high();
            cache_fail();
            return 0u;
        }
    }
    low = (uint8_t)SSP_GetData();

    if (wait_idle(10000u) == 0u)
    {
        cs_high();
        cache_fail();
        return 0u;
    }
    cs_high();

    s_enc.raw = ((uint16_t)high << 8) | low;
    s_enc.ok = 1u;
    s_enc.age = 0u;

    return 1u;
}

uint8_t ma600_update_speed_fast(void)
{
    uint8_t rx[4] = {0u, 0u, 0u, 0u};
    uint32_t timeout;
    uint8_t i;

    if (g_ma600_cfg_busy != 0U)
    {
        return 0U;
    }

    /*
     * MA600 在 MTSP=1 时，32-bit 连续帧返回 angle[15:0] + speed[15:0]。
     * 该路径给速度环使用，仍保留短超时，避免 SPI 异常卡死 ADC 快环。
     */
    cs_low();
    for (i = 0u; i < 4u; i++)
    {
        timeout = 10000u;
        while (!SSP_GetTNFFlag())
        {
            if (--timeout == 0u)
            {
                cs_high();
                cache_fail();
                return 0u;
            }
        }
        SSP_SendData(MA600_DUMMY_BYTE);

        timeout = 10000u;
        while (!SSP_GetRNEFlag())
        {
            if (--timeout == 0u)
            {
                cs_high();
                cache_fail();
                return 0u;
            }
        }
        rx[i] = (uint8_t)SSP_GetData();
    }
    if (wait_idle(10000u) == 0u)
    {
        cs_high();
        cache_fail();
        return 0u;
    }
    cs_high();

    s_enc.raw = ((uint16_t)rx[0] << 8) | rx[1];
    s_enc.speed = (int16_t)(((uint16_t)rx[2] << 8) | rx[3]);
    s_enc.ok = 1u;
    s_enc.age = 0u;

    return 1u;
}

uint16_t ma600_raw(void)
{
    return s_enc.raw;
}

int16_t ma600_speed_raw(void)
{
    return s_enc.speed;
}

uint8_t ma600_ok(void)
{
    return s_enc.ok;
}

uint8_t ma600_age(void)
{
    return s_enc.age;
}

static void spi_pins_init(void)
{
    /* P02=CS/CNS, P03=SCLK, P04=MISO, P05=MOSI，与原理图 MA600 接线一致。 */
    GPIO_PinAFOutConfig(P03CFG, IO_OUTCFG_P03_SCK);
    GPIO_Init(PORT0, PIN3, OUTPUT);

    GPIO_PinAFOutConfig(P04CFG, IO_OUTCFG_P04_MISO);
    GPIO_Init(PORT0, PIN4, INPUT);

    GPIO_PinAFOutConfig(P05CFG, IO_OUTCFG_P05_MOSI);
    GPIO_Init(PORT0, PIN5, OUTPUT);

    RESTPinGpio_Set(ENABLE);
    GPIO_PinAFOutConfig(P02CFG, IO_OUTCFG_P02_SSIO);
    GPIO_Init(PORT0, PIN2, OUTPUT);
}

static void spi_init(void)
{
    CGC_PER12PeriphClockCmd(CGC_PER12Periph_SPI, ENABLE);

    /* SSPCLK = PCLK / ((M + 1) * N). 当前 64 MHz 下约为 4 MHz. */
    SSP_ConfigClk(7, 1);

    SSP_ConfigRunMode(SSP_FRAME_SPI, SSP_CPO_0, SSP_CPHA_0, SSP_DAT_LENGTH_8);
    SSP_EnableMasterMode();
    SSP_DisableMasterAutoControlCS();
    SSP_MasterSetCS();
}

static uint8_t xfer8(uint8_t tx, uint8_t* rx)
{
    uint32_t timeout = MA600_TIMEOUT;

    /* 带超时的 8 bit 全双工传输，避免 SPI 异常时永久卡住控制流程。 */
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    SSP_SendData(tx);

    timeout = MA600_TIMEOUT;
    while (!SSP_GetRNEFlag())
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    *rx = (uint8_t)SSP_GetData();
    return 1u;
}

static uint8_t wait_idle(uint32_t timeout)
{
    while (SSP_GetBusyFlag())
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    return 1u;
}

static void configure_side_bct(void)
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
                     ((g_ma600_et_read & (MA600_ETX_MASK | MA600_ETY_MASK)) == g_ma600_et_write));
}

static uint8_t apply_corr_point(uint8_t index, uint8_t value)
{
    if (index >= MA600_CORR_COUNT)
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
        g_ma600_corr_last_read = (uint8_t)ma600_read_reg((uint8_t)(MA600_REG_CORR_BASE + index));
        if (g_ma600_corr_last_read != value)
        {
            return 0U;
        }
    }
    else
    {
        g_ma600_corr_last_read = (uint8_t)ma600_read_reg((uint8_t)(MA600_REG_CORR_BASE + index));
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
    for (i = 0U; i < MA600_CORR_COUNT; i++)
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

    cs_low();
    xfer(MA600_REG_WRITE_UNLOCK);
    xfer(MA600_REG_STORE_KEY);
    cs_high();

    m0_delay_us(1000);

    cs_low();
    xfer(MA600_REG_WRITE_UNLOCK);
    xfer(block);
    cs_high();

    m0_delay_us(10000);

    cs_low();
    xfer(MA600_DUMMY_BYTE);
    xfer(MA600_DUMMY_BYTE);
    cs_high();

    m0_delay_us(10000);
    g_ma600_nvm_status = (uint8_t)ma600_read_reg(MA600_REG_STATUS);
    return (uint8_t)((g_ma600_nvm_status & MA600_STATUS_ERROR_MASK) == 0U);
}

static uint8_t write_verify_reg(uint8_t addr, uint8_t value)
{
    uint8_t retry;

    for (retry = 0U; retry < MA600_INIT_RETRY; retry++)
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

static void cache_fail(void)
{
    s_enc.ok = 0u;
    if (s_enc.age < MA600_MAX_AGE)
    {
        s_enc.age++;
    }
}
