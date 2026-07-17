#include "foc_ma600.h"
#include "Config.h"
#include "TuneConfig.h"
#include "cgc.h"
#include "common.h"
#include "delay.h"
#include "gpio.h"
#include "ssp.h"
#include <stdint.h>

/**
 * @file foc_ma600.c
 * @brief MA600 SPI 磁编码器驱动实现。
 */

#define MA600_TIMEOUT 100000u
#define MA600_REG_READ_CMD 0xD2u
#define MA600_REG_WRITE_UNLOCK 0xEAu
#define MA600_REG_WRITE_KEY 0x54u
#define MA600_REG_BCT 0x02u
#define MA600_REG_ET 0x03u
#define MA600_REG_MTSP 0x1Cu
#define MA600_ETX_MASK 0x01u
#define MA600_ETY_MASK 0x02u
#define MA600_MTSP_MULTITURN 0x00u
#define MA600_DUMMY_BYTE 0x00u
#define MA600_MAX_AGE 255u
#define MA600_INIT_RETRY 3u
#define MA600_CONFIG_RETRY 3u

/** @brief MA600 最近一次角度读取缓存。 */
typedef struct
{
    volatile uint16_t raw;
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
static void drain_rx_fifo(void);
static uint32_t xfer(uint32_t tx);
static uint8_t xfer8(uint8_t tx, uint8_t *rx);
static uint8_t wait_idle(uint32_t timeout);
static void cache_fail(void);
static void init_compensation_defaults(void);
static uint8_t write_verify_reg(uint8_t addr, uint8_t value);

volatile uint32_t g_ma600_rx_drain_count;
volatile uint8_t g_ma600_rx_drain_last;

/** @brief 初始化 MA600 SPI、默认补偿寄存器和 MTSP 角度输出模式。 */
void ma600_init(void)
{
    uint8_t retry;
    const uint8_t mtsp = MA600_MTSP_MULTITURN;

    /* MA600 使用手动 CS 控制，先配置 SSP，再配置复用引脚并启动外设。 */
    spi_init();
    spi_pins_init();
    SSP_Start();

    init_compensation_defaults();

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

/** @brief 拉低 CS 前清空残留 RX FIFO，开始一帧 SPI 事务。 */
static void cs_low(void)
{
    /* MA600 的 CS 为低有效，拉低后开始一帧 SPI 事务。 */
    drain_rx_fifo();
    SSP_MasterClearCS();
}

/** @brief 拉高 CS，结束当前 SPI 事务。 */
static void cs_high(void)
{
    /* 拉高 CS 结束本次事务，MA600 在后续帧返回对应数据。 */
    SSP_MasterSetCS();
}

/** @brief 清空 SSP RX FIFO 中残留字节，并记录诊断计数。 */
static void drain_rx_fifo(void)
{
    uint8_t drained = 0U;

    while ((SSP_GetRNEFlag() != 0U) && (drained < 16U))
    {
        (void)SSP_GetData();
        drained++;
    }

    if (drained != 0U)
    {
        g_ma600_rx_drain_count += drained;
        g_ma600_rx_drain_last = drained;
    }
    else
    {
        g_ma600_rx_drain_last = 0U;
    }
}

/** @brief 阻塞式传输一个 8-bit SPI 字节，带较长超时。 */
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

/** @brief 按 MA600 unlock/key/addr/dummy 时序写一个 RAM 寄存器。 */
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

/** @brief 按 MA600 两帧 read 命令读取 SFR/RAM 寄存器。 */
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

/** @brief 直接读取一次 16-bit 连续角度，不更新缓存状态。 */
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

/** @brief 普通路径读取 16-bit 角度并更新缓存，调参 busy 时拒绝读取。 */
uint8_t ma600_update(void)
{
    uint8_t high = 0u;
    uint8_t low = 0u;
    uint8_t ok;

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

/** @brief ADC 快环短超时读取 16-bit 角度并更新缓存。 */
uint8_t ma600_update_fast(void)
{
    uint8_t high = 0u;
    uint8_t low = 0u;
    uint32_t timeout;

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

/** @brief 返回最近缓存的 raw 角度。 */
uint16_t ma600_raw(void)
{
    return s_enc.raw;
}

/** @brief 返回最近一次缓存更新是否成功。 */
uint8_t ma600_ok(void)
{
    return s_enc.ok;
}

/** @brief 返回角度缓存年龄。 */
uint8_t ma600_age(void)
{
    return s_enc.age;
}

/** @brief 配置 MA600 SPI 引脚和手动 CS 引脚。 */
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

/** @brief 配置 SSP 为 SPI mode 0、8-bit、手动 CS。 */
static void spi_init(void)
{
    CGC_PER12PeriphClockCmd(CGC_PER12Periph_SPI, ENABLE);

    /* SSPCLK = PCLK / ((M + 1) * N)。当前默认 64 MHz / ((7 + 1) * 2) = 4 MHz。 */
    SSP_ConfigClk(MA600_SSP_CLK_M, MA600_SSP_CLK_N);

    SSP_ConfigRunMode(SSP_FRAME_SPI, SSP_CPO_0, SSP_CPHA_0, SSP_DAT_LENGTH_8);
    SSP_EnableMasterMode();
    SSP_DisableMasterAutoControlCS();
    SSP_MasterSetCS();
}

/** @brief 带超时传输一个 8-bit SPI 字节并返回成功标志。 */
static uint8_t xfer8(uint8_t tx, uint8_t *rx)
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

/** @brief 等待 SSP 总线空闲，用于确保 CS 拉高前帧已完成。 */
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

/** @brief 标记 MA600 缓存失效并增加 age。 */
static void cache_fail(void)
{
    s_enc.ok = 0u;
    if (s_enc.age < MA600_MAX_AGE)
    {
        s_enc.age++;
    }
}

/** @brief 上电按编译配置写入 MA600 BCT/ET RAM 补偿，不触发 NVM。 */
static void init_compensation_defaults(void)
{
#if (MOT_ENCODER_SIDE_BCT_EN != 0U)
    const uint8_t et = (uint8_t)(((MOT_ENCODER_SIDE_ETX != 0U) ? MA600_ETX_MASK : 0U) |
                                 ((MOT_ENCODER_SIDE_ETY != 0U) ? MA600_ETY_MASK : 0U));

    /*
     * 这里保留旧诊断模块的“写入并回读确认”语义。在线调参 service 已冻结，
     * 但默认 BCT/ET RAM 补偿仍是主 FOC 角度质量的一部分，不能只写一次就假定成功。
     */
    (void)write_verify_reg(MA600_REG_BCT, (uint8_t)MOT_ENCODER_SIDE_BCT);
    (void)write_verify_reg(MA600_REG_ET, et);
#endif
}

/** @brief 写 MA600 RAM 寄存器并回读确认，失败时短重试。 */
static uint8_t write_verify_reg(uint8_t addr, uint8_t value)
{
    uint8_t retry;

    for (retry = 0U; retry < MA600_CONFIG_RETRY; retry++)
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
