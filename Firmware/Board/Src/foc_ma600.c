#include "foc_ma600.h"
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
#define MA600_DUMMY_BYTE 0x00u
#define MA600_MAX_AGE 255u

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
static uint32_t xfer(uint32_t tx);
static uint8_t xfer8(uint8_t tx, uint8_t* rx);
static void cache_fail(void);

void ma600_init(void)
{
    /* MA600 使用手动 CS 控制，先配置 SSP，再配置复用引脚并启动外设。 */
    spi_init();
    spi_pins_init();
    SSP_Start();
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

    cs_high();

    s_enc.raw = ((uint16_t)high << 8) | low;
    s_enc.ok = 1u;
    s_enc.age = 0u;

    return 1u;
}

uint16_t ma600_raw(void)
{
    return s_enc.raw;
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
    SSP_ConfigClk(7, 2);

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

static void cache_fail(void)
{
    s_enc.ok = 0u;
    if (s_enc.age < MA600_MAX_AGE)
    {
        s_enc.age++;
    }
}
