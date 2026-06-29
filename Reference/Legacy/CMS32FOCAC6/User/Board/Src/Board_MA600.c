#include "Board_MA600.h"
#include "delay.h"

/**
 * @file Board_MA600.c
 * @brief MA600 SPI 磁编码器板级驱动实现。
 * @details 提供阻塞式读写、运行角度缓存和 ADC 中断中使用的快速读角路径。
 */

#define MA600_SPI_TIMEOUT 100000u
#define MA600_REG_READ_CMD 0xD2u
#define MA600_REG_WRITE_UNLOCK 0xEAu
#define MA600_REG_WRITE_KEY 0x54u
#define MA600_DUMMY_BYTE 0x00u
#define MA600_MAX_AGE 255u

static volatile uint16_t s_ma600_raw;
static volatile uint8_t s_ma600_ok;
static volatile uint8_t s_ma600_age = MA600_MAX_AGE;

static void MA600_InitSpiPins(void);
static void MA600_InitSpiPeripheral(void);
static uint8_t Transfer8(uint8_t tx, uint8_t* rx);

void SPI_Master_Mode(void)
{
    /* MA600 使用手动 CS 控制，先配置 SSP，再配置复用引脚并启动外设。 */
    MA600_InitSpiPeripheral();
    MA600_InitSpiPins();
    SSP_Start();
}

void MA600_SPI_Start(void)
{
    /* MA600 的 CS 为低有效，拉低后开始一帧 SPI 事务。 */
    SSP_MasterClearCS();
}

void MA600_SPI_Stop(void)
{
    /* 拉高 CS 结束本次事务，MA600 在后续帧返回对应数据。 */
    SSP_MasterSetCS();
}

uint32_t SPI_Transmit(uint32_t data)
{
    uint32_t timeout = MA600_SPI_TIMEOUT;

    /* 先等发送 FIFO 可写，再写入一个 8 bit 数据。 */
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    SSP_SendData(data);

    timeout = MA600_SPI_TIMEOUT;
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

void MA600_SPI_Write(uint32_t addr, uint32_t buf)
{
    /*
     * MA600 写寄存器需要 unlock/key、地址数据帧和 dummy 提交帧。
     * 该接口只用于配置阶段，不放入 ADC 中断快环。
     */
    MA600_SPI_Start();
    SPI_Transmit(MA600_REG_WRITE_UNLOCK);
    SPI_Transmit(MA600_REG_WRITE_KEY);
    MA600_SPI_Stop();

    m0_delay_us(10);

    /* 第二帧写入目标寄存器地址和数据。 */
    MA600_SPI_Start();
    SPI_Transmit(addr);
    SPI_Transmit(buf);
    MA600_SPI_Stop();

    m0_delay_us(10);

    /* 第三帧发送 dummy，提交并完成写入事务。 */
    MA600_SPI_Start();
    SPI_Transmit(MA600_DUMMY_BYTE);
    SPI_Transmit(MA600_DUMMY_BYTE);
    MA600_SPI_Stop();
}

uint32_t MA600_SPI_Read_Data(uint32_t addr)
{
    uint32_t data;

    (void)addr;

    /* 兼容旧例程命名的直接读角接口，当前 addr 参数不参与连续角度读取。 */
    MA600_SPI_Start();
    data = (SPI_Transmit(MA600_DUMMY_BYTE) << 8);
    data |= SPI_Transmit(MA600_DUMMY_BYTE);
    MA600_SPI_Stop();

    return data;
}

uint32_t MA600_SPI_Read_SFR(uint32_t cmd)
{
    uint32_t data;

    MA600_SPI_Start();
    SPI_Transmit(MA600_REG_READ_CMD);
    SPI_Transmit(cmd);
    MA600_SPI_Stop();

    m0_delay_us(10);

    MA600_SPI_Start();
    SPI_Transmit(MA600_DUMMY_BYTE);
    data = SPI_Transmit(MA600_DUMMY_BYTE);
    MA600_SPI_Stop();

    return data;
}

uint16_t MA600_ReadAngle(void)
{
    uint8_t high;
    uint8_t low;

    /* 连续发送两个 dummy byte，MA600 同步返回 16 bit 原始磁场角。 */
    MA600_SPI_Start();
    high = (uint8_t)SPI_Transmit(MA600_DUMMY_BYTE);
    low = (uint8_t)SPI_Transmit(MA600_DUMMY_BYTE);
    MA600_SPI_Stop();

    return ((uint16_t)high << 8) | low;
}

uint8_t MA600_UpdateCache(void)
{
    uint8_t high = 0u;
    uint8_t low = 0u;
    uint8_t ok;

    /* 普通缓存更新带完整超时保护，适合主循环或非实时路径调用。 */
    MA600_SPI_Start();
    ok = Transfer8(MA600_DUMMY_BYTE, &high);
    ok = (uint8_t)(ok && Transfer8(MA600_DUMMY_BYTE, &low));
    MA600_SPI_Stop();

    if (ok != 0u)
    {
        s_ma600_raw = ((uint16_t)high << 8) | low;
        s_ma600_ok = 1u;
        s_ma600_age = 0u;
    }
    else
    {
        s_ma600_ok = 0u;
        if (s_ma600_age < MA600_MAX_AGE)
        {
            s_ma600_age++;
        }
    }

    return ok;
}

uint8_t MA600_UpdateCacheFast(void)
{
    uint8_t high = 0u;
    uint8_t low = 0u;
    uint32_t timeout;

    /*
     * 快速路径用于 ADC 中断后同步读角。
     * 保持逻辑直接，超时立即释放 CS 并标记缓存无效。
     */
    MA600_SPI_Start();

    timeout = 10000u;
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            MA600_SPI_Stop();
            s_ma600_ok = 0u;
            if (s_ma600_age < MA600_MAX_AGE)
            {
                s_ma600_age++;
            }
            return 0u;
        }
    }
    SSP_SendData(MA600_DUMMY_BYTE);

    timeout = 10000u;
    while (!SSP_GetRNEFlag())
    {
        if (--timeout == 0u)
        {
            MA600_SPI_Stop();
            s_ma600_ok = 0u;
            if (s_ma600_age < MA600_MAX_AGE)
            {
                s_ma600_age++;
            }
            return 0u;
        }
    }
    high = (uint8_t)SSP_GetData();

    timeout = 10000u;
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            MA600_SPI_Stop();
            s_ma600_ok = 0u;
            if (s_ma600_age < MA600_MAX_AGE)
            {
                s_ma600_age++;
            }
            return 0u;
        }
    }
    SSP_SendData(MA600_DUMMY_BYTE);

    timeout = 10000u;
    while (!SSP_GetRNEFlag())
    {
        if (--timeout == 0u)
        {
            MA600_SPI_Stop();
            s_ma600_ok = 0u;
            if (s_ma600_age < MA600_MAX_AGE)
            {
                s_ma600_age++;
            }
            return 0u;
        }
    }
    low = (uint8_t)SSP_GetData();

    MA600_SPI_Stop();

    s_ma600_raw = ((uint16_t)high << 8) | low;
    s_ma600_ok = 1u;
    s_ma600_age = 0u;

    return 1u;
}

uint16_t MA600_GetRaw(void)
{
    return s_ma600_raw;
}

uint8_t MA600_IsOk(void)
{
    return s_ma600_ok;
}

uint8_t MA600_GetAge(void)
{
    return s_ma600_age;
}

static void MA600_InitSpiPins(void)
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

static void MA600_InitSpiPeripheral(void)
{
    CGC_PER12PeriphClockCmd(CGC_PER12Periph_SPI, ENABLE);

    /* SSPCLK = PCLK / ((M + 1) * N). 当前 64 MHz 下约为 4 MHz. */
    SSP_ConfigClk(7, 2);

    SSP_ConfigRunMode(SSP_FRAME_SPI, SSP_CPO_0, SSP_CPHA_0, SSP_DAT_LENGTH_8);
    SSP_EnableMasterMode();
    SSP_DisableMasterAutoControlCS();
    SSP_MasterSetCS();
}

static uint8_t Transfer8(uint8_t tx, uint8_t* rx)
{
    uint32_t timeout = MA600_SPI_TIMEOUT;

    /* 带超时的 8 bit 全双工传输，避免 SPI 异常时永久卡住控制流程。 */
    while (!SSP_GetTNFFlag())
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    SSP_SendData(tx);

    timeout = MA600_SPI_TIMEOUT;
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
