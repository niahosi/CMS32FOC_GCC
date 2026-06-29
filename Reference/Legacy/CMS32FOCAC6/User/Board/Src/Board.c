/**
 * @file Board.c
 * @brief 板级硬件抽象层实现。
 * @details 实现 CMS32M6513AGE40NB 控制板的硬件初始化和驱动功能，
 *          包括时钟、GPIO、MA600 SPI、ADC/PGA、EPWM 等外设的配置和控制。
 */

#include "Board.h"
#include "Board_Analog.h"
#include "Board_MA600.h"
#include "Board_PWM.h"
#include "CMS32M6510.h"
#include "delay.h"

static void InitClock(void);
static void InitGpio(void);
static void InitMa600Spi(void);

void Board_Init(void)
{
    InitClock();
    InitGpio();
    InitMa600Spi();
    Board_UpdateAngle();
    Board_InitAdc();
    Board_InitPwm();

    Board_CalibCurrentOffset(BOARD_CURRENT_OFFSET_SAMPLES);
    Board_InitPwmAdcSync();
}

uint8_t Board_HandleAdcIrq(void)
{
    return Board_AdcIrqHandler();
}

uint8_t Board_UpdateAngle(void)
{
    return MA600_UpdateCache();
}

uint8_t Board_UpdateAngleFast(void)
{
    return MA600_UpdateCacheFast();
}

uint16_t Board_GetAngleRaw(void)
{
    return MA600_GetRaw();
}

uint8_t Board_IsAngleOk(void)
{
    return MA600_IsOk();
}

uint8_t Board_GetAngleAge(void)
{
    return MA600_GetAge();
}

uint16_t Board_ReadAngleRaw(void)
{
    return MA600_ReadAngle();
}

/**
 * @brief 初始化系统时钟。
 * @details 更新系统时钟配置并初始化延时函数，当前配置为 64 MHz。
 */
static void InitClock(void)
{
    SystemCoreClockUpdate();
    delay_init(SystemCoreClock);
}

/**
 * @brief 初始化 GPIO 引脚。
 * @details 配置 P16 驱动使能脚，初始化为低电平以防止误触发。
 * @warning 开发阶段不要配置 P06/P07，否则调试器无法连接。
 */
static void InitGpio(void)
{
    /* P16 驱动使能脚：上电初始化阶段先拉低，等待 PWM 安全配置完成后再允许输出。 */
    GPIO_Init(PORT1, PIN6, OUTPUT);
    GPIO_PinAFInConfig(P16CFG, IO_OUTCFG_P16_GPIO);
    PORT_ClrBit(PORT1, PIN6);
}

/**
 * @brief 初始化 MA600 SPI 接口。
 */
static void InitMa600Spi(void)
{
    SPI_Master_Mode();
}
