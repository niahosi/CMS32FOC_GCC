/**
 * @file foc_bsp.c
 * @brief FOC 控制板 BSP 总入口。
 */

#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_ma600.h"
#include "foc_pwm.h"
#include "CMS32M6510.h"
#include "delay.h"
#include "gpio.h"

static void clock_init(void);
static void gpio_init(void);


/** @brief 按安全顺序初始化板级 FOC 外设。 */
void bsp_init(void)
{
    clock_init();
    gpio_init();
    ma600_init();
    bsp_update_angle();
    curr_init();
    pwm_init();
    curr_calib(BOARD_CURRENT_OFFSET_SAMPLES);
}

/** @brief 在控制层初始化完成后启动 PWM/ADC 同步采样。 */
void bsp_start_adc_sync(void)
{
    curr_sync_init();
}

/** @brief ADC IRQ 中转入口，返回是否得到有效电流样本。 */
uint8_t bsp_adc_irq(void)
{
    return curr_irq();
}

/** @brief 普通路径更新一次 MA600 角度缓存。 */
uint8_t bsp_update_angle(void)
{
    return ma600_update();
}

/** @brief 快环路径更新 MA600 角度缓存，按配置选择 16-bit 或 32-bit 帧。 */
uint8_t bsp_update_angle_fast(void)
{
#if (MOT_ENCODER_FAST_READ_SPEED_FRAME != 0U)
    return ma600_update_speed_fast();
#else
    return ma600_update_fast();
#endif
}

/** @brief 获取最近缓存的 MA600 raw。 */
uint16_t bsp_angle_raw(void)
{
    return ma600_raw();
}

/** @brief 获取最近缓存的 MA600 speed raw。 */
int16_t bsp_angle_speed_raw(void)
{
    return ma600_speed_raw();
}

/** @brief 获取 MA600 缓存有效标志。 */
uint8_t bsp_angle_ok(void)
{
    return ma600_ok();
}

/** @brief 获取 MA600 缓存年龄。 */
uint8_t bsp_angle_age(void)
{
    return ma600_age();
}

/**
 * @brief 初始化系统时钟。
 * @details 更新系统时钟配置并初始化延时函数，当前配置为 64 MHz。
 */
static void clock_init(void)
{
    SystemCoreClockUpdate();
    delay_init(SystemCoreClock);
}

/**
 * @brief 初始化 GPIO 引脚。
 * @details 配置 P16 驱动使能脚，初始化为低电平以防止误触发。
 * @warning 开发阶段不要配置 P06/P07，否则调试器无法连接。
 */
static void gpio_init(void)
{
    /* P16 驱动使能脚：上电初始化阶段先拉低，等待 PWM 安全配置完成后再允许输出。 */
    GPIO_Init(PORT1, PIN6, OUTPUT);
    GPIO_PinAFInConfig(P16CFG, IO_OUTCFG_P16_GPIO);
    PORT_ClrBit(PORT1, PIN6);
}
