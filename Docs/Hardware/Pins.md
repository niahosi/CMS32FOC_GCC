# CMS32FOC_GCC Hardware Pins

本文记录当前 `cms32foc` 主固件实际使用的板级引脚和外设设置。更完整的软件链路见 `Docs/Architecture/CurrentProgramBuildAndReadMap.md`。

## Pin Mapping

| 功能 | 引脚 | 当前配置 |
| --- | --- | --- |
| Driver enable | P16 | GPIO 输出；初始化和 `pwm_off()` 时拉低，`pwm_enable(1)` 时拉高 |
| EPWM U/NU | P10/P11 | `EPWM0` / `EPWM1`，互补输出 |
| EPWM V/NV | P12/P13 | `EPWM2` / `EPWM3`，互补输出 |
| EPWM W/NW | P14/P15 | `EPWM4` / `EPWM5`，互补输出 |
| MA600 CS | P02 | 手动 CS，`IO_OUTCFG_P02_SSIO` |
| MA600 SCLK | P03 | `SCK` |
| MA600 MISO | P04 | `MISO` |
| MA600 MOSI | P05 | `MOSI` |
| U current PGA input | P00/P01 | PGA0 差分输入 |
| V current PGA input | P24/P25 | PGA1 差分输入 |
| W current PGA input | P26/P27 | PGA2 差分输入 |
| ADC current channels | PGA0/PGA1/PGA2 outputs | `ADC_CH_0`, `ADC_CH_2`, `ADC_CH_3` |

P06/P07 不配置，避免开发阶段影响调试器连接。

## PWM

| 项 | 当前值 |
| --- | ---: |
| 频率 | 20 kHz |
| 中心对齐 period | 1600 |
| 50% duty | 800 |
| duty guard | 32..1568 |
| 死区 | 32 tick |
| 默认 ADC trigger | 650 |

`pwm_init()` 配置 EPWM、互补输出、死区、ADC trigger、软件刹车和输出引脚，最后调用 `pwm_off()`。功率级默认安全关闭。

## ADC/PGA

| 项 | 当前值 |
| --- | ---: |
| ADC reference | 3.6 V |
| ADC counts | 4096 |
| Shunt | 0.08 ohm |
| PGA gain | 2 |
| 零漂采样 | 1024 samples |

`curr_init()` 配置 ADC LDO、三路差分 PGA 和 ADC 单次采样模式。`curr_sync_init()` 在控制层初始化后切换到 EPWM 硬触发连续采样并打开 ADC IRQ。

## MA600 SPI

`ma600_init()` 使用 SSP SPI mode 0、8-bit、手动 CS。当前上电会写 MA600 RAM BCT/ET 默认补偿并回读确认，不写 NVM。

当前默认快环读 16-bit angle frame。32-bit angle+speed frame 代码保留，但 `MOT_ENCODER_FAST_READ_SPEED_FRAME = 0` 时不启用。
