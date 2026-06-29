# 17 LXQS硬件配置参考

本文记录参考工程：

```text
D:\wjw\1_Motor\CMS32M65xx\Reference\LXQS_FBL2\LXQS_LQS_FBL2_CMS32M6510_H100_F306_C000_20260424
```

用途：

- 参考它的同芯片 CMS32M65xx 外设初始化顺序。
- 对照 EPWM、ADC、PGA、ACMP、SPI、GPIO 的配置方法。
- 作为当前 `CMS32FOCAC6` 工程 EPWM/ADC/PGA/ACMP/SPI/中断配置的成熟参考。

注意：该工程不是直接给当前板子复制使用的模板。LXQS 与当前项目按同一种 CMS32M65xx 芯片外设平台处理，但板级硬件、电流采样方式、ADC 通道、SPI 片选脚、板级使能脚和编码器方案可能不同。本文只记录“参考工程怎么做”，以及“当前工程可以借鉴什么”。

## 同芯片参考定位与硬件真值

当前基线按用户确认处理：LXQS 与当前项目属于同一种 CMS32M65xx 芯片外设参考。工程目录名或早期记录里的 `6510/6513` 区分，不再作为“LXQS 不能参考当前芯片外设”的结论。

当前判断优先级：

| 优先级 | 来源 | 用法 |
| --- | --- | --- |
| 1 | 当前板原理图和实测硬件 | 决定引脚、电源、采样电阻、CS、使能、保护和 OUT_U/V/W |
| 2 | 当前 CMS32M65xx 数据手册/用户手册 | 决定寄存器、时钟、EPWM、ADC、PGA、ACMP、SSP 的合法配置 |
| 3 | LXQS 参考工程 | 参考同芯片外设初始化顺序和 FOC 快环调度 |
| 4 | ACC 参考工程 | 参考成熟 FOC、MA600 事务、速度/位置环组织 |

结论：

- LXQS 可以优先参考 `EPWM/ADC/PGA/ACMP/SPI/GPIO/中断` 的初始化结构。
- LXQS 的板级引脚、采样通道、片选脚、使能脚必须回到当前原理图核对。
- LXQS 的编码器算法不能直接照搬到当前 MA600 四对极磁环。
- 当前板的 MA600 原始角度已经随机械一圈变化 4 个周期，FOC 电角度不再额外乘 4。

因此 LXQS 中这些配置仍有外设参考价值：

```c
GPIO_PinAFOutConfig(P10CFG, IO_OUTCFG_P10_EPWM0);
GPIO_PinAFOutConfig(P11CFG, IO_OUTCFG_P11_EPWM1);
GPIO_PinAFOutConfig(P12CFG, IO_OUTCFG_P12_EPWM2);
GPIO_PinAFOutConfig(P13CFG, IO_OUTCFG_P13_EPWM3);
GPIO_PinAFOutConfig(P14CFG, IO_OUTCFG_P14_EPWM4);
GPIO_PinAFOutConfig(P15CFG, IO_OUTCFG_P15_EPWM5);
EPWM->POREMAP = 0xAA543210;
```

但它们是否等同于当前板的正式功率输出路径，必须由当前原理图、当前芯片手册和示波器实测共同确认。

## 关键文件入口

| 内容 | 文件 |
| --- | --- |
| 硬件宏配置 | `Project/code/5_user_h/Set_HW.h` |
| 保护参数 | `Project/code/5_user_h/Set_PROTECT.h` |
| MCU 初始化 | `Project/code/2_mcu_s/mcu_init.c` |
| MCU 接口和 PWM 更新 | `Project/code/2_mcu_s/mcu_port.c` |
| mask/输出开关宏 | `Project/code/2_mcu_h/mcu_port.h` |
| ADC/EPWM/ACMP 中断 | `Project/code/5_user_s/interrupt.c` |
| 主流程 | `Project/code/5_user_s/main.c` |

## 编码器 / MA600 角度处理参考

LXQS 工程中没有直接使用 `MA600` 这个名字，读角接口表现为：

```c
Angle = SPI_KTH7801_Read(CMD_READ_ANGLE);
GetEncoderAngle(&Stru_Encoder, Angle);
Stru_Foc.Elec_Theta = Stru_Encoder.ThetaE;
Stru_Foc.Elec_Omega = Stru_Encoder.Omega;
Stru_Foc.OmegaPU = Stru_Encoder.OmegaPU;
Stru_Sincos = SinCos_Cal(Stru_Foc.Elec_Theta);
```

对应文件：

```text
Project/code/4_ctrl_s/MC_FOC.c
Project/code/2_mcu_s/mcu_init.c
Project/code/4_ctrl_h/MC_FOC.h
Project/code/5_user_h/Set_FOC.h
Project/code/5_user_h/Set_MODE.h
```

### SPI 读角方式

LXQS 的 SPI 配置：

```c
SSP_ConfigClk(7, 4); /* Fapb = 48MHz, sclk = 1MHz */
SSP_ConfigRunMode(SSP_FRAME_SPI, SSP_CPO_1, SSP_CPHA_1, SSP_DAT_LENGTH_16);
SSP_DisableMasterAutoControlCS();
```

读角函数：

```c
uint32_t SPI_KTH7801_Read(uint32_t cmd)
{
    uint32_t temp = SPI_Transmit(cmd);
    return temp;
}
```

`SPI_Transmit()` 内部是阻塞等待：

```c
while (SSP_GetBusyFlag());
while (!SSP_GetTFEFlag());
SSP_SendData(Data);
while (!SSP_GetRNEFlag());
return SSP_GetData();
```

因此 LXQS 的读角方式本质上是“高频任务里阻塞式 SPI 读 16 位角度”。这说明它能作为读角流程参考，但不能直接证明当前项目可以在 20 kHz 快环中无代价读取 MA600。

### 机械角到电角度

LXQS 的角度转换函数：

```c
void GetEncoderAngle(Struct_Encoder* hEncoder, int16_t AngleIn)
{
    hEncoder->ThetaM = AngleIn + hEncoder->Offset_ThetaM;
    hEncoder->ThetaE = hEncoder->ThetaM * hEncoder->Pairs;

    hEncoder->SinCosSig = SinCos_Cal(hEncoder->ThetaE);
    temp = (hEncoder->SinCosSig.Sin * hEncoder->Sincos.Cos -
            hEncoder->SinCosSig.Cos * hEncoder->Sincos.Sin) >> 15;

    hEncoder->Omega = PI_Controller(&hEncoder->pi, temp);
    hEncoder->OmegaPU = (hEncoder->Omega * Stru_AlgorPara.OmegaPLL2SpdPUQ15 >> 8);
    hEncoder->ThetaEPLL += hEncoder->Omega;
    hEncoder->Sincos = SinCos_Cal(hEncoder->ThetaEPLL);
}
```

这个函数假设：

```text
AngleIn 是机械角
ThetaM = 机械角 + 机械零点偏移
ThetaE = ThetaM * 电机极对数
```

也就是说，LXQS 的 `Stru_Foc.Elec_Theta` 是一个缓存后的电角度变量，后续 Park / RevPark / SVPWM 使用这个缓存值。

### 当前项目不能直接照抄的地方

当前项目的特殊点：

```text
电机极对数 = 4
MA600 侧边空心磁环极对数 = 4
实测机械转一圈时 MA600 角度变化 4 次
```

因此当前 MA600 读到的不是“单圈机械角”，而更接近“磁场电角度”。在这种情况下，不能直接照抄 LXQS 的：

```c
ThetaE = ThetaM * Pairs;
```

否则会把已经是电角度的 MA600 角度再乘一次 `4`，导致 FOC 使用的电角度快 4 倍。

当前项目建议：

```text
MA600 raw          = 原始磁场角
angle_raw         = 未滤波原始角
angle_elec        = raw 经过方向和零点修正后的 FOC 电角度
angle_mech        = 如需丝杠位置，再从多圈累计角中除以磁环极对数
```

### 缓存电角度是否有问题

缓存电角度本身没有问题，而且是推荐做法。问题不在“缓存”，而在：

- 缓存的角度是否过期。
- 缓存的是机械角还是电角度。
- 是否重复乘了极对数。
- 是否记录了方向和零点修正。
- 调试显示是否又额外读了一次 SPI。

当前项目建议采用：

```text
Board_MA600:
  只负责 SPI 读 raw，并保存最近一次 raw 缓存。

Motor:
  从 Board 取得 raw 快照。
  计算 angle_elec。
  保存 angle_raw / angle_elec / angle_filt / angle_age。

Debug:
  只读取 Motor 或 Board 的缓存值。
  不主动触发 SPI 读角。
```

后续如果 MA600 读取频率低于电流环频率，可以参考 LXQS 的 PLL 思路：

```text
低频读取实际角度
用角度误差估算速度 Omega
在每个快环中用 Omega 预测 ThetaEPLL
Park 使用预测后的电角度
```

但这一步要在 SPI 耗时、电机最高转速、角度延迟都测过以后再做。

## 总体硬件配置

| 项目 | LXQS 参考工程配置 | 对当前工程的意义 |
| --- | --- | --- |
| MCU 主频 | `MCU_CLK = 64000000ul` | 与当前 AC5 下 `SystemCoreClock = 64000000` 一致 |
| PWM 频率 | `EPWM_FREQ = 15000` | 参考工程跑 15kHz，当前测试按 20kHz 更激进 |
| PWM 模式 | 中心对齐、对称、三对互补、三相独立 | FOC 推荐结构，可参考 |
| 采样方式 | `Config_Shunt_Mode = Double_Shunt` | 参考工程是双电阻；当前板子按三相低边采样调试，不可直接照搬 |
| ADC 参考 | `CONFIG_LDO = ADCREF_VCC`，`HW_ADC_REF = 5.0` | 参考工程使用 VCC 作为 ADC 参考 |
| 相电流增益 | `HW_GAIN_IPHASE = 10.0` | 当前板子要按实际 PGA 增益和采样电阻重新计算 |
| 相电流采样电阻 | `HW_RSHUNT_IPHASE = 0.02` | 当前板子已按原理图/实测记录，不要直接用 0.02R |
| 母线电流采样电阻 | `HW_RSHUNT_IBUS = 0.05` | 只作参考 |
| 母线电压分压 | `HW_VBUS_DIVIDER = 2.0 / 12.0` | 当前板子如果有 VBUS ADC，要按当前原理图重算 |
| 硬件过流 | `HARDOVCUR_PROTECT_ENABLE = 0` | 参考工程虽然配置 ACMP 链路，但宏里默认关闭硬件过流 |

## 初始化顺序

参考工程 `System_Init()` 的主要顺序如下：

```text
SysClock_Init()
CGC->RSTM = 0x1
__disable_irq()
DelayTime_ms(POWERON_DELAY_TIME)
WDT_Restart()
GPIO_Config()
ADCLDO_Init()
EPWM_R2_Init()
ADC_Init()
PGA0_Init()
PGA1_Init()
PGA2_Init()
DAC_Init()
ACMP1_Init()                 仅 HARDOVCUR_PROTECT_ENABLE=1 时
SPI_Master_Mode()
HWDIV_Init()
UART0_Init() 或 RTT 配置       按通信宏选择
ADC_TGSAMP_CONFIG()
DelayTime_ms(100)
SysTick_Init()
```

这个顺序的思路是：

1. 先让系统时钟稳定，并确认 P02 复位/普通口相关配置。
2. 先配置 GPIO 和 ADC 参考，再配置 EPWM/ADC/PGA。
3. EPWM 初始化后先保持输出关闭。
4. ADC/PGA 和触发关系准备好以后，再开中断。
5. 主函数最后调用 `Enable_INT()`，使 ADC/ACMP/SysTick 中断真正工作。

当前工程可借鉴这个顺序，但建议继续保持“默认不真实励磁”的安全策略：先启动 EPWM 计数器、确认寄存器，再逐步打开 `POEN`、mask、brake。

## EPWM 配置

### 频率计算

参考工程：

```c
#define MCU_CLK     (64000000ul)
#define EPWM_FREQ   (15000)
#define EPWM_PERIOD (MCU_CLK / EPWM_FREQ / 2)
```

中心对齐 PWM 频率公式：

```text
PWM频率 = EPWM时钟 / (2 * PERIOD)
```

所以参考工程约为：

```text
PERIOD = 64000000 / 15000 / 2 = 2133
```

当前工程测试 20kHz 时：

```text
PERIOD = 64000000 / 20000 / 2 = 1600 = 0x640
50% duty = 800 = 0x320
```

用户当前已观察到：

```text
EPWM->PERIOD[0/2/4] = 0x640
EPWM->CMPDAT[0/2/4] = 0x320
EPWM->CON2          = 0x00000015
EPWM->POEN          = 0x3F
EPWM->BRKCTL        = 0
EPWM->MASK          = 0
```

这说明当前 EPWM 外设本体已经在按 20kHz 配置运行。

### 运行模式

参考工程 `EPWM_R2_Init()` 使用：

```c
EPWM_ConfigRunMode(EPWM_COUNT_UP_DOWN |
                   EPWM_OCU_SYMMETRIC |
                   EPWM_WFG_COMPLEMENTARYK |
                   EPWM_OC_INDEPENDENT);
```

含义：

| 配置 | 含义 |
| --- | --- |
| `EPWM_COUNT_UP_DOWN` | 中心对齐计数 |
| `EPWM_OCU_SYMMETRIC` | 对称 PWM |
| `EPWM_WFG_COMPLEMENTARYK` | EPWM0/1、2/3、4/5 三对互补 |
| `EPWM_OC_INDEPENDENT` | U/V/W 三相 duty 独立 |

这与当前 FOC 目标一致。FOC 不应使用让三相共用同一组比较值的 group 模式。

### 通道使用

参考工程按三对互补使用：

```text
EPWM0/1 -> U 相互补
EPWM2/3 -> V 相互补
EPWM4/5 -> W 相互补
```

写周期和占空比时只写主通道：

```text
PERIOD/CMPDAT: EPWM0、EPWM2、EPWM4
输出使能/mask/brake: EPWM0~EPWM5 六路都要考虑
```

参考工程初始化时：

```c
EPWM_ConfigChannelClk(EPWM0, EPWM_CLK_DIV_1);
EPWM_ConfigChannelClk(EPWM2, EPWM_CLK_DIV_1);
EPWM_ConfigChannelClk(EPWM4, EPWM_CLK_DIV_1);

EPWM_ConfigChannelPeriod(EPWM0, EPWM_PERIOD);
EPWM_ConfigChannelPeriod(EPWM2, EPWM_PERIOD);
EPWM_ConfigChannelPeriod(EPWM4, EPWM_PERIOD);

EPWM_ConfigChannelSymDuty(EPWM0, EPWM_HALFPERIOD);
EPWM_ConfigChannelSymDuty(EPWM2, EPWM_HALFPERIOD);
EPWM_ConfigChannelSymDuty(EPWM4, EPWM_HALFPERIOD);
```

### 死区

参考工程：

```c
#define EPWM_Tus (64)
#define EPWM_DT  (0.5)
EPWM_EnableDeadZone(0x3F, (uint32_t)(EPWM_DT * EPWM_Tus));
```

即死区计数：

```text
0.5us * 64tick/us = 32 tick
```

当前工程之前配置过 64 tick，若 EPWM 时钟为 64MHz，可理解为约 1us。参考工程的死区更短，当前阶段用 1us 更偏保守，适合先保护功率级；后面可以根据驱动器、MOS 管和实际波形再缩短。

### 输出使能、mask 和 brake

参考工程有三个层次：

```c
EPWM_EnableOutput(EPWM_CH_0_MSK | EPWM_CH_1_MSK |
                  EPWM_CH_2_MSK | EPWM_CH_3_MSK |
                  EPWM_CH_4_MSK | EPWM_CH_5_MSK);

Bridge_Output_Off();  // EPWM->MASK = 0x00003F00
Bridge_Output_On();   // EPWM->MASK = 0x00000000
```

`mcu_port.h` 中的宏：

```c
#define Bridge_Output_Off() \
  { EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY; EPWM->MASK = 0x00003F00; EPWM->LOCK = 0x0; }

#define Bridge_Output_On() \
  { EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY; EPWM->MASK = 0x00000000; EPWM->LOCK = 0x0; }
```

也就是说：参考工程初始化阶段虽然使能了 EPWM 输出通道，但立刻用 `MASK` 把六路输出关住。真正允许运行时再解除 mask。

当前工程已经采用类似安全策略，这一点是正确的。

### POREMAP 端口重映射

参考工程定义：

```c
#define UH (0x00)
#define UL (0x01)
#define VH (0x02)
#define VL (0x03)
#define WH (0x04)
#define WL (0x05)

#define IO_EPWM5 (WL)
#define IO_EPWM4 (WH)
#define IO_EPWM3 (VL)
#define IO_EPWM2 (VH)
#define IO_EPWM1 (UL)
#define IO_EPWM0 (UH)

#define REG_POREMAP_VALUE (((uint32_t)0xAA << 24) | ...)
```

按这些默认宏计算，写入值为：

```text
EPWM->POREMAP = 0xAA543210
```

其中高字节 `0xAA` 是开启/确认重映射写入的关键。当前工程实测：

```text
EPWM->POREMAP = 0x00543210
```

高字节不是 `0xAA`，说明当前没有真正启用 POREMAP 重映射。按手册理解，不启用重映射时默认就是 `EPWM0<-IPG0` 到 `EPWM5<-IPG5`。

对当前工程的建议：

- 如果只是验证 EPWM 外设本体，当前不启用重映射是清晰的。
- 如果 OUT_U/V/W 仍没有输出，LXQS 工程的 `0xAA543210` 是一个值得对比验证的点。
- 验证时不要直接改相序，先只尝试“同样的 identity remap 但高字节带 0xAA”，即 `0xAA543210`。
- 由于手册提到 ADC 触发源和重映射前后的信号关系，后续正式 FOC 中如果启用 POREMAP，需要重新确认 ADC 触发点是否仍对应预期相位。

### P10-P15 EPWM 复用

参考工程显式配置：

```c
GPIO_PinAFOutConfig(P10CFG, IO_OUTCFG_P10_EPWM0);
GPIO_PinAFOutConfig(P11CFG, IO_OUTCFG_P11_EPWM1);
GPIO_PinAFOutConfig(P12CFG, IO_OUTCFG_P12_EPWM2);
GPIO_PinAFOutConfig(P13CFG, IO_OUTCFG_P13_EPWM3);
GPIO_PinAFOutConfig(P14CFG, IO_OUTCFG_P14_EPWM4);
GPIO_PinAFOutConfig(P15CFG, IO_OUTCFG_P15_EPWM5);

GPIO_Init(PORT1, PIN0, OUTPUT);
GPIO_Init(PORT1, PIN1, OUTPUT);
GPIO_Init(PORT1, PIN2, OUTPUT);
GPIO_Init(PORT1, PIN3, OUTPUT);
GPIO_Init(PORT1, PIN4, OUTPUT);
GPIO_Init(PORT1, PIN5, OUTPUT);
```

这说明 LXQS 工程把 EPWM 输出映射到普通 IO 口 P10-P15。当前板子使用集成 3P3N 功率输出，是否必须同时配置 P10-P15 需要结合当前芯片封装和手册确认。

当前调试价值：

- 如果芯片封装上 P10-P15 可探测，可先看这些脚是否有普通 EPWM 方波。
- 如果 P10-P15 有波形而 OUT_U/V/W 没波形，问题更可能在 3P3N 功率级使能/供电/保护链路。
- 如果 P10-P15 也没波形，要继续查 EPWM GPIO 复用、POREMAP、计数器和输出使能。

## ADC/PGA 配置

### 双电阻通道

参考工程 `Config_Shunt_Mode = Double_Shunt`，使用两个相电流采样通道：

```c
#define ADC_DATA_CHA (ADC_CH_2)
#define ADC_SCAN_CHA (ADC_CH_2_MSK)

#define ADC_DATA_CHB (ADC_CH_3)
#define ADC_SCAN_CHB (ADC_CH_3_MSK)
```

参考工程的双电阻采样顺序宏：

```c
#define IP_UV (0)
#define IP_UW (1)
#define IP_VU (2)
#define IP_VW (3)
#define IP_WU (4)
#define IP_WV (5)
#define IP_SAMP_CH (IP_UV)
```

当前板子已经按三相低边采样链路调试，之前软件触发下四个 ADC 都有数据，所以不要直接改成 LXQS 的双电阻结构。LXQS 的价值主要是参考“EPWM 触发 ADC、ADC 中断跑快环”的节拍。

### ADC 初始化

参考工程 `ADC_Init()`：

```c
CGC_PER13PeriphClockCmd(CGC_PER13Periph_ADCEN, ENABLE);
ADC_ConfigRunMode(ADC_MODE_HIGH,
                  ADC_CONVERT_CONTINUOUS,
                  ADC_CLK_DIV_1,
                  25);
ADC_ConfigChannelSwitchMode(ADC_SWITCH_HARDWARE);
ADC_DisableChargeAndDischarge();
ADC_ConfigVREF(ADC_VREFP_VDD);      // CONFIG_LDO == ADCREF_VCC 时
ADC_CHANNEL_CONFIG();
ADC_Start();
```

要点：

- 使用高速 ADC 模式。
- ADC 时钟不分频。
- 通道切换用硬件自动切换。
- ADC 参考选 VDD/VCC。
- 最终由 EPWM 硬件触发相关通道。

当前工程已经切到 EPWM CMP0 触发 ADC，同步采样链路已验证。后续继续参考 LXQS 的重点不是“是否切硬件触发”，而是采样点、ADC 中断入口、PGA/ACMP 保护和快环节拍如何组织。

### PGA 初始化

参考工程启用三个 PGA：

| PGA | 输入脚 | 增益 | 参考 | 模式 |
| --- | --- | --- | --- | --- |
| PGA0 | P00/P01 | `PGA_GAIN_10` | `PGA0BG` | 差分 |
| PGA1 | P24/P25 | `PGA_GAIN_10` | `VrefHalf` | 差分 |
| PGA2 | P26/P27 | `PGA_GAIN_10` | `VrefHalf` | 差分 |

当前板子三相采样也会用到 PGA0/1/2，但要注意参考工程里 PGA0 的参考配置是 `PGA0BG`，PGA1/2 是 `VrefHalf`。当前工程之前按零电流约 2048 count 的思路调试，说明 `VrefHalf` 对偏置理解更直观。正式确定前，需要以当前原理图和实测零漂为准。

## EPWM 触发 ADC

参考工程在 `EPWM_R2_Init()` 中配置两个比较触发点：

```c
T_CPMTG0 = (uint32_t)(EPWM_CPMTG * EPWM_Tus);
if (T_CPMTG0 < 12)  T_CPMTG0 = 12;
if (T_CPMTG0 > 128) T_CPMTG0 = 128;

EPWM_ConfigCompareTriger(EPWM_CMPTG_0,
                         EPWM_CMPTG_FALLING,
                         EPWM_CMPTG_EPWM0,
                         T_CPMTG0);

EPWM_ConfigCompareTriger(EPWM_CMPTG_1,
                         EPWM_CMPTG_RISING,
                         EPWM_CMPTG_EPWM0,
                         EPWM_HALFPERIOD);
```

然后 `ADC_TGSAMP_CONFIG()` 里把 ADC 通道挂到触发源：

```c
ADC_EnableEPWMCmp0TriggerChannel(ADC_SCAN_CHA | ADC_SCAN_CHB);
ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP0);

ADC_EnableEPWMCmp1TriggerChannel(ADC_SCAN_VBUS);
ADC_EnableHardwareTrigger(ADC_TG_EPWM_CMP1);

EPWM_DisableIntAccompanyWithLoad();
```

中断启用：

```c
ADC_EnableChannelInt(ADC_INTER_CH);
NVIC_EnableIRQ(ADC_IRQn);
NVIC_SetPriority(ADC_IRQn, 1);
```

`ADC_INTER_CH` 的定义是取 `ADC_SCAN_CHA` 和 `ADC_SCAN_CHB` 中较大的通道 mask，用最后完成的采样通道作为一次电流采样完成的中断标志。

当前工程后续迁移建议：

1. 继续先用软件触发确认三相零漂和电流换算。
2. EPWM 连续输出验证通过后，再配置 `EPWM_CMPTG_0`。
3. 用 `ADC_TG_EPWM_CMP0` 触发 U/V/W 电流通道。
4. 在 `ADC_IRQHandler()` 中只做快环必需工作：读 ADC、换算电流、角度、电流环、更新 PWM。
5. 慢任务、打印、FreeMASTER/RTT 观测不要放进 ADC 中断。

## 中断结构

参考工程 `ADC_IRQHandler()`：

```c
if (ADC->MIS & ADC_INTER_CH)
{
    ADC_ClearIntFlag_CHA();
    FOC_Task_HighFre();
    Fault_Check_FOCTask();
    FOC_User_Control();
}
else
{
    ADC_ClearAllInt_Flag();
}
```

参考工程 `SysTick_Handler()`：

```text
FOC_Task_MidFre()
Flag_1ms_Intr = 1
```

主循环中：

```text
System_Control()
Task1ms_Main()
WDT_Restart()
```

可借鉴的节拍：

| 层级 | 触发 | 放什么 |
| --- | --- | --- |
| 快环 | ADC 完成中断 | 电流采样、FOC、电流 PI、PWM 更新、快速故障检查 |
| 中频 | SysTick 1ms | 速度估计、状态机中频逻辑 |
| 主循环/1ms任务 | while 循环 | 通信、参数、慢速保护、调试变量 |

## ACMP/DAC/硬件保护

参考工程虽然宏里 `HARDOVCUR_PROTECT_ENABLE = 0`，但代码结构已经放好了硬件过流链路：

```text
PGA 输出 -> ACMP 正端
DAC 阈值 -> ACMP 负端
ACMP 事件 -> EPWM fault brake
ACMP IRQ -> 关输出、置故障状态
```

`ACMP1_Init()` 中：

```c
ACMP_ConfigPositive(ACMP1, ACMP_POSSEL_1PGA0O);
ACMP_ConfigNegative(ACMP1, ACMP_NEGSEL_DAC_O);
ACMPOut_Enable(ACMP1);
ACMP_Start(ACMP1);
ACMP_Filter_Config(ACMP1, ENABLE, ACMP_NGCLK_65_TSYS);
ACMP_Polarity_Config(ACMP1, ACMP_POL_Pos);
ACMP_EnableHYS(ACMP1, ACMP_HYS_POS, ACMP_HYS_S_00);
ACMP_ConfigEventAndIntMode(ACMP1, ACMP_EVENT_INT_RISING);
ACMP_EnableEventOut(ACMP1);
```

`EPWM_R2_Init()` 中：

```c
EPWM_EnableFaultBrake(EPWM_BRK_ACMP1EE);
EPWM_ConfigFaultBrakeLevel(EPWM_CH_0_MSK | EPWM_CH_2_MSK | EPWM_CH_4_MSK, 0);
EPWM_ConfigFaultBrakeLevel(EPWM_CH_1_MSK | EPWM_CH_3_MSK | EPWM_CH_5_MSK, 0);
EPWM_AllBrakeEnable();
EPWM_ConfigBrakeMode(EPWM_BRK_SUSPEND);
```

对当前工程的建议：

- 调 PWM 波形阶段可以暂时不启用硬件过流 brake，避免干扰定位。
- 进入真实电机开环前，至少要有软件过流、欠压和状态机超时。
- 进入闭环或较高电流前，应补 ACMP 或 ADC 比较器硬件关断链路。

## SPI 配置

参考工程 `SPI_Master_Mode()`：

```c
CGC_PER12PeriphClockCmd(CGC_PER12Periph_SPI, ENABLE);
SSP_ConfigClk(7, 4);  // 注释写 Fapb=48MHz, sclk=1MHz
SSP_ConfigRunMode(SSP_FRAME_SPI,
                  SSP_CPO_1,
                  SSP_CPHA_1,
                  SSP_DAT_LENGTH_16);
SSP_EnableMasterMode();
SSP_DisableMasterAutoControlCS();

GPIO_PinAFOutConfig(P03CFG, IO_OUTCFG_P03_SCK);
GPIO_Init(PORT0, PIN3, OUTPUT);

GPIO_PinAFOutConfig(P04CFG, IO_OUTCFG_P04_MISO);
GPIO_Init(PORT0, PIN4, INPUT);

GPIO_PinAFOutConfig(P05CFG, IO_OUTCFG_P05_MOSI);
GPIO_Init(PORT0, PIN5, OUTPUT);

CGC->RSTM = 1;

GPIO_PinAFOutConfig(P23CFG, IO_OUTCFG_P23_GPIO);
GPIO_Init(PORT2, PIN3, OUTPUT);

SSP_Start();
```

它使用：

| 信号 | 引脚 |
| --- | --- |
| SCK | P03 |
| MISO | P04 |
| MOSI | P05 |
| CS | P23 GPIO |

当前板子的 MA600 已经能读出角度，说明当前 SPI 基础链路已经通过。LXQS 的 SPI 价值主要是确认：

- CMS32 的 SSP 配置风格。
- 手动控制 CS 是常见做法。
- 16bit SPI 事务可用于磁编码器类器件。

但当前板子的 CS/CNS 引脚与 LXQS 不一定一样，不要把 `P23` 直接照搬。

## GPIO 中的 P16 拉高

参考工程 `GPIO_Config()`：

```c
GPIO_Init(PORT1, PIN6, OUTPUT);
GPIO_PinAFInConfig(P16CFG, IO_OUTCFG_P16_GPIO);
PORT_SetBit(PORT1, PIN6);
```

这是一个值得关注的板级动作。它可能是某种外部使能、模式选择、电源控制或测试口，也可能只是 LXQS 板子的特定控制脚。

对当前工程的影响：

- 不能直接认定当前板子也需要 P16 拉高。
- 但如果当前 OUT_U/V/W 一直没有波形，而 EPWM 寄存器确认正常，应回到原理图检查是否存在类似 `EN`、`DRV_EN`、`VCC_EN`、`SLEEP`、`MODE`、`CNS`、`RST` 之类需要 MCU 拉高/拉低的板级控制脚。
- 如果当前原理图里没有 P16 对应的使能网络，就不要为了“像参考工程”而添加。

## 与当前 CMS32FOCAC6 的关键对比

| 项目 | LXQS 参考工程 | 当前 CMS32FOCAC6 状态 |
| --- | --- | --- |
| 编译器 | 原厂工程风格，通常更贴近 AC5 | 当前已切回 AC5，`SystemCoreClock=64MHz` 正常 |
| PWM 频率 | 15kHz | 当前示波器测试用 20kHz |
| PERIOD | 约 2133 | 1600，已实测寄存器正确 |
| duty 表达 | 控制层常用 Q15，底层换算到 CMPDAT | 当前底层直接用 0..PERIOD 的 CMPDAT |
| 采样 | 双电阻 CH2/CH3 | 当前三相电流链路已调软件 ADC |
| ADC 触发 | EPWM CMP0/CMP1 硬件触发 | 当前已使用 EPWM CMP0 触发 ADC |
| PWM 输出 | P10-P15 AF + POREMAP | 当前主要看集成 OUT_U/V/W |
| POREMAP | `0xAA543210` | 当前按已打通 PWM 的配置记录，后续改动仍需对照寄存器和示波器 |
| 输出开关 | `MASK` 宏控制 | 当前也用 mask/brake/POEN 分层 |
| 硬件保护 | ACMP/DAC/EPWM brake 结构存在，宏默认关 | 当前后续要补硬件保护 |
| SPI | P03/P04/P05 + P23 CS | 当前 MA600 已能读角度，CS 按当前板子 |

## 当前已验证后的参考用法

当前项目已经在示波器上确认三相 `20 kHz` PWM 输出，LXQS 不再作为“找不到 PWM”的阻塞排查清单，而是作为后续 FOC 外设配置模板。继续参考它时按下面原则处理：

1. EPWM 保持中心对齐、三对互补、死区、mask/brake/POEN 分层。
2. ADC 使用 EPWM CMP0 触发，并在 ADC 中断里进入快环。
3. PGA 和 ACMP 保护参考 LXQS 的配置顺序，但阈值、通道和极性必须按当前原理图与实测电流链路重新确认。
4. SPI/编码器只参考“高频任务读角和缓存电角度”的思想，不复制 LXQS 的片选脚和 `ThetaE = ThetaM * Pairs`。
5. 任意修改 `POREMAP`、输出极性、死区、刹车链路后，都要用示波器和寄存器重新确认三相输出。

## 结论

LXQS 工程最值得借鉴的不是它的全部参数，而是它的初始化结构：

```text
系统时钟
GPIO/板级使能
ADCLDO/ADC参考
EPWM中心对齐互补
ADC/PGA
EPWM触发ADC
mask/brake保护
ADC中断快环
SysTick/主循环慢任务
```

当前工程可以继续保留现有最小分层：`Board` 负责 EPWM/ADC/PGA/MA600/保护，`Motor` 负责状态机、开环、FOC 和闭环控制。后续进入 FOC 时，建议从 LXQS 的 `EPWM_R2_Init()`、`ADC_TGSAMP_CONFIG()`、`ADC_IRQHandler()` 三处抽取思想，但按当前板子的三相采样和 MA600 角度反馈重写，不直接复制双电阻配置。
