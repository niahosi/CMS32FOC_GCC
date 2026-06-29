# 14 PWM输出打通记录

记录日期：2026-05-27

> 2026-05-29 状态说明：本文是 PWM 打通过程的阶段记录，不代表当前主程序仍处在连续示波器测试模式。当前源码已经删除 `PWM_SCOPE_CONTINUOUS_TEST` / `Board_StartScopePwm()` 这类早期测试路径，主线回到 `Board_Init()`、`Motor_TASK()`、`Motor_FastLoop()`；当前闭环只计算不输出。

## 当前已验证状态

用户已确认：烧录当前工程后，示波器可以看到 `20kHz`、约 `12.4V` 的 PWM 波形。

进一步确认：`OUT_U`、`OUT_V`、`OUT_W` 三相都已经能看到 `20kHz` PWM。

这说明当前至少已经打通了以下链路：

- MCU 系统时钟已经按 `64MHz` 工作。
- EPWM 外设时钟已打开。
- `EPWM0/2/4` 计数器已启动。
- `EPWM0~5` 六路输出使能已打开。
- software brake 已解除。
- EPWM mask 已清零。
- `OUT_U/OUT_V/OUT_W` 三相对应的功率输出链路都已经能输出母线电压级 PWM。
- 示波器测点、地线和时间档已经能正确观察到电机 PWM，而不是误看 Buck 电源开关节点。

## 当时的代码状态

当时工程处在示波器连续 PWM 测试模式：

```c
#define PWM_SCOPE_CONTINUOUS_TEST 1U
```

因此 `main()` 初始化后会直接调用：

```c
Board_StartScopePwm();
```

并且主循环中不会执行 `Motor_TASK()`。这意味着当前 PWM 波形不是电机状态机产生的，而是 `Board_StartScopePwm()` 用于示波器验证的固定 50% 占空比输出。

当前 PWM 参数：

| 项目 | 当前值 | 含义 |
| --- | --- | --- |
| 系统主频 | `64MHz` | 已在 AC5 下确认 |
| PWM 频率 | `20kHz` | 电机控制常用起步频率 |
| PWM 周期寄存器 | `1600 / 0x640` | 中心对齐：`64MHz / (2 * 1600) = 20kHz` |
| 50% duty | `800 / 0x320` | 半周期比较值 |
| 死区 | `64 ticks` | 约 `1us`，后续仍需实测确认 |
| 输出通道 | `EPWM0~5` | 三组互补输出 |
| 主通道 | `EPWM0/2/4` | U/V/W 三相 duty 写入通道 |

## 可能让 PWM 开始真正输出的关键改动

因为之前是逐步试验，无法百分百认定“唯一根因”。从当前代码和前面的现象看，最可能的关键点是下面几项组合生效：

1. 启动 EPWM 计数器

```c
EPWM_Start(EPWM_CH_0_MSK | EPWM_CH_2_MSK | EPWM_CH_4_MSK);
```

之前曾观察到 `CON2 = 0`，说明计数器没启动；后来 `CON2 = 0x15`，说明 `EPWM0/2/4` 已启动。

2. 使能六路输出

```c
EPWM_EnableOutput(EPWM_CH_0_MSK |
                  EPWM_CH_1_MSK |
                  EPWM_CH_2_MSK |
                  EPWM_CH_3_MSK |
                  EPWM_CH_4_MSK |
                  EPWM_CH_5_MSK);
```

互补 PWM 是 `0/1`、`2/3`、`4/5` 成对工作的。只开主通道可能不足以让功率级按预期工作，因此当前按六路一起使能。

3. 解除 software brake

```c
EPWM_DisableSoftwareBrake();
EPWM_ClearBrakeIntFlag();
EPWM_ClearBrake();
```

之前曾观察到 `BRKCTL = 0x1000`，说明软件刹车还压着；当前测试中刹车已经解除。

4. 清 EPWM mask

```c
EPWM->LOCK = EPWM_LOCK_P1B_WRITE_KEY;
EPWM->MASK = 0x00000000;
EPWM->LOCK = 0x0;
```

如果 mask 没清，即使计数器在跑、输出使能打开，也可能看不到真实输出。

5. 复刻 LXQS/原厂的普通 EPWM 端口配置

当前代码里也加入了：

```c
GPIO_PinAFOutConfig(P10CFG, IO_OUTCFG_P10_EPWM0);
GPIO_PinAFOutConfig(P11CFG, IO_OUTCFG_P11_EPWM1);
GPIO_PinAFOutConfig(P12CFG, IO_OUTCFG_P12_EPWM2);
GPIO_PinAFOutConfig(P13CFG, IO_OUTCFG_P13_EPWM3);
GPIO_PinAFOutConfig(P14CFG, IO_OUTCFG_P14_EPWM4);
GPIO_PinAFOutConfig(P15CFG, IO_OUTCFG_P15_EPWM5);

EPWM->POREMAP = 0xAA543210;
```

注意：当前板子的正式电机输出仍然是 `OUT_U/OUT_V/OUT_W`，不是把 `P10~P15` 当作最终电机输出脚。这里记录为“当前成功状态的一部分”，后续可以再做控制变量实验，确认它是否真的是必需项。

## 当前还不能下的结论

现在能发波，但还不能直接说明：

- 死区一定正确，只能说寄存器配置和当前波形没有立刻阻止输出。
- 三相互补关系一定正确，还需要分别看 `U/V/W` 与对应低边行为。
- 当前配置就是最终 FOC 配置，因为现在还没有加入 PWM 触发 ADC、硬件过流刹车、FOC 快环。
- `P10~P15/POREMAP` 一定是 3P3N 输出必需条件。它们可能只是当前成功实验中的伴随配置，后续可单独验证。

## 历史复现步骤

1. 使用 AC5 编译当前工程。
2. 临时恢复连续 PWM 测试路径。
3. 确认所有输出放行条件只在该测试中使用，测试后立即删除或关闭。
4. 烧录并运行，不进入在线单步卡住程序。
5. 12V 限流电源给 `VM` 供电。
6. 示波器地夹接板子 `GND/PGND`。
7. 探头尖接 `OUT_U/OUT_V/OUT_W` 之一。
8. 示波器时间档先用 `10us/div` 或 `20us/div`。
9. 期望看到约 `20kHz`、接近母线电压幅值的 PWM。

建议同时观察寄存器：

| 寄存器 | 期望 |
| --- | --- |
| `EPWM->CON2` | `CNTEN0/2/4` 置位，常见为 `0x15` |
| `EPWM->POEN` | `0x3F` |
| `EPWM->BRKCTL` | software brake 未生效 |
| `EPWM->MASK` | `0x00000000` |
| `EPWM->PERIOD[0/2/4]` | `0x640` |
| `EPWM->CMPDAT[0/2/4]` | `0x320` |
| `EPWM->DTCTL` | 当前约 `0x04010040` |
| `EPWM->POREMAP` | 当前成功状态为 `0xAA543210` |

## 下一步建议

下一步不要急着让电机转。建议按顺序做：

1. 用固定 50% 继续观察三相波形是否稳定，有无异常毛刺、丢波或电源限流。
2. 如果有条件，用双通道或差分方式看互补输出和死区。
3. 再恢复由 `Motor_TASK()` 控制 PWM，而不是一直使用 `Board_StartScopePwm()`。
4. 加入“短时输出超时”和“输出请求撤销后立即关断”的安全逻辑。
5. 之后再做 PWM 触发 ADC 和硬件过流保护。
