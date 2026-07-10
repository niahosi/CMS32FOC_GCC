# FOC 程序阅读路线图

本文给第一次接手当前 `cms32foc` 程序的人用。目标不是一次读完所有文件，而是按“电机怎么从命令变成 PWM 输出”的路径，把主线读通。

当前主固件只需要先理解三条线：

```text
1. main.c 主循环：命令、慢环、watch
2. ADC_IRQHandler 快环：电流采样后运行 FOC
3. Board 层：PWM / 电流采样 / MA600 编码器
```

旧 C++ 控制层、Align、EncoderVoltage、MA600 在线调参都已经冻结，不作为当前阅读入口。

## 第 1 步：从 main.c 看程序骨架

先看：

```text
Firmware/App/main.c
Firmware/MotorControl/Inc/MotorControl.h
```

重点看三个全局变量/入口：

```c
volatile MotorControlCommand_t g_motor_cmd;
volatile MotorControlWatch_t g_motor_watch;
void ADC_IRQHandler(void);
```

`g_motor_cmd` 是 Ozone 改命令的地方。  
`g_motor_watch` 是 Ozone 看状态的地方。  
`ADC_IRQHandler()` 是真正控制快环入口。

`g_motor_cmd` 和 `g_motor_watch` 在 `MotorControl.h` 中声明，在 C 控制层实现中定义。`main.c` 只保留启动顺序和调度逻辑。

启动顺序：

```text
bsp_init()
MotorControl_Init()
MotorControl_UpdateWatch()
bsp_start_adc_sync()
while(1) {
    MotorControl_ApplyCommand()
    MotorControl_RunSlowLoop()
    MotorControl_UpdateWatch()
}
```

先记住一句话：主循环不直接算 FOC，主循环只是复制命令、做慢速状态检查、刷新 watch。

## 第 2 步：理解命令和 watch

再看：

```text
Firmware/MotorControl/Inc/MotorControl.h
```

先只读两个结构：

```c
MotorControlCommand_t
MotorControlWatch_t
```

`MotorControlCommand_t` 是输入：

| 字段 | 先理解成 |
| --- | --- |
| `enable` | 总开关 |
| `control_mode` | 1 Current, 2 Speed, 3 VF |
| `id_ref/iq_ref` | 电流环给定 |
| `speed_ref_rpm` | 速度环给定 |
| `iq_limit` | 速度环最大扭矩 |
| `vf_voltage` | VF 开环电压 |
| `open_loop_speed_ref` | VF 开环速度 |
| `elec_zero_trim` | 临时电角零位修正 |
| `voltage_theta_offset` | 动态相位补偿调试量 |

`MotorControlWatch_t` 是输出：

| 字段 | 先理解成 |
| --- | --- |
| `state/fault_reason` | 状态机有没有允许运行 |
| `iu_cnt/iv_cnt/iw_cnt` | 三相电流采样结果 |
| `id/iq` | FOC 看到的 d/q 电流 |
| `vd/vq` | 电流 PI 输出的 d/q 电压 |
| `duty_u/v/w` | 最终 PWM duty |
| `encoder_raw/encoder_elec` | MA600 原始角和电角度 |
| `speed_fb_rpm` | 速度反馈 |
| `open_loop_theta` | VF 生成的开环角 |

读程序时，先用这些 watch 字段去反推函数在做什么，会比从数学公式开始轻松很多。

## 第 3 步：读控制层主文件

然后看：

```text
Firmware/MotorControl/C/motor_control_c.c
Firmware/MotorControl/C/motor_control_current.c
Firmware/MotorControl/C/motor_control_encoder.c
Firmware/MotorControl/C/motor_control_output.c
Firmware/MotorControl/C/motor_control_watch.c
```

`motor_control_c.c` 现在是控制层外壳：拥有唯一的 `s_mc` 状态，负责公共 API、命令复制、慢环状态机和 ADC 快环分发。其他文件都通过 `MotorControlCState*` 操作状态，为后续迁移 C++ 做准备。

建议按这个顺序读，不要从第一行一路硬啃：

### 3.1 `MotorControl_ApplyCommand()`

它做三件事：

```text
1. 从 volatile g_motor_cmd 复制命令到内部 s_mc.command
2. 对命令做限幅
3. 检测模式切换，必要时 reset 电流环/速度环/VF
```

关键点：ADC 中断不会直接读 `g_motor_cmd`，只读复制后的内部缓存。

### 3.2 `MotorControl_RunSlowLoop()`

它决定当前能不能进入 `MC_STATE_CLOSED_LOOP`。

Current/Speed 需要：

```text
current_ok = 1
encoder_ok = 1 或者编码器还没初始化
```

VF 不依赖编码器角度产生电压，但仍会做电流安全检查。

### 3.3 `MotorControl_FastLoopFromAdcIrq()`

这是 ADC 中断里的控制分发：

```text
bsp_adc_irq() -> curr_irq()
sample_ready = 0: 返回
sample_ready = 1:
    mode 1 -> MotorControl_CurrentRunFastLoop(0)
    mode 2 -> MotorControl_CurrentRunFastLoop(1)
    mode 3 -> MotorControlVf_RunFastLoop()
```

这就是当前程序最重要的分叉。

### 3.4 `MotorControl_CurrentRunFastLoop()`

这是 Current 和 Speed 共用的 FOC 主线。
它已经放在 `motor_control_current.c` 中。

简化流程：

```text
读 curr_u/v/w
检查电流安全
按 CTRL_FAST_LOOP_DIV 分频
读取 MA600 电角度
Current/Speed 都定期更新 speed_fb_rpm 观察值
Speed 模式时额外运行速度环，得到 iq_ref
Clarke: 三相 -> alpha/beta
Park: alpha/beta -> d/q
PI: id/iq 误差 -> vd/vq
限幅 vd/vq
InvPark: vd/vq -> alpha/beta
SVPWM: alpha/beta -> duty_u/v/w
pwm_set_duty()
pwm_enable()
```

如果你只想先读懂闭环，就把这个函数读透。

### 3.5 编码器相关函数

再读 `motor_control_encoder.c` 里的这些：

```c
electrical_from_raw()
update_encoder_angle_state()
accept_encoder_angle()
reject_bad_encoder_angle()
retry_encoder_angle()
update_encoder_speed_state()
```

它们负责：

```text
MA600 raw -> encoder_elec
坏角判断
坏角即时重读
raw 差分测速
rpm watch 换算
```

记住：`encoder_elec` 是 16-bit 周期角，图上锯齿回绕正常。

## 第 4 步：单独读 VF

看：

```text
Firmware/MotorControl/C/motor_control_vf.c
```

这个文件很短，适合建立信心。

核心函数：

```c
MotorControlVf_RunFastLoop()
```

它做：

```text
读电流
检查电流安全
检查 open-loop timeout
按 open_loop_speed_ref 推进 open_loop_theta
用 open_loop_theta 输出 q 轴 vf_voltage
定期读取 MA600，仅用于观察速度/角度
```

VF 的电压角来自程序自己生成的 `open_loop_theta`，不是 MA600。MA600 在 VF 下只是观察转子有没有跟上。

## 第 5 步：读 Board 层

Board 层不要一口气全读。按外设分开。

### 5.1 BSP 总入口

```text
Firmware/Board/Src/foc_bsp.c
Firmware/Board/Inc/foc_bsp.h
```

它只是薄薄的板级门面：

```text
bsp_init()
bsp_adc_irq()
bsp_update_angle_fast()
bsp_angle_raw()
```

控制层只依赖 `bsp_*`，不直接碰寄存器。

### 5.2 PWM

```text
Firmware/Board/Src/foc_pwm.c
Firmware/Board/Inc/foc_pwm.h
```

先读这些函数：

```c
pwm_init()
pwm_set_duty()
pwm_off()
pwm_enable()
pwm_set_adc_triggers()
```

重点理解：

```text
EPWM0/2/4 是 U/V/W 主通道
互补输出由硬件生成
P16 是驱动使能
pwm_set_duty() 后会调用 curr_sync_timing()
```

也就是说，PWM duty 变了，下一拍 ADC 采样窗口也要跟着重算。

### 5.3 电流采样

```text
Firmware/Board/Src/foc_curr.c
Firmware/Board/Inc/foc_curr.h
```

这个文件最长，不要一次全读。按下面顺序：

```text
curr_init()
curr_calib()
curr_sync_init()
curr_sync_timing()
curr_irq()
```

再读内部采样流程：

```text
trigger_update()
window_select()
ti_window_select()
window_apply_pair()
trigger_pair()
sample_pair()
sample_resolve()
reconstruct()
map_logic()
```

你可以把它理解成：

```text
根据当前 duty 找哪两相低边窗口适合采样
设置 ADC 在这个窗口采两次
ADC IRQ 读两相
第三相用 ia + ib + ic = 0 重构
映射成控制层使用的 U/V/W
```

先不用纠结每个 tick 公式，先抓住“窗口选择 -> ADC 触发 -> 双点采样 -> 重构”这条线。

### 5.4 MA600

```text
Firmware/Board/Src/foc_ma600.c
Firmware/Board/Inc/foc_ma600.h
```

先读：

```c
ma600_init()
ma600_update_fast()
ma600_raw()
ma600_ok()
ma600_age()
```

当前主固件：

```text
上电写 BCT/ET RAM 默认补偿
不写 NVM
快环默认读 16-bit angle
32-bit angle+speed 代码保留但默认不用
```

## 第 6 步：读 FOC 数学

最后看：

```text
Firmware/MotorControl/Algorithm/foc_math.c
Firmware/MotorControl/Algorithm/foc_math.h
```

按使用顺序读：

```text
foc_clarke_3phase()
foc_park()
foc_pi_update()
foc_limit_dq()
foc_inv_park()
foc_svpwm()
```

如果你在调电机，先不需要证明所有公式，只需要知道它们在 `MotorControl_CurrentRunFastLoop()` 中串起来以后，对应这条路：

```text
三相电流 -> dq 电流 -> PI 输出 dq 电压 -> 三相 PWM duty
```

## 推荐阅读顺序

第一次完整阅读建议按这个顺序：

```text
1. Firmware/App/main.c
2. Firmware/MotorControl/Inc/MotorControl.h
3. Firmware/MotorControl/C/motor_control_c.c
4. Firmware/MotorControl/C/motor_control_current.c
5. Firmware/MotorControl/C/motor_control_encoder.c
6. Firmware/MotorControl/C/motor_control_output.c
7. Firmware/MotorControl/C/motor_control_watch.c
8. Firmware/MotorControl/C/motor_control_vf.c
9. Firmware/Board/Src/foc_bsp.c
10. Firmware/Board/Src/foc_pwm.c
11. Firmware/Board/Src/foc_curr.c
12. Firmware/Board/Src/foc_ma600.c
13. Firmware/MotorControl/Algorithm/foc_math.c
14. Firmware/Board/Config/BoardConfig.h
15. Firmware/Board/Config/TuneConfig.h
```

其中 `motor_control_current.c`、`motor_control_encoder.c` 和 `foc_curr.c` 最重。读不动的时候先回到 Ozone watch，看某个变量从哪里被写，再顺藤摸瓜。

## 调试时怎么把变量和代码对应起来

| 你在 Ozone 看见 | 去哪里找 |
| --- | --- |
| `state/fault_reason` | `MotorControl_RunSlowLoop()` |
| `encoder_raw/encoder_elec` | `motor_control_encoder.c` 中的 `update_encoder_angle_state()`、`electrical_from_raw()` |
| `encoder_raw_step/reject_count` | `motor_control_encoder.c` 中的 `encoder_raw_plausible()`、`reject_bad_encoder_angle()` |
| `iu_cnt/iv_cnt/iw_cnt` | `curr_irq()`、`map_logic()` |
| `id/iq` | `MotorControl_CurrentRunFastLoop()` 中 Clarke/Park 后 |
| `vd/vq` | `foc_pi_update()` 后 |
| `duty_u/v/w` | `MotorControl_InternalApplyVoltageVector()`、`foc_svpwm()` |
| `speed_fb_rpm` | `motor_control_encoder.c` 中的 `update_encoder_speed_state()` |
| `speed_iq_cmd` | `motor_control_current.c` 中的 `update_speed_loop()` |
| `open_loop_theta` | `motor_control_vf.c` |

## 当前不要先读的东西

这些不是当前主线入口：

```text
Firmware/MotorControl/Src/*.cpp
Firmware/FrozenDiagnostics/**
Reference/**
```

它们有参考价值，但会把你带回旧架构或辅助诊断流程。等 Current/Speed/VF 主线读通之后，再看它们才不会乱。

## 一句话总图

当前程序可以先记成这一句：

```text
Ozone 命令 -> main.c 复制命令 -> 慢环允许运行 -> ADC 有效采样 -> 电流/速度/VF 快环 -> SVPWM duty -> PWM 输出
```

读懂这条链，整个工程就不会再像一团线。
