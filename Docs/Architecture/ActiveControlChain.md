# Active FOC Firmware State

本文是当前 `cms32foc` 主固件的状态源。旧 C++ 控制层、Align、EncoderVoltage、MA600 在线调参、旧采样实验方案都已经从默认构建链路中移出；需要参考时看 `Firmware/FrozenDiagnostics/` 或 git 历史，不要按旧文档推断当前程序。

## 构建边界

当前 CMake 只启用 `C ASM`，主固件链接关系如下：

```text
cms32foc
  -> cms32_motor_control_c
       -> motor_control_c.c
       -> motor_control_current.c
       -> motor_control_encoder.c
       -> motor_control_output.c
       -> motor_control_vf.c
       -> motor_control_watch.c
       -> cms32_bsp
       -> cms32_foc_algorithm
```

默认 `cmake --build --preset gcc-debug` 只构建主固件。以下诊断固件保留但不进默认 all build：

```text
cmake --build --preset gcc-debug --target cms32_board_watch_test
cmake --build --preset gcc-debug --target cms32_startup_smoke_test
```

冻结源码目录：

| 路径 | 内容 |
| --- | --- |
| `Firmware/FrozenDiagnostics/MotorControl/motor_control_diag_frozen.c` | 旧 Align、EncoderVoltage、诊断 watch |
| `Firmware/FrozenDiagnostics/Board/foc_ma600_diag.c/.h` | MA600 在线 BCT/CORR/NVM 调参 |
| `Firmware/FrozenDiagnostics/Board/foc_ma600_aux_frozen.c` | 旧裸读角和 NVM store 辅助流程 |

## 启动调用链

`Firmware/App/main.c` 是唯一主入口；`g_motor_cmd` 和 `g_motor_watch` 在 `MotorControl.h` 中声明、在 C 控制层实现中定义：

```text
main()
  -> bsp_init()
       -> clock_init()
       -> gpio_init()              // P16 驱动使能脚先拉低
       -> ma600_init()             // SPI、BCT/ET RAM 补偿、MTSP
       -> bsp_update_angle()       // 先读一次 MA600 缓存
       -> curr_init()              // ADC LDO、PGA、ADC、采样状态
       -> pwm_init()               // EPWM、死区、刹车、输出关闭
       -> curr_calib(1024)         // PWM 关闭状态静态零漂
  -> MotorControl_Init()
       -> 初始化状态机、PI、watch 内部状态
       -> pwm_off()
  -> MotorControl_UpdateWatch()
  -> bsp_start_adc_sync()
       -> curr_sync_init()         // 连续 ADC + EPWM CMP 触发 + ADC IRQ
  -> while(1)
       -> MotorControl_ApplyCommand(&g_motor_cmd)
       -> MotorControl_RunSlowLoop()
       -> MotorControl_UpdateWatch(&g_motor_watch)
```

ADC 中断快环：

```text
ADC_IRQHandler()
  -> MotorControl_FastLoopFromAdcIrq()
       -> bsp_adc_irq()
            -> curr_irq()
                 -> 未完成双点采样: return 0
                 -> 完成有效采样: 更新 curr_u/v/w/sum, return 1
       -> 若 enable 且 state=CLOSED_LOOP:
            mode=1 Current -> MotorControl_CurrentRunFastLoop(0)
            mode=2 Speed   -> MotorControl_CurrentRunFastLoop(1)
            mode=3 VF      -> MotorControlVf_RunFastLoop()
            mode=4/5       -> 不运行；慢环进入 unsupported fault
```

主循环不直接算 FOC，不直接更新 PWM。PWM 只在 ADC 有效采样后由快环更新。

## 控制模式

| `control_mode` | 状态 | 快环行为 |
| ---: | --- | --- |
| 0 | Off | PWM 安全关闭 |
| 1 | Current | MA600 电角度 + Clarke/Park + d/q 电流 PI + SVPWM；同时更新速度观察值 |
| 2 | Speed | 速度 PI 生成 `iq_ref`，再进入同一套电流环 |
| 3 | VF | 开环角 + `vf_voltage` 输出 q 轴电压；仍保留电流安全检查和编码器观察 |
| 4 | Frozen Align | 主固件不支持，进入 `MC_FAULT_UNSUPPORTED_MODE` |
| 5 | Frozen EncoderVoltage | 主固件不支持，进入 `MC_FAULT_UNSUPPORTED_MODE` |

VF 开环角只允许在明确切入 VF 时重置。运行中如果 `open_loop_reset_count` 持续增加，就是状态重入或命令切换问题。

## 板级外设参数

### PWM 和功率级

配置来自 `Firmware/Board/Config/BoardConfig.h`：

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `PWM_FREQ_HZ` | 20000 | 中心对齐 PWM 20 kHz |
| `PWM_PERIOD` | 1600 | 64 MHz / (2 * 20 kHz) |
| `PWM_DUTY_50` | 800 | 50% duty |
| `PWM_DUTY_MIN/MAX` | 32 / 1568 | duty guard |
| `PWM_DEADTIME_TICKS` | 32 | 死区 tick，约 0.5 us |
| `PWM_ADC_TRIGGER_TICK_DEFAULT` | 650 | 默认 ADC 触发中心 |
| `PWM_SVPWM_V_LIMIT` | 886 | 当前 duty guard 下的 SVPWM 电压 count 上限 |

`foc_pwm.c` 使用 EPWM0/2/4 做 U/V/W 主通道，互补输出由 EPWM 硬件生成。P16 是驱动使能脚，初始化和 `pwm_off()` 时保持低电平。

### 电流采样和 ADC/PGA

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `ADC_VREF_V` | 3.6 V | ADC 参考 |
| `ADC_COUNTS` | 4096 | 12-bit ADC |
| `SHUNT_OHM` | 0.08 ohm | 采样电阻 |
| `PGA_GAIN` | 2 | 三路 PGA 增益 |
| `BOARD_CURRENT_OFFSET_SAMPLES` | 1024 | 上电零漂平均次数 |
| `CS_MULTI_EN` | 1 | 双点采样 |
| `CS_MULTI_DELTA_TICK` | 4 | 双点 A/B 偏移 |
| `CS_MULTI_SPREAD_LIMIT_CNT` | 40 | 双点差值过大则拒收 |
| `CS_OPEN_SETTLE_TICK` | `PWM_DEADTIME_TICKS + 4` | 下管打开后的稳定等待 |
| `CS_TAIL_MARGIN_TICK` | 60 | 采样点离窗口尾部最小余量 |
| `CS_USE_2PHASE_IN_ALL_WINDOW` | 1 | 三相都可采时仍只采两相 |
| `CS_ALL_WINDOW_PAIR` | `CS_PAIR_UV` | 全窗口默认 pair |
| `CS_HIGH_VF_SINGLE_EN` | 1 | 高 VF 电压时允许单点采样 |
| `CS_HIGH_VF_SINGLE_VOLTAGE` | 660 | 单点切换阈值 |

采样流程在 `foc_curr.c` 内部完成：

```text
pwm_set_duty()
  -> curr_sync_timing()
       -> trigger_update()
            -> window_select()
                 -> ti_window_select()       // 根据三相 duty 找共同低边窗口
                 -> window_apply_pair()      // 选择 UV/UW/VW 和 A/B 触发 tick
            -> trigger_pair()                // 配置 ADC scan channel 和中断通道
ADC IRQ
  -> curr_irq()
       -> sample_pair()                      // 读当前 pair 的 ADC
       -> sample_resolve()
            -> pair_sample_in_range()
            -> reconstruct()                 // 第三相 = -两相和
            -> apply_phys()
            -> map_logic()                   // 相序和符号映射
```

`curr_u/v/w/sum()` 返回的是控制逻辑相电流，单位是 ADC count 缩放后的内部值，不是直接 mA。

### MA600 编码器

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `MOT_POLE_PAIRS` | 4 | 电机极对数 |
| `MOT_SENSOR_POLE_PAIRS` | 4 | 磁环极对数 |
| `MOT_SENSOR_ELEC` | 1 | raw 到电角度倍数 |
| `MOT_SENSOR_DIR` | 1 | 当前已验证方向 |
| `MOT_ELEC_ZERO` | -13478 | 固化电角零位 |
| `MOT_ENCODER_SIDE_BCT_EN` | 1 | 上电写 RAM BCT/ET |
| `MOT_ENCODER_SIDE_BCT` | 180 | 侧轴 BCT 强度 |
| `MOT_ENCODER_SIDE_ETX/ETY` | 0 / 1 | 只削弱 Y 轴 |
| `MOT_ENCODER_MTSP_SPEED_EN` | 0 | MTSP 不输出 speed |
| `MOT_ENCODER_FAST_READ_SPEED_FRAME` | 0 | 快环读 16-bit angle 帧 |
| `MOT_ENCODER_MAX_STEP_RAW` | 8192 | 单拍 raw 最大可信步进 |
| `MOT_ANGLE_MAX_AGE` | 4 | 允许保持上一角度的最大 age |

`ma600_init()` 配置 SPI 后会写 BCT/ET RAM 默认补偿，并回读确认，失败最多重试 3 次。不写 NVM。在线 BCT/CORR/NVM 入口已经冻结，不参与主固件。

电角度计算：

```text
MOT_SENSOR_DIR = 1:
encoder_elec = MOT_ELEC_ZERO + elec_zero_trim + raw * MOT_SENSOR_ELEC

MOT_SENSOR_DIR = -1:
encoder_elec = MOT_ELEC_ZERO + elec_zero_trim - raw * MOT_SENSOR_ELEC
```

`encoder_elec` 是 16-bit 周期角，Ozone 画图出现 0..65535 回绕锯齿是正常现象。判断坏角看 `encoder_raw_step`、`encoder_reject_count`、`encoder_retry_count`，不要只看锯齿是否刚好落到 0。
高 iq 调试时如果 `fault_reason=4` 且 `encoder_age` 增加，说明 MA600 连续坏角超过 `MOT_ANGLE_MAX_AGE`；主 watch 已保留 `encoder_reject_step/reject_count/retry_count/retry_accept_count` 用于判断是单帧毛刺还是连续掉角。

## 控制参数

### 电流环

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `CTRL_FAST_LOOP_DIV` | 2 | 20 kHz ADC 有效采样下，电流环约 10 kHz |
| `CTRL_CUR_KP` | 4 | d/q 电流 PI 比例 |
| `CTRL_CUR_KI` | 1 | d/q 电流 PI 积分 |
| `CTRL_CUR_PI_SHIFT` | 3 | 定点 PI 右移 |
| `CTRL_CUR_REF_RAMP_STEP` | 2 | 电流给定斜坡 |
| `CTRL_CUR_REF_LIMIT` | 1000 | 电流给定最大值 |
| `CTRL_CUR_SAFE_LIMIT` | 400 | 运行电流保护阈值 |
| `CTRL_CUR_OVER_LIMIT` | 4 | 连续过流计数 |
| `CTRL_CUR_V_LIMIT` | `PWM_SVPWM_V_LIMIT` | 输出电压限幅 |

### 速度环

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `CTRL_SPD_EST_HZ` | 1000 | 速度估算/速度 PI 频率 |
| `CTRL_SPD_FB_SOURCE` | `CTRL_SPD_FB_SOURCE_DIFF` | 默认用 raw 差分测速 |
| `CTRL_SPD_FILTER_SHIFT` | 2 | 差分速度低通 |
| `CTRL_SPD_KP` | 32 | rpm 误差输入的比例，等效 0.03125 iq/rpm |
| `CTRL_SPD_KI` | 3 | 积分用于消除稳态速度误差 |
| `CTRL_SPD_ERR_SHIFT` | 10 | 速度 PI 右移 |
| `CTRL_SPD_CMD_DEADBAND_RPM` | 5 | 小速度给定归零 |
| `CTRL_SPD_REF_RAMP_RPM_PER_S` | 2000 | 速度内部目标斜坡 |
| `CTRL_SPD_IQ_LIMIT` | 80 | 默认速度环扭矩限制，约 0.44 A |
| `CTRL_SPD_REF_LIMIT_RPM` | 5000 | `speed_ref_rpm` 最大命令 |
| `CTRL_SPD_IQ_SLEW_STEP` | 4 | 速度环输出对称斜率 |

`speed_ref_rpm` 非 0 时覆盖 `speed_ref`。速度环不会直接吃阶跃命令，而是先生成 `speed_ref_active_rpm` 斜坡目标，再由速度 PI 直接生成 q 轴电流给定。当前版本去掉了前馈、禁反向制动和非对称收油逻辑，低惯量空心杯电机先用更快的 1 kHz 差分测速、更低的 PI 增益和对称 iq 斜率闭环。

差分测速每个 1 kHz 速度周期更新一次，并用一阶低通平滑。速度 PI 直接使用该一次低通后的 `speed_fb_rpm`。`CTRL_SPD_DIFF_SPIKE_RPM` 只用于拒绝接近半圈的 raw 毛刺，不再把正常高速运行误判为速度尖峰。

### VF 应急开环

当前 `TuneConfig.h` 默认：

| 参数 | 当前值 | 说明 |
| --- | ---: | --- |
| `OL_SPEED_REF` | 50 | 默认开环速度给定，sensor counts/s |
| `OL_VF_VOLTAGE` | 80 | 默认 q 轴开环电压，SVPWM count |
| `OL_TIMEOUT_MS` | 30000 | 默认超时 |
| `OL_SPEED_TO_THETA_STEP` | 131 | 速度到角步进定点系数 |
| `OL_SPEED_TO_THETA_SHIFT` | 8 | 速度到角步进右移 |

VF 调用：

```c
MotorControl_InternalApplyVoltageVector(mc, 0, mc->command.vf_voltage, theta);
```

也就是开环电压放在 q 轴参数。VF 不使用 MA600 角度产生电压，但会定期读取 MA600 更新 watch 和速度观察。

## 主要函数说明

### App 和控制层

| 函数 | 调用位置 | 职责 |
| --- | --- | --- |
| `MotorControl_Init()` | `main()` 初始化 | 清状态、清 PI、初始化 VF、PWM 安全关闭 |
| `MotorControl_ApplyCommand()` | 主循环 | 从 `g_motor_cmd` 复制命令，限幅，处理模式切换 reset |
| `MotorControl_RunSlowLoop()` | 主循环 | 更新慢速安全检查，决定 `IDLE/CLOSED_LOOP/FAULT` |
| `MotorControl_FastLoopFromAdcIrq()` | `ADC_IRQHandler()` | 等待有效电流采样后分发 Current/Speed/VF |
| `MotorControl_UpdateWatch()` | 主循环 | 填充 `g_motor_watch` |
| `MotorControl_CurrentRunFastLoop()` | 控制层内部 | Current/Speed 共用 dq 电流环；Current 只更新速度观察，Speed 额外运行速度 PI |
| `MotorControl_InternalUpdateEncoderAngle()` | 控制层内部 | 快读 MA600、坏角拒绝、即时重读 |
| `MotorControl_InternalUpdateEncoderSpeed()` | 控制层内部 | raw 差分或 MA600 speed 转速度反馈 |
| `update_speed_loop()` | `motor_control_current.c` 内部 | rpm 速度 PI，输出 `speed_iq_ref` |
| `MotorControl_InternalApplyVoltageVector()` | Current/Speed/VF | dq 电压限幅、反 Park、SVPWM、PWM 使能 |
| `MotorControlVf_RunFastLoop()` | ADC 快环 mode 3 | VF 开环角推进和 q 轴电压输出 |

当前 `MotorControlCState s_mc` 只由 `motor_control_c.c` 拥有；拆分模块都通过 `MotorControlCState*` 操作状态，为后续迁移到 C++ 类成员函数保留边界。

慢环故障判定顺序：电流检查失败时报 `MC_FAULT_CURRENT`；Current/Speed 下编码器不可用时报 `MC_FAULT_ENCODER`；这样 Ozone 中 `fault_reason` 不会把 MA600 掉线误标成电流故障。

MA600 SPI 当前显式配置为 4 MHz：`MA600_SSP_CLK_M=7, MA600_SSP_CLK_N=2`。高电流调试时如果 `encoder_raw_step/encoder_reject_step` 出现远大于正常速度的跳变，说明角度链路仍有毛刺。`MOT_ENCODER_MAX_STEP_RAW=1024` 用于拒绝运行中的单帧坏角；首帧角度仍会接受，速度估算用 `CTRL_SPD_STARTUP_BLANK_SAMPLES` 跳过初始不连续窗口。

### BSP

| 函数 | 职责 |
| --- | --- |
| `bsp_init()` | 板级初始化总入口，按安全顺序初始化 MA600、电流采样、PWM 和零漂 |
| `bsp_start_adc_sync()` | 在控制层初始化后启动 PWM 触发 ADC 和中断 |
| `bsp_adc_irq()` | ADC IRQ 中转到 `curr_irq()` |
| `bsp_update_angle()` | 普通路径更新 MA600 缓存 |
| `bsp_update_angle_fast()` | 快环短超时更新 MA600；按宏选择 16-bit 或 32-bit 帧 |
| `bsp_angle_raw/speed_raw/ok/age()` | 返回 MA600 缓存观察值 |

### PWM

| 函数 | 职责 |
| --- | --- |
| `pwm_init()` | 配置 EPWM0/2/4、互补输出、死区、ADC trigger、刹车和引脚 |
| `pwm_set_duty()` | 写三相 duty，并通知电流采样层重算下一拍窗口 |
| `pwm_set_adc_trigger()` | 设置单中心 ADC 触发 tick |
| `pwm_set_adc_triggers()` | 设置双点 ADC 触发 tick A/B |
| `pwm_off()` | 关闭 P16、打开软件刹车、禁用 EPWM 输出、duty 回 50% |
| `pwm_enable()` | 解除刹车并使能输出；前提是状态机已经确认安全 |
| `pwm_is_off_safe()` | 判断输出关闭和刹车状态是否安全 |
| `pwm_snapshot()` | 给主 watch 或诊断固件读取 duty/output/brake |

### 电流采样

| 函数 | 职责 |
| --- | --- |
| `curr_init()` | 初始化 ADC LDO、PGA、ADC 和内部采样状态 |
| `curr_calib()` | PWM 关闭时采集静态零漂 |
| `curr_sync_init()` | 配置连续 ADC、EPWM 触发、中断并启动 |
| `curr_sync_timing()` | PWM duty 改变后重选采样窗口 |
| `curr_set_vf_voltage()` | VF 电压变化时切换高调制单点/双点策略 |
| `curr_irq()` | ADC IRQ 中读取样本、双点解析、重构三相电流 |
| `curr_u/v/w/sum()` | 返回控制使用的逻辑三相电流 |
| `curr_raw_adc_u/v/w()` | 返回未扣零漂 ADC 原始码值，主要给 BoardWatch |
| `curr_sync_count()` | 返回有效同步采样计数 |

### MA600

| 函数 | 职责 |
| --- | --- |
| `ma600_init()` | 配置 SPI 引脚/SSP、写 BCT/ET RAM、设置 MTSP |
| `ma600_write_reg/read_reg()` | 配置阶段 RAM/SFR 寄存器访问，不用于 ADC 快环 |
| `ma600_update()` | 普通路径读取 16-bit angle 并更新缓存 |
| `ma600_update_fast()` | ADC 快环短超时读取 16-bit angle |
| `ma600_update_speed_fast()` | 可选 32-bit angle+speed 帧读取；默认未启用 |
| `ma600_raw/speed_raw/ok/age()` | 返回缓存状态 |

### FOC 数学

| 函数 | 职责 |
| --- | --- |
| `foc_clarke_3phase()` | 三相电流到 alpha/beta |
| `foc_park()` | alpha/beta 到 d/q |
| `foc_pi_update()` | 定点 PI 更新，带积分和输出限幅 |
| `foc_limit_dq()` | 限制 d/q 电压矢量幅值 |
| `foc_inv_park()` | d/q 电压到 alpha/beta |
| `foc_svpwm()` | alpha/beta 电压到三相 duty |

## Watch 和调试入口

`g_motor_cmd` 是唯一命令入口，`g_motor_watch` 是唯一主固件 watch。`g_motor_diag_watch` 已移除。

常用 watch 字段：

| 字段 | 用途 |
| --- | --- |
| `state/fault_reason/control_mode/enable` | 状态机和故障 |
| `iu_cnt/iv_cnt/iw_cnt/i_sum` | 逻辑三相电流 |
| `id/iq/id_ref/iq_ref` | dq 电流投影和给定 |
| `speed_ref_rpm/speed_fb_rpm/speed_err_rpm/speed_iq_cmd` | 速度环 |
| `speed_iq_target` | 速度环斜率限制前的 q 轴电流目标 |
| `speed_pi_output/speed_iq_ff/speed_pi_integral` | 速度 PI 与前馈拆分观察 |
| `speed_reset_count/safe_state_count/speed_loop_count/speed_deadband_count` | 诊断闭环是否反复重入、进入安全态或速度给定死区清零 |
| `vd/vq/voltage_theta/v_limited` | 输出电压和限幅 |
| `duty_u/duty_v/duty_w` | PWM 输出 |
| `encoder_raw/encoder_elec/encoder_pos/encoder_ok/encoder_age` | 编码器基础状态 |
| `open_loop_theta/open_loop_ticks/open_loop_reset_count/vf_voltage` | VF 开环 |
| `check.ma600_ok/current_ok/pwm_off_safe/ready_closed_loop` | 慢环安全条件 |

- 调 Current：`enable=1, control_mode=1, id_ref=0, iq_ref=10..80`。
- 调 Speed：先确认 Current 稳定，再 `control_mode=2`，小 `speed_ref_rpm` 起步，观察 `speed_iq_cmd` 是否长期顶住 `iq_limit`。
- 调 VF：`control_mode=3`，当前默认 `open_loop_speed_ref=50, vf_voltage=80`；VF 运行中 `open_loop_reset_count` 不应增加。
