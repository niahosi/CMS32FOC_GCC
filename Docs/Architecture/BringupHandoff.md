# FOC Bring-up Handoff

本文记录当前调试状态，方便后续继续接手。

## 当前硬件/控制基线

- MCU/PWM: CMS32M65xx, center-aligned PWM 20 kHz.
- `PWM_PERIOD = 1600`, `PWM_DUTY_50 = 800`.
- 死区当前 `PWM_DEADTIME_TICKS = 32`.
- Duty guard 当前 `PWM_DUTY_MIN = 32`, `PWM_DUTY_MAX = 1568`.
- 电机和磁环都是 4 极对：
  - `MOT_POLE_PAIRS = 4`
  - `MOT_SENSOR_POLE_PAIRS = 4`
  - `MOT_SENSOR_ELEC = 1`
- MA600 是侧轴安装，不是轴端同轴安装。
- 当前电角零位配置为 `MOT_ELEC_ZERO = -13478`.

## 电流采样状态

- 低电感电机约 19 uH，电流纹波和采样相位很敏感。
- 当前策略是三电阻动态窗口采样，T1/T2/T3 选择采样窗口。
- 即使 T1 三相都有效，也配置为只采两相并重构第三相：
  - `CS_USE_2PHASE_IN_ALL_WINDOW = 1`
  - `CS_ALL_WINDOW_PAIR = CS_PAIR_UV`
- 当前采样点参数：
  - `CS_OPEN_SETTLE_TICK = PWM_DEADTIME_TICKS + 4`
  - `CS_TAIL_MARGIN_TICK = 60`
  - `CS_MULTI_EN = 1`
  - `CS_MULTI_DELTA_TICK = 4`
  - `CS_MULTI_SPREAD_LIMIT_CNT = 40`
  - `CS_HIGH_VF_SINGLE_EN = 1`
  - `CS_HIGH_VF_SINGLE_VOLTAGE = 660`
- 现场观察：电流环 `iq_ref` 在 10-80 范围内比较稳定，`iq` 能贴近 `iq_ref`，`id` 不大。
- 用户观察 `iq_ref=80` 时数字电源约 80 mA，但这只是当前工况近似线性；`iq` 仍是内部电流单位/ADC 缩放，不等同于严格 mA。

## MA600 当前状态

- MA600 侧轴 BCT 已验证比较好：
  - `MOT_ENCODER_SIDE_BCT = 180`
  - `MOT_ENCODER_SIDE_ETX = 0`
  - `MOT_ENCODER_SIDE_ETY = 1`
- 这个配置只在上电时写 MA600 RAM 寄存器，不写 NVM。
- 反推 `k ~= 3.31`，仍低于手册提醒的 BCT 200 温漂警戒区。
- 32 点残差表工具已实现，但当前建议保持 32 点表全 0。
- 原因：没有独立参考角度，也不能保证机械恒速。用当前电机自身 VF/闭环数据做 32 点表，容易把真实速度波动写进 MA600 校准。
- `cms32foc` 主固件不再链接 MA600 在线 BCT/CORR/NVM 调参 service。
- MA600 主驱动只保留 init、angle/speed read、cache 和必要 register read/write；上电按 `TuneConfig.h` 写 BCT/ET RAM 默认补偿，不写 NVM。

### MA600 在线调参变量（冻结）

以下变量保留在 `Firmware/FrozenDiagnostics/Board/foc_ma600_diag.c/.h` 作为冻结诊断源码参考，但不再链接进 `cms32foc` 主固件；Ozone 主固件 watch 中不可用。该目录中的源码是“找结果后写死”的历史工具，需要恢复时再同步头文件接口并手动加入 CMake。

BCT 在线写 RAM：

```c
g_ma600_bct_cmd
g_ma600_etx_cmd
g_ma600_ety_cmd
g_ma600_bct_apply
g_ma600_bct_write
g_ma600_bct_read
g_ma600_et_write
g_ma600_et_read
g_ma600_config_ok
```

32 点表在线写 RAM：

```c
g_ma600_corr_index
g_ma600_corr_value
g_ma600_corr_apply
g_ma600_corr_table_cmd[32]
g_ma600_corr_table_read[32]
g_ma600_corr_table_apply
g_ma600_corr_ok
g_ma600_corr_write_count
g_ma600_corr_table_fail_index
g_ma600_corr_table_fail_write
g_ma600_corr_table_fail_read
```

NVM 手动触发：

```c
g_ma600_nvm_block_cmd       // 1 = store CORR table block 1, 0 = store config block 0
g_ma600_nvm_store_apply
g_ma600_nvm_status
g_ma600_nvm_ok
```

当前不要存 NVM，尤其不要存 32 点表。若以后确实要存，先停机，再确认 RAM 表有效。

## MA600 角度跳变

- 曾观察到电机周期性被踢一下，`encoder_reject_count` 在跳变时增加。
- 读 4 字节 angle+speed 帧时，坏点周期从约 23 个锯齿变成约 46 个锯齿，说明问题和 MA600 SPI 读帧长度/时序相关。
- 当前默认：
  - `MOT_ENCODER_MTSP_SPEED_EN = 0`
  - `MOT_ENCODER_FAST_READ_SPEED_FRAME = 0`
  - 快环读 16-bit angle frame。
- 已在 MA600 快读结束后等待 `SSP_GetBusyFlag()` 清零再拉高 CS。
- 已加坏角拒绝：
  - `MOT_ENCODER_MAX_STEP_RAW = 8192`
  - `MOT_ANGLE_MAX_AGE = 4`
  - 监控 `encoder_raw_step`, `encoder_reject_step`, `encoder_reject_raw`, `encoder_reject_count`
  - 碰到坏角会立即重读一次 16-bit angle frame；监控 `encoder_retry_count` 和 `encoder_retry_accept_count` 判断是否为单帧 SPI 坏读。

## 当前控制模式注意点

- `control_mode = 1/2` 是干净主线，由 `motor_control_c.c` 分发到 `motor_control_current.c` 实现 Current/Speed。
- `control_mode = 3` 是 VF 应急开环，实现在 `motor_control_vf.c`。
- 当前 VF 默认 `OL_SPEED_REF = 50`，`OL_VF_VOLTAGE = 80`；这两个值由 `g_motor_cmd.open_loop_speed_ref/vf_voltage` 上电初始化。
- `control_mode = 4/5` 的 Align/EncoderVoltage 已冻结到 `Firmware/FrozenDiagnostics/MotorControl/motor_control_diag_frozen.c`，并从 `cms32foc` 主固件移出；命令这些模式会进入 unsupported fault/安全态。
- VF 运行中的开环角只能在明确切入 VF 或初始化时重置；慢环状态重入不得重置开环角。观察 `g_motor_watch.open_loop_reset_count`，VF 稳定运行时它不应持续增加。
- `run_vf_fast_loop()` 当前实际调用：

```c
MotorControl_InternalApplyVoltageVector(mc, 0, mc->command.vf_voltage, theta);
```

也就是 VF 开环电压放在 q 轴参数，不是之前讨论过的 d 轴参数。接手时必须按代码当前状态判断，不要按旧口头结论假设。

## Watch 分工

- `g_motor_watch` 是唯一主固件 watch：`state/fault/mode`、电流 dq、Current/Speed refs、速度反馈、PI 输出、`vd/vq`、`voltage_theta`、VF `open_loop_theta/open_loop_reset_count/vf_voltage`、duty、PWM/check、少量 encoder 基础状态。
- `g_motor_diag_watch` 已从 `cms32foc` 移除；Current/Speed/VF 都不依赖诊断 watch。

- 电流环：
  - `CTRL_CUR_KP = 4`
  - `CTRL_CUR_KI = 1`
  - `CTRL_CUR_PI_SHIFT = 3`
  - `CTRL_CUR_REF_RAMP_STEP = 2`
  - `CTRL_CUR_SAFE_LIMIT = 400`
  - `CTRL_CUR_REF_LIMIT = 1000`
- 速度环当前配置：
  - `CTRL_SPD_EST_HZ = 1000`
  - `CTRL_SPD_FB_SOURCE = CTRL_SPD_FB_SOURCE_DIFF`
  - `CTRL_SPD_KP = 32`
  - `CTRL_SPD_KI = 3`
  - `CTRL_SPD_ERR_SHIFT = 10`
  - `CTRL_SPD_FILTER_SHIFT = 2`
  - `CTRL_SPD_CMD_DEADBAND_RPM = 5`
  - `CTRL_SPD_IQ_LIMIT = 80`
  - `CTRL_SPD_REF_LIMIT_RPM = 5000`
  - `CTRL_SPD_IQ_SLEW_STEP = 4`

## 速度环下一步建议

1. 先确认电流环稳定范围：
   - 继续用 current mode 测 `iq_ref = 10..80`
   - 看 `iq` 是否贴近、`id` 是否小、母线电流是否可控
2. 再低速进入 speed mode：
   - 先用很小速度给定，避免直接打满 `iq_limit`
   - 默认 `g_motor_cmd.iq_limit = 30`，初调时也可以临时降到 20
   - 观察 `speed_ref_rpm`, `speed_fb_rpm`, `speed_err_rpm`, `speed_iq_cmd`, `iq`
3. 判断方向：
   - 若速度给定为正，`speed_fb_rpm` 也应为正
   - 若方向相反，先不要调 PI，优先修 `MOT_SENSOR_DIR` 或速度符号
4. 当前速度 PI 节点：
   - 默认 `speed_ki = 3`，用于提供 3000 rpm 以上需要的稳态扭矩
   - 已验证 3000 rpm 以内平均速度稳定；3300 rpm 可作为当前高转节点
   - 若 `speed_iq_cmd` 正负打满，先降低 `speed_kp/speed_ki` 或减小 `iq_limit`，不要继续放开扭矩上限
5. 速度闭环不稳时先看角度质量：
   - `encoder_reject_count` 是否增加
   - `encoder_raw_step` 是否有周期性尖峰
   - 若有角度坏点，先回到 MA600 SPI 时序，不要硬调速度 PI

## 不建议现在做的事

- 不要现在生成或写入 MA600 32 点 NVM 残差表。
- 不要仅凭电机自身锯齿波“不直”去做 32 点表；没有参考角度会混入机械速度波动。
- 不要同时开启 `ETX` 和 `ETY`。
- 不要在角度还会周期跳变时强行提高速度环增益。

## 最后一次构建状态

最近一次构建通过：

```text
cmake --build --preset gcc-debug --target cms32foc
Flash image: 26988 bytes, 41.2%
RAM total: 4640 bytes, 56.6%
```
