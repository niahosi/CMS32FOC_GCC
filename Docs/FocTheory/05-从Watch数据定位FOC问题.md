# 从 Ozone 数据定位 FOC 问题

本文从外部理论笔记迁入，并已按当前 GCC 工程路径和参数做同步；若后续代码与本文冲突，以当前源码为准。

当前主线已经删除旧的慢环转发 watch。调试时直接在 Ozone 展开真实状态：

```text
g_motor.runtime
g_motor.check
g_motor.diag
g_motor.encoder
g_motor.speed
g_motor.current
g_motor.output
g_motor.debug
g_screw_axis_watch
```

本文专门讲怎么根据这些 `g_mc_*` 数据定位问题。

原则：

```text
先判断链路断在哪一层，再判断那一层为什么错。
不要一看到不转就调 PI。
不要一看到电流大就改电流环。
不要一看到速度不对就调速度环。
```

当前状态定义在：

```text
Firmware/MotorControl/Abi/motor_control_state.h
Firmware/MotorControl/Types/debug_state.hpp
```

公共结构定义在：

```text
Firmware/MotorControl/Abi/MotorControl.h
```

## 1. 先看状态层

先看这些字段：

```text
enable
control_mode
g_motor.runtime.state
g_motor.runtime.mode
g_motor.runtime.fault
g_motor.runtime.enabled
g_motor.check.current_ok
g_motor.check.ma600_ok
g_motor.check.pwm_off_safe
g_motor.check.ready_closed_loop
g_motor.diag.slow_loop_count
g_motor.diag.fast_loop_count
g_motor.diag.safe_state_count
```

判断：

| 现象 | 意义 | 下一步 |
| --- | --- | --- |
| `g_motor.diag.slow_loop_count` 不动 | 主循环没跑或卡住 | 查 `main()`、中断、硬 fault |
| `g_motor.diag.fast_loop_count` 不动 | 控制快环没有有效执行 | 查 ADC IRQ、PWM trigger、模式状态 |
| `g_motor.runtime.state = MC_STATE_FAULT` | fault | 看 `g_motor.runtime.fault` |
| `g_motor.diag.safe_state_count` 增长 | 反复进安全态 | 查电流、编码器、模式是否不支持 |
| `g_motor.check.ready_closed_loop = 0` | 慢环认为不能进闭环 | 看 `g_motor.check.current_ok` 和 `g_motor.check.ma600_ok` |

当前模式约定：

```text
control_mode = 0: off
control_mode = 1: current
control_mode = 2: speed
control_mode = 3: VF open loop
```

当前故障约定：

```text
fault_reason = 0: none
fault_reason = 1: unsupported mode
fault_reason = 2: current
fault_reason = 3: open-loop timeout
fault_reason = 4: encoder
```

## 2. 再看命令是否真的进入内部

看：

```text
id_ref
iq_ref
speed_ref
speed_ref_rpm
speed_ref_active_rpm
vf_voltage
```

注意：

```text
Current 模式:
  iq_ref 来自命令 iq_ref，但会被 iq_limit 和斜坡限制。

Speed 模式:
  iq_ref 来自速度环输出 speed_iq_cmd。

VF 模式:
  使用 vf_voltage 和 open_loop_theta，不使用编码器角度闭环。
```

如果你写了很大的命令，但 watch 里 `iq_ref` 慢慢变化，这是正常的：

```text
CTRL_CUR_REF_RAMP_STEP = 2
```

每个电流环 tick 只允许变化 2 count。

## 3. 电流采样层

看：

```text
iu_cnt
iv_cnt
iw_cnt
i_sum
id
iq
```

判断表：

| 现象 | 更可能的原因 | 下一步 |
| --- | --- | --- |
| PWM 关闭时 `iu/iv/iw` 大 | 零漂、ADC、PGA、采样通道问题 | 重新校准零漂，查硬件通道 |
| `i_sum` 长期很大 | 三相重构、相序、零漂、采样窗口异常 | 查 `foc_curr.c` 采样 pair 和 current map |
| 小电流时 `id/iq` 噪声很大 | 采样点受开关噪声污染 | 降低电压，查采样窗口和双点差值 |
| duty 变化但电流基本没反应 | 功率级没输出、接线、采样不在正确相 | 查 PWM、驱动使能、相序 |
| 某一相长期偏大 | ADC 通道、PGA、相线、采样电阻或映射问题 | 对比 raw/logic，查板级通道 |

经验顺序：

```text
先确认 iu/iv/iw 可信，再讨论 id/iq。
先确认 i_sum 合理，再讨论 PI 参数。
```

## 4. 编码器和电角度层

看：

```text
encoder_raw
encoder_elec
encoder_raw_step
encoder_delta
encoder_pos
encoder_age
encoder_ok
encoder_reject_count
encoder_retry_count
encoder_retry_accept_count
encoder_hold_count
speed_reject_count
speed_reject_delta
speed_fb_rpm
```

判断表：

| 现象 | 更可能的原因 | 下一步 |
| --- | --- | --- |
| `encoder_raw` 不变 | MA600 没读到、SPI/供电/磁体问题 | 查 MA600 通信和磁体 |
| `encoder_reject_count` 增长 | 单拍 raw 跳变超过阈值 | 查 SPI 毛刺、线束、磁环偏心 |
| `encoder_hold_count` 增长 | 读角失败或坏角保持 | 查 MA600 ok、干扰和供电 |
| `encoder_age` 增长 | 当前使用旧角度 | 超过窗口后会影响闭环 |
| `speed_fb_rpm` 符号反 | 传感器方向不对 | 查 `MOT_SENSOR_DIR` |
| 手转连续但速度偶发跳变 | 速度差分 spike | 看 `speed_reject_count`，位置累计仍看 `encoder_pos` |

电角度错误常见表现：

```text
Current 模式给小 iq_ref 后：
  iq 不按预期上升
  id 明显偏大
  vd/vq 很快饱和
  电机抖动或反转
```

这时优先查：

```text
MOT_SENSOR_DIR
MOT_ELEC_ZERO
elec_zero_trim
voltage_theta_offset
PWM 相序
电流相序
电流符号
```

## 5. 电流环层

看：

```text
id_ref
iq_ref
id
iq
vd
vq
v_limited
duty_u
duty_v
duty_w
```

判断表：

| 现象 | 更可能的原因 | 下一步 |
| --- | --- | --- |
| `iq_ref` 变化，`iq` 基本不动 | 电压没输出、角度错、采样错、PI 太弱 | 先看 `vd/vq/duty` |
| `iq` 方向反 | 角度方向、PWM 相序、电流符号错误 | 查方向和相序，不先调 PI |
| `id` 随 `iq_ref` 明显变化 | 电角度零位偏 | 调 `elec_zero_trim` 验证 |
| `vd/vq` 很大但 `iq` 不跟 | 电压饱和、角度错、功率级问题 | 看 `v_limited/duty` |
| `v_limited = 1` | dq 电压被限幅 | 降低目标，查电压余量 |
| duty 贴近 `32/1568` | 接近 PWM 边界 | 查电压、相位、负载、采样 |
| `iq` 有稳态误差且不饱和 | 积分偏弱或采样偏置 | 适度增 KI，先确认采样 |
| 高频抖动明显 | KP 偏大、采样噪声、死区影响 | 降 KP 或改善采样 |

`v_limited = 1` 是关键分界线：

```text
v_limited = 0:
  PI 还没被电压限制，调 PI 才可能有效。

v_limited = 1:
  PI 输出已经碰墙，继续加参数通常只会更糟。
```

## 6. 速度环层

看：

```text
speed_ref_rpm
speed_ref_active_rpm
speed_fb_rpm
speed_err_rpm
speed_iq_target
speed_iq_cmd
speed_pi_output
speed_pi_integral
iq_ref
iq
v_limited
```

判断表：

| 现象 | 更可能的原因 | 下一步 |
| --- | --- | --- |
| `speed_ref_active_rpm` 慢慢变 | 速度斜坡正常 | 不要误判为命令没生效 |
| `speed_iq_cmd = 0` | 速度目标在死区或速度 PI reset | 看 `speed_deadband_count` |
| `speed_iq_cmd` 到限幅但速度不上去 | 负载大、电流环跟不上、电压饱和 | 看 `iq/v_limited` |
| `speed_fb_rpm` 符号反 | 编码器方向错 | 查 `MOT_SENSOR_DIR` |
| 速度抖但电流环正常 | 速度 KP/KI、速度滤波、机械负载 | 再调速度环 |
| `speed_pi_integral` 很大 | 外环长期误差 | 查是否被 iq_limit 或电压限制 |

速度环调试原则：

```text
只有在 Current 模式下 iq 能稳定跟随 iq_ref 后，才调 Speed 模式。
```

## 7. VF 模式怎么用来定位问题

VF 模式是开环电压旋转：

```text
open_loop_theta += open_loop_speed_ref
vd = 0
vq = vf_voltage
```

它不依赖编码器角度闭环，所以适合验证：

```text
PWM 是否输出
SVPWM duty 是否变化
功率级是否能推动电机
电流采样是否有响应
编码器观察速度是否跟着变化
```

看：

```text
open_loop_theta
open_loop_ticks
open_loop_reset_count
vf_voltage
duty_u/duty_v/duty_w
iu_cnt/iv_cnt/iw_cnt
speed_fb_rpm
```

判断：

| 现象 | 意义 |
| --- | --- |
| `open_loop_theta` 不变 | VF 快环没跑 |
| duty 不变 | SVPWM/PWM 输出链路没生效 |
| duty 变但电流不变 | 功率级、接线、采样可能有问题 |
| 电机动但 `speed_fb_rpm` 不对 | 编码器方向/读数问题 |
| VF 能动但 Current 抖 | 闭环角度、相序、电流方向问题 |

VF 能转不代表闭环正确。它只能证明一部分底层链路可用。

## 8. 常见故障路径

### 8.1 一使能就 fault

看：

```text
state
fault_reason
safe_state_count
check.current_ok
check.ma600_ok
encoder_ok
i_sum
```

判断：

```text
fault_reason = current:
  先查电流采样、零漂、KCL、硬限。

fault_reason = encoder:
  先查 MA600 raw、reject/hold/age。

fault_reason = unsupported mode:
  control_mode 写了当前主固件不支持的值。
```

### 8.2 电机只抖不转

看：

```text
iq_ref, iq
id
vd, vq
v_limited
duty_u/v/w
encoder_elec
encoder_reject_count
i_sum
```

优先级：

```text
1. 角度零位/方向
2. PWM 相序
3. 电流相序/符号
4. 采样噪声
5. PI 参数
```

### 8.3 空载能转，带载不行

看：

```text
speed_iq_cmd
iq
vq
v_limited
duty_u/v/w
speed_err_rpm
```

判断：

```text
speed_iq_cmd 到限幅:
  iq_limit 可能太小，或负载超过当前允许转矩。

iq 跟不上 iq_ref 且 v_limited = 1:
  电压余量不足，不是速度 PI 问题。

iq 跟不上但 v_limited = 0:
  电流 PI、采样或角度需要查。
```

### 8.4 低速抖动

看：

```text
speed_deadband_count
speed_fb_rpm
speed_iq_cmd
id/iq
vd/vq
encoder_raw_step
```

可能原因：

```text
速度估算量化
速度 PI 积分保持
编码器零速抖动
电角度零位偏
电流采样噪声
```

先确认电流环稳定，再处理速度环死区、滤波和 KP/KI。

## 9. 推荐 Ozone 观察分组

不要一开始展开整个大结构。先分组看少量字段。

状态组：

```text
g_motor.runtime.state
g_motor.runtime.mode
g_motor.runtime.fault
g_motor.runtime.enabled
g_motor.runtime.pwm_output
g_motor.check.current_ok
g_motor.check.ma600_ok
g_motor.check.ready_closed_loop
g_motor.diag.slow_loop_count
g_motor.diag.fast_loop_count
g_motor.diag.safe_state_count
```

电流组：

```text
g_motor.current.phase.u
g_motor.current.phase.v
g_motor.current.phase.w
g_motor.current.dq.d
g_motor.current.dq.q
g_motor.current.id_ref_active.value
g_motor.current.iq_ref_active.value
```

角度组：

```text
g_motor.encoder.raw
g_motor.encoder.elec
g_motor.encoder.delta
g_motor.encoder.pos
g_motor.encoder.ok
g_motor.diag.encoder_raw_step
g_motor.diag.encoder_reject_count
g_motor.diag.encoder_hold_count
```

电压/PWM 组：

```text
g_motor.output.voltage_dq.d
g_motor.output.voltage_dq.q
g_motor.output.voltage_limited
g_motor.output.duty.u
g_motor.output.duty.v
g_motor.output.duty.w
g_motor.runtime.pwm_output
```

速度组：

```text
g_motor.debug.speed_ref_cmd_rpm
g_motor.debug.speed_ref_active_rpm
g_motor.debug.speed_fb_rpm
g_motor.debug.speed_err_rpm
g_motor.speed.iq_target
g_motor.speed.iq_ref.value
g_motor.speed.pi.integral
```

## 10. 最短定位流程

每次不转或异常，按这个顺序走：

```text
1. state/fault_reason 是否正常？
2. adc_sample_count 和 fast_loop_count 是否增长？
3. encoder_ok 是否正常，reject/hold 是否增长？
4. iu/iv/iw/i_sum 是否可信？
5. 给小 iq_ref 时 iq 是否跟随，id 是否接近 0？
6. vd/vq 是否饱和，v_limited 是否为 1？
7. duty 是否合理变化，pwm_running 是否为 1？
8. 如果是 Speed 模式，speed_iq_cmd 是否已经到限幅？
```

走完这八步，通常能把问题归到：

```text
状态机/命令问题
ADC/PWM 同步问题
电流采样问题
编码器/电角度问题
电流环参数问题
电压余量问题
速度环参数问题
机械负载问题
```
