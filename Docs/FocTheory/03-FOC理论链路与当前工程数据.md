# FOC 理论链路与当前工程数据

本文从外部理论笔记迁入，并已按当前 GCC 工程路径和参数做同步；若后续代码与本文冲突，以当前源码为准。

本文专门讲 FOC 的理论链路，并把每个理论量对应到当前 `cms32foc`
工程里的数据、代码和 watch 字段。

不要把 FOC 理解成一个单独公式。它是一条连续的数据链：

```text
三相电流 iu/iv/iw
  -> Clarke 得到 alpha/beta
  -> Park 得到 id/iq
  -> 电流 PI 得到 vd/vq
  -> InvPark 得到 alpha/beta 电压
  -> SVPWM 得到 duty_u/v/w
  -> 功率级输出三相电压
  -> 电机电流变化
  -> ADC 再采样
```

当前主线代码在：

```text
Firmware/MotorControl/Core/current.cpp
Firmware/MotorControl/Core/output.cpp
Firmware/MotorControl/Algorithm/foc_math.c
```

## 1. FOC 控制的目标

三相电机本来有三相电流：

```text
iu, iv, iw
```

直接控制三相正弦电流很麻烦，因为它们一直随电角度旋转。FOC 的目标是把
这个旋转问题变成两个近似直流量：

```text
id: d 轴电流，磁链方向
iq: q 轴电流，转矩方向
```

表贴 PMSM / BLDC 的基础用法通常是：

```text
id_ref = 0
iq_ref = 需要的转矩电流
```

所以初学时先记住：

```text
iq 决定转矩
id 通常先压到 0
```

当前工程 Ozone 里对应：

```text
g_motor.current.phase.u / v / w
g_motor.current.dq.d / q
g_motor.current.id_ref_active.value / iq_ref_active
g_motor.output.voltage_dq.d / q
g_motor.output.duty.u / v / w
g_motor.output.voltage_limited
```

## 2. Clarke 变换

三相星型电机理想满足：

```text
iu + iv + iw = 0
```

所以三相信息可以压缩成静止坐标里的两轴：

```text
alpha, beta
```

当前工程先去掉三相公共偏置：

```text
mid = (iu + iv + iw) / 3
u' = iu - mid
v' = iv - mid
w' = iw - mid
```

然后用 U/V 两相做 Clarke：

```text
alpha = iu
beta  = (iu + 2 * iv) / sqrt(3)
```

固件里用整数近似：

```text
beta = (iu * 9459 + iv * 18919) >> 14
```

这里：

```text
9459 / 16384 ~= 1 / sqrt(3)
18919 / 16384 ~= 2 / sqrt(3)
```

这里选择 `>> 14`，也就是用 `2^14 = 16384` 做分母，不是因为 Q14 比 Q15 更“高级”，而是因为 beta 公式里有一个系数：

```text
2 / sqrt(3) = 1.154700...
```

这个数大于 1。普通 signed Q15 的正数范围约为：

```text
0 到 32767 / 32768 = 0.999969
```

所以 Q15 装不下 `1.154700...`。Q14 的正数范围约为：

```text
0 到 32767 / 16384 = 1.999939
```

因此 Q14 可以同时表示：

```text
1 / sqrt(3) ~= 0.577350
2 / sqrt(3) ~= 1.154700
```

换算成整数就是：

```text
round(0.577350 * 16384) = 9459
round(1.154700 * 16384) = 18919
```

所以当前实现：

```text
beta = (iu * 9459 + iv * 18919) >> 14
```

含义就是：

```text
beta ~= iu * 0.577350 + iv * 1.154700
```

注意这只说明 Clarke 的 beta 系数使用 Q14。后面的 Park / InvPark 仍然使用 Q15 的 sin/cos，因为 sin/cos 天然在 `-1.0 到 +1.0` 范围内，适合用 Q15。

如果 `i_sum` 很大，Clarke 前的三相基础就不可靠。优先查：

```text
ADC 零漂
电流采样窗口
两相重构
MOT_CURR_PHASE_MAP
MOT_CURR_SIGN
```

## 3. Park 变换

Clarke 后的 `alpha/beta` 仍然是静止坐标。Park 用电角度 `theta` 把它旋转到
转子坐标：

```text
d =  alpha * cos(theta) + beta * sin(theta)
q = -alpha * sin(theta) + beta * cos(theta)
```

当前工程里的 `theta` 来自：

```text
encoder_elec + voltage_theta_offset
```

对应代码：

```text
theta_used = encoder_elec + voltage_theta_offset
current_ab = foc_clarke_3phase(current)
current_dq = foc_park(current_ab, theta_used)
```

这一步最怕电角度错。电角度错后，真实 q 轴电流会被投影到 d 轴，或者正负方向反掉。

典型现象：

```text
给正 iq_ref，iq 不上升或方向反
id 明显跟着 iq_ref 变化
vd/vq 很快变大
v_limited = 1
duty 贴边
```

优先查：

```text
MOT_SENSOR_DIR
MOT_ELEC_ZERO
elec_zero_trim
voltage_theta_offset
MOT_PWM_PHASE_MAP
MOT_CURR_PHASE_MAP
MOT_CURR_SIGN
```

## 4. 电流 PI

Park 后，电流环就变成两个单轴闭环：

```text
d 轴: id_ref -> PI -> vd
q 轴: iq_ref -> PI -> vq
```

当前工程的 PI 形式：

```text
error = ref - feedback
output = (kp * error + ki * integral) >> shift
```

其中：

```text
ref/feedback: 电流 count
output: 电压 count
shift: 定点缩放
```

当前默认：

```text
CTRL_CUR_KP = 4
CTRL_CUR_KI = 1
CTRL_CUR_PI_SHIFT = 3
```

所以比例项量级：

```text
4 / 8 = 0.5 voltage_count/current_count
```

也就是电流误差 100 count 时，比例项先给约 50 count 电压。

理论上低速可把电机单轴看成 RL 对象：

```text
L * di/dt = v - R * i
```

常见初始推导：

```text
wc = 2 * pi * fc
Kp_phys = L * wc
Ki_phys = R * wc
```

然后再换成当前固件的 count 参数。这个推导的完整换算在
`02-当前程序-理论与参数指南.md` 的电流 PI 小节。

调试时要看：

```text
id_ref, iq_ref
id, iq
vd, vq
v_limited
duty_u, duty_v, duty_w
```

如果 `id/iq` 跟不上，但 `v_limited = 0`，可能是 PI 偏弱、角度偏差或采样噪声。
如果 `v_limited = 1`，先不要继续加 PI，说明电压命令已经碰到限幅。

## 5. InvPark 和 SVPWM

电流 PI 输出的是转子坐标里的电压：

```text
vd, vq
```

功率级不能直接输出 d/q 电压，所以先反 Park：

```text
alpha = d * cos(theta) - q * sin(theta)
beta  = d * sin(theta) + q * cos(theta)
```

然后 SVPWM 把 `alpha/beta` 电压转成三相 duty。

当前工程的 SVPWM 思路是零序注入：

```text
vu = alpha
vv = -alpha / 2 + beta * 0.866
vw = -alpha / 2 - beta * 0.866
vzero = -(max(vu, vv, vw) + min(vu, vv, vw)) / 2
duty = center + phase_voltage + vzero
```

当前 PWM 标尺：

```text
PWM_PERIOD = 1600
PWM_DUTY_50 = 800
PWM_DUTY_MIN = 32
PWM_DUTY_MAX = 1568
```

所以：

```text
duty 接近 800: 输出电压小
duty 接近 32/1568: 输出接近边界
```

`vd/vq` 在输出前还会被 `CTRL_CUR_V_LIMIT` 限制。当前约：

```text
PWM_SVPWM_V_LIMIT = 886 count
```

如果 duty 贴边或 `v_limited = 1`，通常表示：

```text
电压余量不够
电角度/相序不对
负载太大
PI 输出过强
电流采样异常导致 PI 误判
```

## 6. 速度环和电流环的关系

当前工程有 Current 和 Speed 两条主模式。

Current 模式：

```text
g_mc_cmd.iq_ref
  -> MotorControl_ApplyCommand()
  -> g_motor.command.current.iq_ref
  -> g_motor.current.iq_ref_active.value
```

Speed 模式：

```text
speed_ref_rpm / speed_ref
  -> 速度 PI
  -> speed_iq_ref
  -> 电流环 iq_ref
```

所以速度环不是直接控制 duty，也不是直接控制电压。它输出的是 q 轴电流命令。

调速度前必须确认电流环正常。否则速度环只会把 `iq_ref` 推到限幅，掩盖底层问题。

Speed 模式重点看：

```text
speed_ref_rpm
speed_ref_active_rpm
speed_fb_rpm
speed_err_rpm
speed_iq_target
speed_iq_cmd
iq_ref
iq
v_limited
```

如果 `speed_iq_cmd` 已经到 `iq_limit`，但速度还上不去，先判断：

```text
iq 是否真的跟上 iq_ref
v_limited 是否为 1
duty 是否贴边
speed_fb_rpm 方向是否正确
负载是否超过当前 iq_limit 能提供的转矩
```

## 7. 一条完整的读数链

分析 FOC 问题时，不要只看一个字段。按这条链读：

```text
enable/control_mode/state/fault_reason
  -> adc_sample_count/fast_loop_count
  -> encoder_ok/encoder_elec/encoder_raw_step
  -> iu_cnt/iv_cnt/iw_cnt/i_sum
  -> id_ref/iq_ref/id/iq
  -> vd/vq/v_limited
  -> duty_u/duty_v/duty_w/pwm_running
  -> speed_fb_rpm/speed_err_rpm/speed_iq_cmd
```

每一层都回答一个问题：

```text
状态机是否允许控制？
快环是否真的在跑？
角度是否可信？
电流采样是否可信？
d/q 投影是否符合预期？
电压是否已经饱和？
PWM 是否真的输出？
速度外环是否只是被底层问题拖住？
```
