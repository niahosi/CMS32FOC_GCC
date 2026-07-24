# VF 与 IF 开环理论

本文从外部理论笔记迁入，并已按当前 GCC 工程路径和参数做同步；若后续代码与本文冲突，以当前源码为准。

本文讲开环 VF/IF 的理论，以及当前工程里的 VF 模式应该怎么用。

当前主固件支持：

```text
MC_MODE_VF_OPEN_LOOP
```

IF 字段还保留在命令结构里，但主固件当前不运行 IF 主线。

## 1. VF 开环是什么

VF 开环就是：

```text
给一个旋转角 theta
给一个固定或随速度变化的电压幅值 V
用 InvPark + SVPWM 输出旋转电压矢量
```

当前实现：

```text
theta = open_loop_theta
vd = 0
vq = vf_voltage
MotorControl_InternalApplyVoltageVector(vd, vq, theta)
```

open-loop theta 按命令速度推进：

```text
open_loop_theta += open_loop_speed_ref * OL_SPEED_TO_THETA_STEP >> OL_SPEED_TO_THETA_SHIFT
```

当前配置：

```text
OL_SPEED_REF = 50
OL_VF_VOLTAGE = 80
OL_TIMEOUT_MS = 30000
```

## 2. VF 为什么能让电机转

旋转电压矢量会在定子里形成旋转磁场。转子永磁体会尝试跟随这个旋转磁场。

如果：

```text
电压足够
频率不太高
负载不太大
方向正确
```

电机就能开环转起来。

但 VF 没有使用真实转子角度闭环，所以它不知道转子是否真的跟上。

## 3. 低频为什么要补电压

电机 q 轴电压近似：

```text
vq ~= R * iq + L * d(iq)/dt + back_emf
```

低速时反电动势小，但如果电阻 R 较大，仍需要电压克服 `R*iq`。

所以简单 VF 里，太小的 `vf_voltage` 可能：

```text
磁场在转，但电流不够
转子跟不上
只轻微抖动
```

当前 VF 是诊断入口，不是完整 V/f 曲线控制。不要把它当成长期运行模式。

## 4. VF 适合定位什么

VF 不依赖编码器电角度闭环，所以适合分层验证：

```text
PWM/SVPWM 输出是否生效
功率级是否能推动电机
电流采样是否有响应
编码器 raw/speed 是否能观察到运动
电机和相线大体是否接通
```

watch：

```text
open_loop_theta
open_loop_ticks
open_loop_reset_count
vf_voltage
duty_u/v/w
iu_cnt/iv_cnt/iw_cnt
speed_fb_rpm
```

判断：

```text
open_loop_theta 不变:
  VF 快环没有运行。

duty 不变:
  输出链路没有生效。

duty 变但电流不变:
  功率级、相线、采样链路要查。

电机动但 speed_fb_rpm 符号反:
  查 MOT_SENSOR_DIR。
```

## 5. VF 能转不代表闭环正确

VF 不证明这些正确：

```text
MOT_ELEC_ZERO
Park/InvPark 使用的闭环电角度
电流方向
PWM 相序和电流相序一致性
电流 PI 参数
```

所以：

```text
VF 能转，Current 模式仍可能抖。
```

这通常指向：

```text
电角度零位
相序
电流符号
电流采样
```

## 6. IF 开环是什么

IF 开环不是直接给电压，而是给开环角下的电流目标：

```text
open_loop_theta
id_ref / iq_ref
电流 PI 闭环
vd/vq 输出
```

它比 VF 多用了电流环，理论上能控制电流幅值。

但 IF 仍然不使用真实转子角度闭环，所以转子是否同步仍然是开环问题。

当前主固件里：

```text
if_id_ref
if_iq_ref
OL_IF_ID_REF
OL_IF_IQ_REF
```

这些字段保留，但主线没有运行完整 IF 模式。

## 7. 开环风险

开环模式风险：

```text
转子可能失步
低速可能发热
高电压可能过流
机械可能突然动
编码器角度问题不会阻止 VF 输出
```

当前 VF 保留基础电流检查和超时：

```text
current_ok
open_loop_timeout_ms
MC_FAULT_OPEN_LOOP_TIMEOUT
```

使用 VF 时建议：

```text
先小 vf_voltage。
先低 open_loop_speed_ref。
短时间观察。
手边准备 enable=0 或 stop。
```

## 8. 推荐 VF 实验

| vf_voltage | open_loop_speed_ref | duty 是否变化 | 电流响应 | speed_fb_rpm | 现象 | 判断 |
| ---: | ---: | --- | --- | ---: | --- | --- |
| 40 | 低 | 待测 | 待测 | 待测 | 待填 | 待填 |
| 80 | 低 | 待测 | 待测 | 待测 | 待填 | 待填 |

只要能证明输出、电流、编码器观察链路，就可以退出 VF，进入 Current 模式做闭环验证。
