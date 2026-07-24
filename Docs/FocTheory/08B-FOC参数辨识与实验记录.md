# FOC 参数辨识与实验记录

本文从外部理论笔记迁入，并已按当前 GCC 工程路径和参数做同步；若后续代码与本文冲突，以当前源码为准。

本文讲怎么把 FOC 理论参数变成当前工程里的实测数据。

目标不是一次算出完美参数，而是建立一套可复现的记录方法：

```text
测量 R/L/Ke
换算 count
做小信号阶跃
看 id/iq/vd/vq/v_limited
根据现象判断下一步
```

## 1. 先区分三类参数

硬件/电机参数：

```text
R: 相电阻
L: 相电感
Ke: 反电动势常数
pole_pairs: 电机极对数
VBUS: 母线电压
```

采样/输出换算：

```text
ADC_TO_AMP
I_COUNT_PER_A
V_COUNT_PER_V
PWM_SVPWM_V_LIMIT
```

控制参数：

```text
CTRL_CUR_KP / CTRL_CUR_KI / CTRL_CUR_PI_SHIFT
CTRL_SPD_KP / CTRL_SPD_KI / CTRL_SPD_ERR_SHIFT
iq_limit
current_v_limit
```

先测前两类，再调第三类。不要拿控制参数去掩盖硬件参数未知。

## 2. 测相电阻 R

低阻电机用普通万用表直接量很容易受表笔电阻影响。

建议：

```text
1. 短接表笔，记录表笔电阻。
2. 测任意两相线电阻 R_line。
3. 扣除表笔电阻。
4. 若星型电机且测的是线线电阻，单相电阻约 R_phase = R_line / 2。
```

记录：

| 项目 | 数值 |
| --- | ---: |
| 表笔短接电阻 | 待测 |
| U-V 线线电阻 | 待测 |
| V-W 线线电阻 | 待测 |
| W-U 线线电阻 | 待测 |
| 估算相电阻 R | 待测 |

注意温升。铜电阻会随温度上升，冷态和热态电流响应会不同。

## 3. 测相电感 L

如果有 LCR 表，记录测量频率和接法。

建议：

```text
1. 测 U-V、V-W、W-U 线线电感。
2. 记录 LCR 表频率，例如 1 kHz。
3. 若按每相模型使用，说明采用的是线线电感还是折算相电感。
```

没有 LCR 表时，也可以用电流阶跃估算。低速堵转、小电压下：

```text
i(t) = V/R * (1 - exp(-t * R / L))
tau = L / R
```

如果能记录电流上升曲线，可以从 63% 上升时间估算：

```text
L ~= R * tau
```

当前工程没有高速波形记录通道时，这种估算可以先在低速小电流下用 watch
做粗略判断，但不要把 watch 低刷新率当成精密波形仪。

## 4. 估 Ke

反电动势常数可以用开路拖动测量。

思路：

```text
1. 让电机以已知机械 rpm 匀速旋转。
2. PWM 关闭，测任意两相线线反电动势。
3. 记录是 RMS、峰值还是峰峰值。
4. 换算为 V/krpm。
```

注意单位必须写清楚：

```text
line-line RMS
line-line peak
phase peak
```

当前 `Tools/foc_teaching_sim.py` 的教学参数写的是：

```text
Ke = 2.16 V/krpm line-to-line peak
```

这个值适合教学演示。真机调试时要以你的电机实测值为准。

## 5. 换算当前 count

电流换算：

```text
ADC_TO_AMP = ADC_VREF_V / ADC_COUNTS / SHUNT_OHM / PGA_GAIN
I_COUNT_PER_A = 1 / ADC_TO_AMP
```

当前：

```text
ADC_VREF_V = 3.6
ADC_COUNTS = 4096
SHUNT_OHM = 0.08
PGA_GAIN = 2
I_COUNT_PER_A ~= 182 count/A
```

电压换算按母线粗估：

```text
V_COUNT_PER_V ~= (PWM_DUTY_MAX - PWM_DUTY_MIN) / VBUS
```

如果 VBUS = 12 V：

```text
V_COUNT_PER_V ~= 128 count/V
```

记录：

| 项目 | 数值 |
| --- | ---: |
| VBUS | 待测 |
| ADC_TO_AMP | 0.005493 A/count |
| I_COUNT_PER_A | 182 count/A |
| V_COUNT_PER_V | 待算 |
| CTRL_CUR_V_LIMIT | 886 count |
| 约等效电压上限 | 待算 |

## 6. 从 R/L 计算电流 PI 候选值

选目标带宽：

```text
fc = 1 kHz 起步
wc = 2 * pi * fc
```

连续 PI：

```text
Kp_phys = L * wc
Ki_phys = R * wc
```

换算固件参数：

```text
kp_count = Kp_phys * V_COUNT_PER_V / I_COUNT_PER_A
ki_count_per_sample = Ki_phys * Ts * V_COUNT_PER_V / I_COUNT_PER_A

CTRL_CUR_KP = round(kp_count * 2^CTRL_CUR_PI_SHIFT)
CTRL_CUR_KI = round(ki_count_per_sample * 2^CTRL_CUR_PI_SHIFT)
```

当前：

```text
Ts = 1 / 20000 = 50 us
CTRL_CUR_PI_SHIFT = 3
```

建议至少算三组：

| fc | KP | KI | 用途 |
| ---: | ---: | ---: | --- |
| 1 kHz | 待算 | 待算 | 保守起步 |
| 2 kHz | 待算 | 待算 | 响应更快 |
| 3 kHz | 待算 | 待算 | 只在采样/角度很干净时尝试 |

## 7. 电流阶跃实验

实验条件：

```text
Current 模式
id_ref = 0
iq_ref 从 0 分级给 20 / 40 / 60 count
iq_limit 略高于 iq_ref
speed_ref_rpm = 0
```

观察：

```text
id_ref
iq_ref
id
iq
vd
vq
v_limited
duty_u/v/w
i_sum
encoder_ok
```

记录表：

| KP | KI | iq_ref | iq 稳态 | id 稳态 | vq | v_limited | duty 是否贴边 | 现象 |
| ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| 4 | 1 | 20 | 待测 | 待测 | 待测 | 0/1 | 待填 | 待填 |

判断：

```text
iq 慢且不饱和:
  PI 可能偏弱。

iq 抖、vd/vq 抖:
  KP 偏大或采样噪声大。

iq 有稳态误差且不饱和:
  KI 偏弱。

v_limited = 1:
  先查电压余量、角度、相序，不先加 PI。

id 明显偏大:
  先查零位，不先调 PI。
```

## 8. 速度阶跃实验

前提：

```text
Current 模式下 iq 能稳定跟随。
```

实验：

```text
Speed 模式
speed_ref_rpm = 50 / 100 / 200 分级
iq_limit 先小，例如 40 或 80 count
```

观察：

```text
speed_ref_rpm
speed_ref_active_rpm
speed_fb_rpm
speed_err_rpm
speed_iq_target
speed_iq_cmd
speed_pi_integral
iq_ref
iq
v_limited
```

记录表：

| speed_kp | speed_ki | speed_ref_rpm | iq_limit | speed_fb_rpm | speed_iq_cmd | iq | v_limited | 现象 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 32 | 3 | 100 | 80 | 待测 | 待测 | 待测 | 0/1 | 待填 |

判断：

```text
speed_iq_cmd 到限幅:
  速度环已经要最大转矩，查电流环、电压和负载。

speed_fb_rpm 符号反:
  先查传感器方向。

速度来回摆:
  speed_kp 可能偏大，或速度反馈延迟/噪声。

长期稳态误差但 iq 未限幅:
  speed_ki 可能偏弱。
```

## 9. 实验注意事项

```text
每次只改一个参数。
先小电流，再大电流。
先低速，再高速。
先 Current 模式，再 Speed 模式。
先确认 v_limited=0，再讨论 PI 好坏。
```

不要用整结构 watch 当波形仪。Ozone live watch 适合看慢变量和状态趋势；
电流环 20 kHz 细节需要示波器、DAC 输出、RTT/SWO 或更专门的记录机制。

## 10. 最后形成参数结论

每组最终参数都应该写清楚：

```text
电机型号
VBUS
R/L/Ke 来源
采样电阻和 PGA
PWM 频率
电流 PI 参数
速度 PI 参数
验证过的 iq_limit / speed_ref_rpm 范围
已知限制，例如高转速 v_limited
```

这样以后换电机、换电源、换采样电阻时，能知道哪些参数需要重算，哪些只是调试入口。
