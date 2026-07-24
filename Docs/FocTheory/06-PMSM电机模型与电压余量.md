# PMSM 电机模型与电压余量

本文从外部理论笔记迁入，并已按当前 GCC 工程路径和参数做同步；若后续代码与本文冲突，以当前源码为准。

本文补 FOC 里最容易被忽略的一层：电机本体和电压余量。

当前工程已经能看到：

```text
iq_ref / iq
vd / vq
v_limited
duty_u / duty_v / duty_w
speed_fb_rpm
```

这些字段背后真正描述的是：

```text
母线电压够不够
电机反电动势吃掉了多少电压
电阻压降吃掉了多少电压
电流变化还需要多少电压
```

如果这些关系没想清楚，就容易把“电压已经不够”误判成“PI 不够大”。

## 1. d/q 电压方程

PMSM 在 d/q 坐标下的常见模型是：

```text
vd = R * id + Ld * d(id)/dt - we * Lq * iq
vq = R * iq + Lq * d(iq)/dt + we * Ld * id + we * psi
```

其中：

```text
vd/vq = d/q 轴电压
id/iq = d/q 轴电流
R     = 相电阻
Ld/Lq = d/q 轴电感
we    = 电角速度，单位 rad/s
psi   = 永磁体磁链
```

表贴 PMSM 通常可以先近似：

```text
Ld ~= Lq ~= L
id_ref = 0
```

于是 q 轴更容易看：

```text
vq ~= R * iq + L * d(iq)/dt + we * psi
```

这三个电压分量分别代表：

```text
R * iq:
  稳态电流需要的电阻压降。

L * d(iq)/dt:
  电流快速变化需要的电感电压。

we * psi:
  转速越高越大的反电动势。
```

## 2. 转矩从哪里来

表贴 PMSM 的基础转矩公式：

```text
Te = 1.5 * pole_pairs * psi * iq
```

所以初学阶段可以这样理解：

```text
iq 越大，转矩越大。
psi 越大，同样 iq 产生的转矩越大。
pole_pairs 越多，同样机械转速下电角速度越高。
```

当前 watch 里没有直接输出转矩，所以调试时只能间接看：

```text
iq_ref / iq
speed_fb_rpm
speed_err_rpm
speed_iq_cmd
```

如果 `speed_iq_cmd` 到限幅，但 `iq` 没跟上，先查电流环和电压余量。
如果 `iq` 跟上了但速度仍上不去，才考虑负载转矩、机械卡滞或 `iq_limit` 太小。

## 3. 当前工程的电压 count

当前 SVPWM 输入的电压单位不是伏特，而是 voltage count。

当前宏：

```text
PWM_PERIOD = 1600
PWM_DUTY_50 = 800
PWM_DUTY_MIN = 32
PWM_DUTY_MAX = 1568
PWM_SVPWM_V_LIMIT ~= 886 count
```

`CTRL_CUR_V_LIMIT` 默认跟随 `PWM_SVPWM_V_LIMIT`。

如果按 12 V 母线粗估：

```text
V_COUNT_PER_V ~= (PWM_DUTY_MAX - PWM_DUTY_MIN) / VBUS
              = 1536 / 12
              = 128 count/V
```

所以：

```text
886 count ~= 6.9 V
```

这个 6.9 V 不是精密标定值，只是用于判断量级。真实电压还会受死区、MOSFET
压降、母线波动、采样时序和 SVPWM 调制方式影响。

## 4. 为什么高速更容易饱和

低速时，q 轴电压大致用于：

```text
R * iq
L * d(iq)/dt
```

高速时还要加上：

```text
we * psi
```

也就是反电动势。

电角速度和机械速度关系：

```text
we = mechanical_rpm * 2 * pi / 60 * pole_pairs
```

当前电机极对数：

```text
MOT_POLE_PAIRS = 4
```

所以同样机械 rpm 下，电角速度是机械角速度的 4 倍。

如果 `Ke` 较大，高速时 `we * psi` 会占掉大部分可用电压。结果是：

```text
vq 增大
v_limited = 1
duty 贴边
iq 跟不上 iq_ref
speed_iq_cmd 越积越大
speed_fb_rpm 上不去
```

这时问题不是速度 PI 不够强，而是可用电压已经不够。

## 5. 电阻压降也会吃掉电压

当前文档和教学仿真中用过一个量级：

```text
R ~= 4 ohm
```

只看电阻压降：

```text
0.5 A -> 2 V
1.0 A -> 4 V
1.7 A -> 6.8 V
```

如果当前可用 q 轴电压只有约 6.9 V，那么在低速稳态下，1.7 A 左右已经接近
电压天花板。高速时还要扣掉反电动势，能维持的电流更小。

所以看到 `CTRL_CUR_REF_LIMIT = 1000 count` 不代表硬件真能长期输出 1000 count 电流。

当前电流换算：

```text
1 count ~= 5.49 mA
1000 count ~= 5.49 A
```

如果 4 ohm 电机真要 5.49 A，仅电阻压降就是：

```text
5.49 A * 4 ohm = 21.96 V
```

这已经超过 12 V 母线。这个例子说明：软件电流限幅和物理可实现电流不是一回事。

## 6. 看数据判断是不是电压余量问题

优先看：

```text
vd
vq
v_limited
duty_u
duty_v
duty_w
iq_ref
iq
speed_iq_cmd
speed_fb_rpm
```

更像电压余量不足：

```text
v_limited = 1
duty_u/v/w 贴近 32 或 1568
vq 长期很大
iq_ref 增大但 iq 不再增加
speed_iq_cmd 到限幅但 speed_fb_rpm 上不去
高速比低速更明显
```

不太像电压余量不足：

```text
v_limited = 0
duty 远离边界
vd/vq 很小但 iq 不跟
```

后者更应该查：

```text
电角度
相序
电流采样
PI 参数
功率级使能
```

## 7. 电压余量不足时怎么处理

按优先级：

```text
1. 降低 iq_ref 或 iq_limit，确认小电流是否正常。
2. 降低 speed_ref_rpm，确认低速是否正常。
3. 看电角度和相序，排除错误相位导致的无效电压。
4. 检查电源母线是否真的达到预期电压。
5. 检查 duty guard 是否过窄，但不要为了追电压盲目压缩安全余量。
6. 高速场景再考虑反电动势前馈、解耦前馈或弱磁。
```

当前主固件还没有实现：

```text
反电动势前馈
d/q 解耦前馈
弱磁控制
MTPA
```

所以当前阶段的正确判断是：先证明基础闭环和电压余量，再谈高级补偿。

## 8. 推荐记录表

| VBUS | speed_ref_rpm | iq_ref/iq_limit | iq | vq | v_limited | duty 范围 | 现象 | 判断 |
| ---: | ---: | ---: | ---: | ---: | --- | --- | --- | --- |
| 12 V | 低速 | 小 | 待测 | 待测 | 0/1 | 待测 | 待填 | 待填 |
| 12 V | 高速 | 同上 | 待测 | 待测 | 0/1 | 待测 | 待填 | 待填 |

对比低速和高速很重要。如果低速电流环正常，高速才开始 `v_limited = 1`，
大概率就是反电动势和电压余量问题。
