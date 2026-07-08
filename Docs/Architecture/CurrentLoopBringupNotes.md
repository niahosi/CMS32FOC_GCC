# Current Loop Bring-up Notes

本文记录当前小电机 FOC bring-up 中已经确认的关键经验：电角度零位、传感器方向、为什么 q 轴给定会影响电机转不转，以及转起来后 `id` 变大的原因。

当前主固件是纯 C bring-up 路径：

```text
ADC_IRQHandler -> curr_irq() -> MotorControl_FastLoopFromAdcIrq()
```

C++ 控制层暂时冻结，不作为当前调试依据。

## Electrical Angle

当前电角度由 MA600 原始角度、固定零位和 Ozone trim 共同决定。

在 `MOT_SENSOR_DIR = 1` 时：

```c
encoder_elec = MOT_ELEC_ZERO + elec_zero_trim + raw * MOT_SENSOR_ELEC;
```

在 `MOT_SENSOR_DIR = -1` 时：

```c
encoder_elec = MOT_ELEC_ZERO + elec_zero_trim - raw * MOT_SENSOR_ELEC;
```

因此：

```text
实际零位 = MOT_ELEC_ZERO + g_motor_cmd.elec_zero_trim
```

这个结果按 16-bit 电角度自然回绕。例如：

```text
MOT_ELEC_ZERO = 9577
elec_zero_trim = -24905

实际零位 = 9577 - 24905 = -15328
16-bit 回绕后 = 50208
```

如果要固化当前 Ozone 调出来的零位，可以把 `MOT_ELEC_ZERO` 改成回绕后的值，然后把 `g_motor_cmd.elec_zero_trim` 重新设为 `0`。

## Why Direction Matters

`MOT_SENSOR_DIR` 不是简单的显示方向，它决定控制算法看到的电角度方向。

FOC 的 Park/InvPark 都依赖同一个电角度：

```text
phase current -> Clarke -> Park(theta) -> id/iq
vd/vq -> InvPark(theta) -> SVPWM -> PWM duty
```

如果 PWM 产生的旋转磁场是正方向，但 MA600 反馈电角度在算法里是反方向，就会出现：

```text
d 轴能吸住
q 轴没有持续转矩
速度环给定后只动一下或掉下去
```

本项目中曾经出现过这个现象。把 `MOT_SENSOR_DIR` 从 `-1` 改为 `1` 后，`control_mode = 1/2` 下 q 轴给定才开始能让电机转动。这说明当前硬件和算法坐标需要：

```c
#define MOT_SENSOR_DIR (1)
```

## Zero Is Not Only "Smallest id"

调零位时不能只看 `id` 是否最小，还必须看电机是否能稳定产生转矩。

有效零位的判断顺序应该是：

```text
1. 电机能稳定转
2. 声音平顺
3. iq 能跟随 iq_ref
4. id 均值尽量接近 0
```

如果某个 zero 下 `id` 看起来很小，但电机不转或像锁住一样，这个点不能作为工作零位。它可能只是让当前测得的 dq 投影变得安静，并不代表真实 q 轴转矩正确。

推荐调零流程：

```text
1. control_mode = 4 做 align，得到 align_zero_trim。
2. 把 align_zero_trim 写入 g_motor_cmd.elec_zero_trim。
3. 切 control_mode = 1。
4. 使用 id_ref = 0，iq_ref = 20/30/40。
5. 围绕当前 zero 扫 elec_zero_trim。
6. 选择能转、声音顺、id 均值较小的点。
```

## Static id And Dynamic id

当前已经观察到：

```text
电机停下或静止锁轴时，id 可以很小。
电机转起来后，id 明显变大。
```

这说明主要问题不再是静态零位，而是动态相位误差。

动态 `id` 常见来源：

```text
1. MA600 读取、控制计算、PWM 更新之间存在延迟。
2. 转子转动后，实际电角度已经前进，但控制还在使用旧角度。
3. q 轴电流因此投影出一部分 d 轴电流。
4. 电流采样噪声和 PWM/ADC 同步误差会进一步放大 dq 抖动。
```

经验判断：

```text
静止 id 小，转动 id 大 -> 动态角度滞后/相位提前问题。
静止 id 也大 -> 零位、相序或电流采样映射问题。
iq_ref 正负能让电机正反转 -> 基本方向已经通。
```

后续可以加入速度相关电角度提前：

```text
theta_used = encoder_elec + theta_advance
theta_advance = speed_fb * advance_gain
```

在正式加入动态提前前，可以先用固定 offset 诊断：

```text
theta_used = encoder_elec + g_motor_cmd.voltage_theta_offset
```

当前 `voltage_theta_offset` 已接入 `Current` 和 `Speed` 闭环路径，也接入 `EncoderVoltage` 诊断路径。VF 开环不使用 MA600 电角度，因此不使用该 offset。

观察转动时 `id` 是否变小、`iq` 是否更接近 `iq_ref`、声音是否更顺。

## MA600 Side-Shaft Hollow Ring Notes

当前硬件不是 MA600 端轴同轴安装，而是：

```text
MA600 + 侧边空心磁环
```

上一层参考文档已经记录：

```text
Reference/Legacy/CMS32FOCAC6/Docs/Hardware/H02 MA600磁体安装与读数.md
Reference/Legacy/CMS32FOCAC6/Docs/15 MA600手册阅读指南.md
Reference/Legacy/CMS32FOCAC6/Docs/06 电机参数填写表.md
```

MA600 官方手册也说明，side-shaft/off-axis 安装时，传感器测到的是封装 XY 平面内磁场方向。这个磁场角和真实机械角不再天然线性。侧边安装时径向磁场 `BRAD` 和切向磁场 `BTAN` 幅值通常不同，比例：

```text
k = BRAD / BTAN
```

当 `k != 1` 时，角度误差呈周期性双正弦形。MA600 支持用 BCT 和 32 点用户校准改善这个非线性。

这对当前 FOC 的影响是：

```text
1. 如果磁环极对数等于电机极对数，MA600 磁场角可以作为 FOC 电角度的基础。
2. 但侧边安装的非线性会让这个电角度带周期性误差。
3. 静止对齐时 zero 可能看起来正确。
4. 一旦电机转起来，周期性角度误差会把 q 轴电流投影到 d 轴。
5. 表现为 iq 能出力，但 id 随转动明显变大，并伴随嘟嘟声或转矩脉动。
```

因此，当前 `id` 压不进很小范围，不一定全是电流环 PI 或 zero 没调好。它也可能来自 MA600 侧边磁环的角度非线性。

当前配置：

```c
#define MOT_POLE_PAIRS 4u
#define MOT_SENSOR_POLE_PAIRS 4u
#define MOT_SENSOR_ELEC (MOT_POLE_PAIRS / MOT_SENSOR_POLE_PAIRS)
```

含义是：磁环一圈内的 4 个磁场周期与电机 4 对极匹配，所以 FOC 电角度可以直接使用校正后的 MA600 磁场角。但机械位置换算时不能把 `angle_raw` 当作一圈机械角，机械一圈对应：

```text
4 * 65536 counts
```

后续若要把 `id` 和转矩脉动进一步压低，调试顺序应是：

```text
1. 保持当前已验证的 MOT_SENSOR_DIR、相序和有效 zero。
2. 先加固定 theta offset 或速度相关 theta advance，补偿动态延迟。
3. 低速匀速记录 encoder_raw / encoder_elec / id / iq / speed_fb_rpm。
4. 如果 id 和速度波动呈固定电角度周期变化，再考虑 MA600 BCT 或 32 点校准。
5. BCT/32 点校准后再重新标定 zero。
```

不要一开始就写 MA600 NVM。BCT、方向和零位应先用临时寄存器或固件侧补偿验证。

## Current PI Notes

当前测试还观察到：

```text
current_ki = 0 时，电机可能不容易起转。
加入 current_ki 后能克服摩擦、死区和绕组压降，但 iq 可能超调，id 抖动也会变大。
```

因此 PI 调试顺序建议：

```text
1. 先用较小 iq_ref，例如 20/30。
2. current_kp 从 2 或 3 开始。
3. current_ki 先用 1，确认能起转。
4. 如果 iq 明显高于 iq_ref，降低 kp 或限制积分。
5. 不要在 id 动态偏差很大时直接上高速速度环。
```

进入速度环前，电流环至少应满足：

```text
iq 与 iq_ref 同号并可控
id 均值不要长期偏大
v_limited 不长期为 1
电机声音平顺
正反 iq_ref 都能转
```

## What Has Been Ruled Out

在相同条件下测试过 `MOT_CURR_PHASE_MAP`：

```text
UVW: 能转，是当前最接近正确的电流相序。
VUW: 母线电流明显升高，约 300 mA，且不转。
WUV: 母线电流较低，约 70 mA，但不转。
```

因此当前阶段不要继续优先改电流相序，应保持：

```c
#define MOT_CURR_PHASE_MAP MOT_PHASE_MAP_UVW
```

主线应放在：

```text
1. 固化有效 zero。
2. 优化电流 PI。
3. 判断是否需要动态角度提前。
4. 再进入速度环。
```
