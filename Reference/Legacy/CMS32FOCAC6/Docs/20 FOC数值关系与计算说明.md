# 20 FOC数值关系与计算说明

本文只解释当前 `CMS32FOCAC6` 程序里的数值关系，不泛泛讲 FOC。目标是让你在 Keil Watch、JScope 或 RTT 中看到 `g_cmd`、`g_elementary` 的数值时，能判断：

```text
这个数的单位是什么
它从哪里来
经过哪个公式变成下一个数
它大不大
异常时优先怀疑哪里
```

当前基线：

```text
MCU 主频        = 64 MHz
PWM 频率        = 20 kHz
PWM_PERIOD      = 1600
PWM_DUTY_50     = 800
ADC/PWM 同步    = EPWM CMP0 触发 ADC
快环入口        = ADC_IRQHandler() -> Motor_FastLoop()
电流环频率      = 20 kHz / MOTOR_FAST_LOOP_DIV = 20 kHz / 10 = 2 kHz
速度环频率      = 2 kHz / MOTOR_SPEED_LOOP_DIV = 2 kHz / 4 = 500 Hz
编译器          = Keil ARMCC5
```

## 1. PWM 数值关系

当前配置在：

```text
User/Config/Config.h
```

关键宏：

```c
#define PWM_FREQ_HZ 20000U
#define PWM_PERIOD 1600U
#define PWM_DUTY_50 800U
#define PWM_DUTY_MIN 300U
#define PWM_DUTY_MAX 1300U
#define PWM_DEADTIME_TICKS 64U
```

### 1.1 为什么 PWM_PERIOD 是 1600

中心对齐 PWM 上下计数，一个完整 PWM 周期包含上数和下数两段，所以：

```text
PWM_PERIOD = 64 MHz / (2 * 20 kHz)
           = 64,000,000 / 40,000
           = 1600
```

含义：

```text
计数 0     = 周期边界
计数 800   = 50% 中心点
计数 1600  = 半周期顶点
```

### 1.2 duty 数值代表什么

当前 `Foc_Svpwm()` 里：

```c
center = PWM_PERIOD / 2;  // 800
duty = center + phase_voltage + zero_sequence;
```

所以：

```text
duty = 800       约等于 50%
duty = 300       约等于 18.75%
duty = 1300      约等于 81.25%
1 duty count     约等于 1 / 1600 = 0.0625% PWM 周期
```

注意：这里的 `vd/vq/v_alpha/v_beta` 不是物理电压 V，而是“PWM count 尺度的电压命令”。例如 `vq = 20`，最终只会让三相 duty 在 800 附近变化几十个 count 以内，不一定能带动电机。

### 1.3 死区时间

当前：

```text
PWM_DEADTIME_TICKS = 64
EPWM tick 约为 64 MHz
1 tick = 1 / 64 MHz = 15.625 ns
64 tick = 1 us
```

所以当前死区约 `1 us`。这是为了避免上下桥臂同时导通。死区太小有直通风险，太大会造成低占空比失真和电流采样误差。

## 2. ADC 和电流换算

当前配置：

```c
#define ADC_VREF_V 3.6f
#define ADC_COUNTS 4096.0f
#define SHUNT_OHM 0.08f
#define PGA_GAIN 2.0f
#define ADC_TO_AMP (ADC_VREF_V / ADC_COUNTS / SHUNT_OHM / PGA_GAIN)
```

理论换算：

```text
1 ADC count 对应电压 = 3.6 V / 4096 = 0.0008789 V
采样电阻             = 0.08 ohm
PGA 增益             = 2

1 count 对应电流 = 0.0008789 / 0.08 / 2
                 = 0.005493 A
                 约 5.49 mA
```

常用数值：

```text
20 count   ≈ 0.110 A
60 count   ≈ 0.330 A
80 count   ≈ 0.439 A
300 count  ≈ 1.648 A
400 count  ≈ 2.197 A
```

这只是理论值，真实值还会受采样电阻误差、PGA 增益误差、ADC 参考误差、PCB 噪声和采样时刻影响。

## 3. raw current 和 logic current

当前运行时采样配置：

```c
#define CURRENT_SAMPLE_PAIR CURRENT_SAMPLE_VW
```

在 `Board_Analog.c` 中：

```c
s_iv_cnt = s_iv_raw_cnt;
s_iw_cnt = s_iw_raw_cnt;
s_iu_cnt = -s_iv_cnt - s_iw_cnt;
```

含义：

```text
raw current   = 某个 ADC/PGA 物理通道减零漂后的结果
logic current = 给 FOC 使用的 U/V/W 三相电流
```

当前 `VW` 模式下：

```text
Iv = raw V
Iw = raw W
Iu = -Iv - Iw
```

所以你看到：

```text
g_elementary.current.iv_raw_cnt
g_elementary.current.iw_raw_cnt
```

是两个实际采样相。`iu_raw_cnt` 在当前运行采样中可能被压回 offset 附近，因为 U 相没有作为本组 CMP0 触发采样通道。

判断 FOC 时优先看：

```text
g_elementary.current.iu
g_elementary.current.iv
g_elementary.current.iw
g_elementary.current.sum
g_elementary.foc.id
g_elementary.foc.iq
```

其中 `sum` 理论上应接近 0。当前两相重构模式下，logic 三相和通常会被构造得接近 0；更适合观察采样异常的是 raw 通道、id/iq 和电机响应。

## 4. MA600 角度、电角度和累计位置

当前宏：

```c
#define MOTOR_POLE_PAIRS 4u
#define MOTOR_SENSOR_POLE_PAIRS 4u
#define MOTOR_SENSOR_COUNTS_PER_REV 65536ul
#define MOTOR_POS_COUNTS_PER_MOTOR_REV (65536 * 4)
#define MOTOR_SENSOR_DIR (-1)
#define MOTOR_ELEC_ZERO 38778u
```

### 4.1 MA600 raw angle

`MA600` 原始角度范围：

```text
angle_raw = 0 ~ 65535
65536 count = MA600 磁场一圈 = 360 度
```

你的电机转子和空心磁环都按 `4 对极` 处理。当前结论是：

```text
FOC 电角度不再额外乘 4
```

原因：MA600 看到的是 4 对极磁环形成的磁场角，已经和电机电角度同频。若再乘 4，会把电角度放大错。

### 4.2 当前电角度公式

在 `Motor_GetElecAngle()` 中，因为：

```c
#define MOTOR_SENSOR_DIR (-1)
```

所以当前公式是：

```text
angle_elec = s_elec_zero - angle_raw
```

其中：

```text
s_elec_zero = MOTOR_ELEC_ZERO + g_cmd.angle.elec_zero_trim
```

如果临时零点扫描 `zero_scan_test.enable` 打开，当前程序会由零点扫描模块接管 trim。测试结束后要把 `best_trim` 写回：

```text
g_cmd.angle.elec_zero_trim = g_elementary.zero_scan_test.best_trim
g_cmd.zero_scan_test.enable = 0
```

### 4.3 累计位置 pos

在 `Motor_UpdateAngleFromIsr()` 中：

```c
delta = (int16_t)(raw - prev_raw);
pos += delta;
```

`delta` 用 `int16_t` 的原因是自动处理 0/65535 跨界。例如：

```text
raw 从 65530 到 10
raw - prev_raw = 10 - 65530
转成 int16_t 后得到一个小的正数
```

当前：

```text
1 个 MA600 raw 周期 = 65536 count
电机机械一圈       = 65536 * 4 = 262144 count
```

所以机械转速换算：

```text
mechanical_rps = speed_counts_per_s / 262144
mechanical_rpm = speed_counts_per_s * 60 / 262144
```

例子：

```text
speed_fb = 262144 count/s  -> 60 rpm
speed_fb = 131072 count/s  -> 30 rpm
speed_fb = 200000 count/s  -> 约 45.8 rpm
```

## 5. Clarke / Park 变换

FOC 里三相电流先变成静止坐标系 `alpha/beta`：

```c
i_alpha = iu;
i_beta = (iu + 2 * iv) * 5773 / 10000;
```

这里：

```text
5773 / 10000 ≈ 1 / sqrt(3)
```

然后用电角度做 Park 变换：

```c
id = (i_alpha * cos + i_beta * sin) >> 15;
iq = (-i_alpha * sin + i_beta * cos) >> 15;
```

当前正弦表使用 Q15：

```text
sin/cos 最大约 32767
>> 15 等价于除以 32768
```

角度单位：

```text
65536 count = 360 度
16384 count = 90 度
32768 count = 180 度
4096 count  = 22.5 度
512 count   = 2.8125 度
```

所以 `elec_zero_trim = 512` 实际就是电角度偏移约：

```text
512 / 65536 * 360 = 2.8125 度
```

如果 `id/iq` 跟预期不一致，常见原因是：

```text
电角度零点错
角度方向错
电流相序/符号错
PWM 相序和采样相序对应错
```

## 6. 电流 PI 怎么把 iq_ref 变成 vq

当前电流环宏：

```c
#define MOTOR_CURRENT_REF_LIMIT 300
#define MOTOR_CURRENT_V_LIMIT 300
#define MOTOR_CURRENT_PI_SHIFT 3u
#define MOTOR_CURRENT_KP 2
#define MOTOR_CURRENT_KI 0
#define MOTOR_CURRENT_SAFE_LIMIT 400
```

PI 更新公式在 `Foc_PiUpdate()`：

```text
error = ref - fb
integral_new = clamp(integral + error, -32767, 32767)
output_raw = kp * error + ki * integral
output = output_raw >> shift
output = clamp(output, -MOTOR_CURRENT_V_LIMIT, MOTOR_CURRENT_V_LIMIT)
```

当前 `Ki = 0`，所以积分不起作用：

```text
output = kp * error >> shift
       = 2 * error / 8
       = error / 4
```

例子，假设 `iq = 0`：

```text
iq_ref = 20   -> error = 20   -> vq = 5
iq_ref = 80   -> error = 80   -> vq = 20
iq_ref = 300  -> error = 300  -> vq = 75
```

这解释了一个重要现象：

```text
只开 Kp 且 Kp=2、shift=3 时，iq_ref=20 实际只生成 vq=5。
vq=5 对 PWM_PERIOD=1600 来说非常小，电机可能不动。
```

如果旧参数是 `Kp=4, Ki=0, shift=3`：

```text
output = 4 * error / 8 = error / 2

iq_ref = 20   -> vq = 10
iq_ref = 80   -> vq = 40
iq_ref = 300  -> vq = 150
```

### 6.1 为什么 Ki=1 时 iq_ref=0 也会跳

当 `Ki=1` 时，即使：

```text
iq_ref = 0
```

只要反馈 `iq` 因电角度零点、电流零漂、采样相位或噪声出现持续偏差：

```text
error = 0 - iq
```

积分项就会累加：

```text
integral += error
vq = (kp * error + ki * integral) >> shift
```

如果偏差方向周期变化，积分会把小误差放大成能推动电机的 `vq`，于是你会看到：

```text
iq_ref = 0
iq 在正负几十 count 内跳
vq 也周期摆动
电机可能自己来回动或慢慢转
```

这不是“Ki 一定错”，而是说明当前还不适合加积分。正确顺序是：

```text
先确认电流零漂
再确认电角度零点
再确认 q 轴正负方向
再逐步增加 Kp
最后才小心加入 Ki
```

## 7. vd/vq 限幅

PI 输出后还会经过：

```c
Foc_LimitDq(&vd, &vq, MOTOR_CURRENT_V_LIMIT);
```

当前限幅：

```text
MOTOR_CURRENT_V_LIMIT = 300
```

程序没有计算精确平方根，而是用近似幅值：

```text
mag ≈ max(abs(vd), abs(vq)) + min(abs(vd), abs(vq)) / 2
```

如果：

```text
mag > 300
```

就把 `vd/vq` 按比例缩小。

观察量：

```text
g_elementary.foc.v_limited
```

判断：

```text
v_limited = 0  -> dq 电压命令没有顶限幅
v_limited = 1  -> dq 电压命令已经被压缩
```

如果小 `iq_ref` 就经常 `v_limited=1`，优先怀疑：

```text
电角度零点错
电流反馈方向错
PI 参数过大
电流采样噪声/零漂大
电机堵转或负载太重
```

## 8. 反 Park 和 SVPWM

PI 得到 `vd/vq` 后，先变回静止坐标系：

```c
v_alpha = (vd * cos - vq * sin) >> 15;
v_beta  = (vd * sin + vq * cos) >> 15;
```

再进入 SVPWM：

```c
vu = v_alpha;
vv = -v_alpha / 2 + v_beta * 0.866;
vw = -v_alpha / 2 - v_beta * 0.866;

vzero = -(vmax + vmin) / 2;

duty_u = clamp(800 + vu + vzero, 300, 1300);
duty_v = clamp(800 + vv + vzero, 300, 1300);
duty_w = clamp(800 + vw + vzero, 300, 1300);
```

所以完整链路是：

```text
iq_ref
  -> error = iq_ref - iq
  -> PI 得到 vq
  -> InvPark 得到 v_alpha/v_beta
  -> SVPWM 得到 duty_u/v/w
  -> Board_SetPwmDuty()
  -> 三相功率输出
```

一个实用判断：

```text
如果 iq_ref 改变，vq 正负也改变，duty_u/v/w 也变化，说明命令链路基本通。
如果电机仍不按预期转，问题更可能在电角度零点、方向、相序、电流符号或幅值不足。
```

## 9. 电流环频率和速度环频率

ADC/PWM 同步基础频率：

```text
20 kHz
```

在 `Motor_FastLoop()` 中：

```c
div++;
if (div < MOTOR_FAST_LOOP_DIV) return;
div = 0;
```

当前：

```text
MOTOR_FAST_LOOP_DIV = 10
电流环频率 = 20 kHz / 10 = 2 kHz
电流环周期 = 0.5 ms
```

速度环在电流环基础上再分频：

```text
MOTOR_SPEED_LOOP_DIV = 4
速度环频率 = 2 kHz / 4 = 500 Hz
速度环周期 = 2 ms
```

速度估算不是用单次 `angle_delta * 20 kHz`，而是在 500 Hz 节拍做窗口差分：

```text
pos_delta = 当前 pos - 上次 pos
如果 abs(pos_delta) <= 16，则 speed_raw = 0
否则 speed_raw = pos_delta * 500
speed_filt += (speed_raw - speed_filt) >> 4
如果 abs(speed_filt) <= 200，则 speed_filt = 0
```

这样做是为了避免静止时 MA600 抖动被 20 kHz 放大。

## 10. 速度环怎么输出 iq_ref

速度环当前宏：

```c
#define MOTOR_SPEED_KP 1
#define MOTOR_SPEED_KI 0
#define MOTOR_SPEED_ERR_SHIFT 10u
#define MOTOR_SPEED_REF_LIMIT 200000l
#define MOTOR_SPEED_IQ_LIMIT_DEFAULT 20
```

速度 PI 公式：

```text
error = speed_ref - speed_fb
error = clamp(error, -200000, 200000)
error_scaled = error >> 10
iq_ref = Kp * error_scaled + Ki * integral
iq_ref = clamp(iq_ref, -iq_limit, iq_limit)
```

当前 `Kp=1, Ki=0`，默认 `iq_limit=20`。

例子：

```text
speed_ref = 20000 count/s
speed_fb  = 0
error_scaled = 20000 >> 10 ≈ 19
iq_ref ≈ 19
```

如果：

```text
speed_ref = 100000 count/s
error_scaled ≈ 97
```

但默认 `iq_limit=20`，所以最终：

```text
iq_ref = 20
```

这就是速度环不会直接给很大电流的原因。

## 11. VF/IF 开环数值关系

`VF` 和 `IF` 在：

```text
User/Motor/Src/Motor_OpenLoop.c
```

它们是早期验证工具，不是最终闭环主线。

### 11.1 VF

VF 做的事：

```text
按开环 theta 旋转一个固定 vd 电压矢量
不使用电流 PI
不证明真实电角度零点正确
```

链路：

```text
ol_speed_ref -> theta
vf_voltage -> vd
vd/vq -> InvPark -> SVPWM -> PWM
```

### 11.2 IF

IF 做的事：

```text
按开环 theta 做 Park/InvPark
中间加入 id/iq PI
可以验证电流采样和 PI 链路
方向主要由 ol_speed_ref 决定
```

链路：

```text
ol_speed_ref -> theta
if_id_ref / if_iq_ref -> PI -> vd/vq -> PWM
```

### 11.3 当前 open-loop 速度注释需要后续复核

`Config.h` 中写：

```c
#define MOTOR_OL_SPEED_REF_DEFAULT 400l
#define MOTOR_OL_SPEED_TO_THETA_STEP 131l
#define MOTOR_OL_SPEED_TO_THETA_SHIFT 8u
```

代码实际计算：

```text
theta_step = ol_speed_ref * 131 / 256
```

当前 IF/VF 调用频率约为 `2 kHz`，所以若 `ol_speed_ref=400`：

```text
theta_step ≈ 400 * 131 / 256 ≈ 204 count/次
每秒 theta ≈ 204 * 2000 = 408000 count/s
机械 rpm ≈ 408000 * 60 / 262144 ≈ 93 rpm
```

这和注释中“约 36 rpm”不完全一致。因此后续应统一 open-loop 速度单位。当前调试时以实际观察为准：

```text
增大 ol_speed_ref -> theta 变化更快
减小 ol_speed_ref -> theta 变化更慢
正负号决定开环旋转方向
```

## 12. 当前限幅总表

| 项目 | 当前值 | 含义 |
| ---- | ---- | ---- |
| `PWM_DUTY_MIN` | 300 | 最小 duty，约 18.75% |
| `PWM_DUTY_MAX` | 1300 | 最大 duty，约 81.25% |
| `MOTOR_CURRENT_REF_LIMIT` | 300 count | id/iq 给定限幅，理论约 1.65 A |
| `MOTOR_CURRENT_V_LIMIT` | 300 PWM count | vd/vq 电压命令限幅 |
| `MOTOR_CURRENT_SAFE_LIMIT` | 400 count | 运行中过流判断阈值，理论约 2.20 A |
| `MOTOR_CURRENT_OVER_LIMIT` | 4 次 | 连续过流 4 次才 fault |
| `FOC_PI_INTEGRAL_LIMIT` | 32767 | PI 积分内部限幅 |
| `MOTOR_SPEED_REF_LIMIT` | 200000 count/s | 速度给定限幅，约 45.8 rpm |
| `MOTOR_SPEED_IQ_LIMIT_DEFAULT` | 20 count | 速度环默认输出 iq 限幅，理论约 0.11 A |
| `MOTOR_SPEED_POS_DEADBAND` | 16 count | 速度估算位置死区 |
| `MOTOR_SPEED_ZERO_SNAP` | 200 count/s | 速度反馈归零吸附阈值 |

## 13. 常见现象怎么从数值判断

### 13.1 iq_ref 改了，电机不动

先看：

```text
g_elementary.foc.iq_ref
g_elementary.foc.iq
g_elementary.foc.vq
g_elementary.foc.v_limited
g_elementary.pwm.duty_u/v/w
```

判断：

```text
vq 只有几 count
  -> 当前 Kp-only 输出太小，力矩可能不够。

vq 正负会变，duty 也会变，但电机不动
  -> 可能幅值不足，也可能电角度/相序/电流符号不对。

v_limited = 1
  -> 已经顶到 dq 电压限幅，不要继续盲目加 ref。
```

### 13.2 iq_ref=0，Ki=1 时电机会自己动

判断：

```text
Ki 把零漂、角度误差、采样相位误差积分成了 vq。
```

处理顺序：

```text
Ki 先设 0
重新确认电流零漂
用零点扫描或手动 trim 找电角度
确认 q 正负方向
再逐步加 Kp
最后加很小 Ki
```

### 13.3 id 和 iq 都在跳，且幅值差不多

常见含义：

```text
d/q 坐标没有对准真实磁场
电流采样相序或符号可能仍有问题
角度和电流同步点可能不理想
```

此时不应先调速度环，也不应先加 Ki。

### 13.4 speed_fb 静止时跳

当前已经用 500 Hz 窗口差分、死区和滤波抑制。如果仍明显跳：

```text
看 angle.delta 是否静止抖动过大
看 pos_delta 是否超过 MOTOR_SPEED_POS_DEADBAND
看 MA600 安装间隙、磁环偏心、SPI 读数稳定性
```

### 13.5 duty 看起来变化很小

这是正常可能：

```text
vq=5 只代表 5 个 PWM count 量级
1 count 约 0.0625%
示波器或 JScope 上不一定明显
```

如果要做开环幅值验证，VF/IF 中的电压或电流给定要在安全范围内逐步增大，而不是直接判断闭环小 `iq_ref`。

## 14. 推荐调试顺序

当前阶段建议按这个顺序：

```text
1. Ki = 0，确认 iq_ref=0 时不会自己积分跑偏
2. 用零点扫描或手动 trim 找电角度零点
3. 电流环模式下给小 iq_ref，确认 vq 正负、duty 正负趋势
4. 如果 vq 太小，先提高 Kp 或提高 iq_ref，但观察 v_limited 和 current_over_count
5. q 轴方向稳定后，再加很小 Ki
6. 电流环可控后，再打开速度环
```

最重要的判断原则：

```text
命令链路通，不等于坐标正确。
IF 能转，不等于闭环电角度正确。
Kp-only 不动，不等于电流环完全错，可能只是输出尺度太小。
Ki 导致零给定也动，通常说明当前偏差还没被校正，不能先靠积分硬压。
```

## 15. 由当前电机参数推导出的控制量

用户补充的电机参数：

```text
相电阻 R     = 4 ohm
相电感 L     = 19 uH
反电势常数 Ke = 2.16 V/krpm
```

先说明一个关键前提：你的空心杯无刷电机是三线星型连接。如果电阻/电感是用万用表或 LCR 表直接量两根电机线得到的，那么它通常是线间值：

```text
R_line_line ≈ 2 * R_phase
L_line_line ≈ 2 * L_phase
```

也就是说：

```text
如果 4 ohm 是线间测量值，则单相 R_phase 约 2 ohm。
如果 19 uH 是线间测量值，则单相 L_phase 约 9.5 uH。
```

下面先按用户描述的“相电阻 4 ohm、相电感 19 uH”计算，同时给出线间测量时的修正理解。

### 15.1 Kv 转速常数

```text
Ke = 2.16 V/krpm
Kv = 1000 rpm / 2.16 V
   ≈ 463 rpm/V
```

如果用 `12 V` 母线做非常粗略的空载速度估算：

```text
no_load_speed ≈ 12 * 463
              ≈ 5556 rpm
```

这只是理想估算，实际会受母线利用率、绕组压降、摩擦、风阻、驱动压降和控制方式影响。

### 15.2 Ke 换成 SI 单位

```text
1000 rpm = 1000 * 2*pi / 60
         ≈ 104.72 rad/s

Ke = 2.16 V / 104.72 rad/s
   ≈ 0.0206 V/(rad/s)
```

如果厂家给的 `2.16 V/krpm` 是常见的线电压 RMS 反电势常数，那么换到 FOC 的相峰值/iq 体系还需要额外换算。当前阶段可以先记住工程近似：

```text
Kt 量级大约是 0.02 N*m/A
```

但不要立刻把它当最终扭矩标定值。后续要确认：

```text
Ke 是线电压还是相电压
Ke 是 RMS 还是峰值
电流 A 是相电流 RMS、相峰值，还是 FOC iq
```

### 15.3 电气时间常数

按 `R = 4 ohm`、`L = 19 uH`：

```text
tau = L / R
    = 19e-6 / 4
    = 4.75 us
```

对应电气极点：

```text
R / L = 4 / 19e-6
      ≈ 210526 1/s

f = R / (2*pi*L)
  ≈ 33.5 kHz
```

这个时间常数非常小，说明电流变化很快。对当前程序的影响：

```text
PWM 周期 = 1 / 20 kHz = 50 us
电气时间常数 tau ≈ 4.75 us
```

也就是一个 PWM 周期约等于 10 个电气时间常数。低电感空心杯电机电流纹波会比较明显，ADC 采样点和 PWM 触发位置非常重要。

如果 4 ohm 和 19 uH 实际是线间值，则单相约为：

```text
R_phase ≈ 2 ohm
L_phase ≈ 9.5 uH
tau = 9.5e-6 / 2 = 4.75 us
```

时间常数仍然是同一个量级。

### 15.4 电流变化斜率

电机绕组近似模型：

```text
V = R * I + L * dI/dt + E
```

静止且电流很小时，先忽略反电势 `E` 和电阻压降：

```text
dI/dt ≈ V / L
```

若给绕组等效 `12 V`：

```text
dI/dt = 12 / 19e-6
      ≈ 631579 A/s
      ≈ 0.632 A/us
```

这说明不能用很粗暴的电压长时间硬推低电感电机。即使当前有电阻限制，PWM 纹波和采样噪声也会比高电感电机更明显。

### 15.5 电流 count 和绕组压降

当前 ADC 理论换算：

```text
1 count ≈ 0.005493 A
```

所以：

```text
20 count  ≈ 0.110 A
80 count  ≈ 0.439 A
300 count ≈ 1.648 A
400 count ≈ 2.197 A
```

如果 `R_phase = 4 ohm`，维持这些电流需要的电阻压降约为：

```text
20 count:  0.110 A * 4 ohm = 0.44 V
80 count:  0.439 A * 4 ohm = 1.76 V
300 count: 1.648 A * 4 ohm = 6.59 V
400 count: 2.197 A * 4 ohm = 8.79 V
```

如果实际单相是 `2 ohm`，这些压降减半：

```text
20 count:  0.22 V
80 count:  0.88 V
300 count: 3.30 V
400 count: 4.39 V
```

这能解释当前调试中的一个现象：只用很小的 `Kp` 时，`iq_ref=20` 得到的 `vq` 很小，可能连维持 0.11 A 所需的绕组压降都不够。

### 15.6 vq count 粗略对应多少电压

当前 `SVPWM` 里，`vq/vd` 最终是 PWM count 尺度的电压命令。粗略估算：

```text
1 PWM count 约等于 VM / PWM_PERIOD
```

如果 `VM = 12 V`：

```text
1 count ≈ 12 / 1600 = 0.0075 V
```

所以：

```text
vq = 5    -> 约 0.038 V
vq = 20   -> 约 0.15 V
vq = 60   -> 约 0.45 V
vq = 120  -> 约 0.90 V
vq = 300  -> 约 2.25 V
```

这是很粗略的相电压尺度估算，真实相电压还受 SVPWM 零序注入、母线电压、死区、功率级压降和角度影响。但它足够说明：

```text
当前 Kp=2、shift=3 时：
iq_ref = 20, iq = 0 -> vq = 5
vq = 5 对 4 ohm 电机来说明显偏小。
```

如果要维持 `20 count ≈ 0.11 A`，4 ohm 情况下仅电阻压降就约 `0.44 V`，折算成 PWM count 大约：

```text
0.44 / 12 * 1600 ≈ 59 count
```

这就是为什么 `vq=5` 可能不动，而加入积分后又可能自己慢慢推起来。

### 15.7 对当前电流环调试的直接结论

由这些参数可以得到几个实用结论：

```text
1. 这个电机电感很低，电流纹波和采样点会非常敏感。
2. 当前 2 kHz 电流环只是安全调试频率，不是最终高性能电流环频率。
3. 只靠 Kp=2、shift=3，输出尺度偏小，小 iq_ref 可能推不动。
4. Ki 会把小偏差积累成足够电压，所以 Ki=1 时 iq_ref=0 也可能动。
5. MOTOR_CURRENT_REF_LIMIT=300 count 理论约 1.65 A，但在 MOTOR_CURRENT_V_LIMIT=300 下不一定能达到，尤其如果单相电阻真是 4 ohm。
6. 后续调 Kp/Ki 前，必须先确认电角度零点、采样相序和电流符号。
```

下一步如果要把这些参数用于 PI 初值，建议先确认：

```text
4 ohm 是线间电阻还是单相等效电阻
19 uH 是线间电感还是单相等效电感
Ke 的定义是线电压 RMS、相电压 RMS 还是峰值
当前 VM 实测值
电流采样的实际 A/count 是否和理论一致
```

## 16. 当前定点程序中 Q15 和 Q16 怎么分工

当前程序不建议把所有变量统一改成 Q15 或 Q16，而应按物理意义分工：

```text
角度：uint16_t 全周期角度，0~65535 表示 0~360 度，可理解为 unsigned Q16 turn。
sin/cos：int16_t Q15，范围约 -32767~32767。
电流：int16_t ADC count，不是 Q 格式。
vd/vq：int16_t PWM count 尺度，不是 Q 格式。
速度：int32_t sensor count/s，不是 Q 格式。
PI：整数 Kp/Ki + 右移 shift，属于工程定点缩放。
```

当前 FOC 变换采用的核心组合是：

```text
theta: uint16_t 角度 count
sin/cos: Q15
current/voltage: count
乘法中间量: int32_t
结果右移 15 位
```

例如 Park：

```text
id = (i_alpha * cos + i_beta * sin) >> 15
iq = (-i_alpha * sin + i_beta * cos) >> 15
```

这里必须用 Q15 的原因是 `sin/cos` 有正负号，且要表示接近 `+1.0/-1.0`。如果用 16 bit signed Q16，会遇到 `+1.0` 无法用 `int16_t` 表示的问题；若改用 32 bit Q16，又会让当前 M0+ 程序的乘法、右移和范围管理变复杂。

因此当前推荐：

```text
角度继续用 uint16_t 全周期角度。
sin/cos 和归一化系数用 Q15。
电流、vd/vq、duty 继续用工程 count。
需要更高精度的速度滤波、PI 内部积分，可以用 int32_t。
不要把整个 FOC 工程强行统一成 Q16。
```

如果后续要做更系统的 per-unit 控制，可以把电流和电压也归一化成 Q15，但这属于一次较大的控制尺度重构，当前调通电流环阶段不建议做。
