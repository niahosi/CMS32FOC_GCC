# FOC 电流环 PI 理论计算

本文档记录当前电机 FOC 电流环 PI 参数的完整理论推导，包括物理量到定点数的换算、连续域零极点对消设计、离散域直接设计，以及从当前调试参数出发的手动调参建议。

## 电机与系统参数

| 参数 | 符号 | 值 | 单位 | 来源 |
|------|------|-----|------|------|
| 相电感 | L | 19 | μH | 电机规格 |
| 相电阻 | R | 4 | Ω | 电机规格 |
| 反电势常数 | Ke | 2.16 | V/krpm | 电机规格 |
| 极对数 | p | 4 | pairs | BoardConfig.h |
| PWM 频率 | f_pwm | 20 | kHz | BoardConfig.h |
| 电流环频率 | f_cur | 20 | kHz | CTRL_FAST_LOOP_DIV=1 |
| 控制周期 | Ts | 50 | μs | 1/f_cur |
| ADC 参考电压 | Vref | 3.6 | V | BoardConfig.h |
| ADC 满量程 | — | 4096 | counts | 12-bit |
| 采样电阻 | Rshunt | 0.08 | Ω | BoardConfig.h |
| PGA 增益 | G_pga | 2.0 | — | BoardConfig.h |
| 电流环 PI 移位 | shift | 3 | bits | CTRL_CUR_PI_SHIFT |
| SVPWM 电压限幅 | v_limit | 887 | counts | PWM_SVPWM_V_LIMIT |

> **注意**：电压 scaling 依赖母线电压 Vbus。本文以 12V 为例计算，若实际 Vbus 不同，需要按比例修正 V_scale。公式见 §2.2。

---

## 1. 电机电气时间常数

FOC 电流环控制的物理对象是 dq 轴 RL 负载（忽略反电势耦合项，将其视为扰动）：

```
电机传递函数:  G_plant(s) = i(s)/v(s) = 1/(R + sL)
```

电气时间常数：

```
τ_e = L/R = 19×10⁻⁶ / 4 = 4.75 μs
```

对比控制周期：

```
Ts = 1/20000 = 50 μs
τ_e / Ts = 4.75 / 50 ≈ 0.095
```

电机 RL 极点位于：

```
f_pole = R/(2πL) = 4/(2π × 19×10⁻⁶) ≈ 33.5 kHz
```

这个频率远超 Nyquist 频率（10 kHz）和电流环带宽目标（1-2 kHz）。**从数字控制器的视角，电机 RL 动力学在每个采样周期内已经完成，可以近似为纯电阻增益加上 PWM 更新延迟。**

离散域被控对象模型（ZOH 等效 + 一拍计算延迟）：

```
G_plant(z) ≈ K_plant / z
K_plant = (1/R) × (I_scale⁻¹ / V_scale⁻¹)
```

这是后续所有 PI 设计的基础。

---

## 2. 物理量到定点 counts 的换算

打通从"安培/伏特"到代码中 int16 的完整换算链，是理解 PI 参数物理含义的前提。

### 2.1 电流 scaling

```
ADC LSB = Vref / ADC_COUNTS = 3.6 / 4096 = 0.8789 mV

电流采样电压:  V_sense = I_phase × Rshunt × G_pga = I_phase × 0.08 × 2.0 = I_phase × 0.16 V

1 ADC count 对应电流:  0.8789 mV / 0.16 V/A = 5.493 mA

ADC_TO_AMP = 3.6 / 4096 / 0.08 / 2.0 = 0.005493 A/count
```

换算关系：

```
1 A   →  1 / 0.005493 = 182.0 counts
1 mA  →  0.182 counts
1 count → 5.493 mA
```

### 2.2 电压 scaling

PI 输出的 vd/vq 经过 `foc_inv_park` → `foc_svpwm` 转换为 PWM 占空比。电压量在代码中用与 duty 相同的 scale。

SVPWM 线性调制区最大相电压幅值 = Vbus/√3。以 Vbus = 12V 为例：

```
V_phase_max = 12 / √3 = 6.928 V
v_limit     = PWM_SVPWM_V_LIMIT = ((800 - 32) × 1000) / 866 ≈ 887 counts

1 V_count  = 6.928 / 887 = 0.00781 V
1 V        → 887 / 6.928 = 128.0 counts
```

若 Vbus = 24V，则 V_scale 翻倍：

```
V_phase_max = 24 / √3 = 13.856 V
1 V → 887 / 13.856 = 64.0 counts
```

> **调试提示**：如果你的实际母线电压不是 12V，需要重新代入计算。V_count_per_V = v_limit × √3 / Vbus。

### 2.3 离散对象增益

PI 输出 → V_count，经过逆变器 → 相电压 V，在电机绕组产生电流 I → ADC 采样 → I_count。

整个链路的稳态增益（不含延迟）：

```
I_count / V_count = (I_per_A⁻¹) / (R × V_per_V⁻¹)
                  = 182 / (4 × 128)
                  = 182 / 512
                  = 0.355 count/count
```

加上一拍 PWM 更新延迟：

```
G_plant(z) = K_plant / z = 0.355 / z
```

物理含义：在 12V 母线、4Ω 电阻下，每输出 1 count 的 vd/vq，大约产生 0.355 count 的 id/iq 反馈。一拍采样延迟体现在分母 `z` 上。

---

## 3. 定点 PI 控制器的实现

### 3.1 代码实现

当前 PI 实现在 [foc_math.c](../Firmware/MotorControl/Algorithm/foc_math.c) 的 `foc_pi_update()` 中。

伪代码：

```
error        = clamp(ref - feedback, ±32767)
integral_new = clamp(integral + error, ±32767)
output_raw   = Kp × error + Ki × integral_new
output       = clamp(output_raw >> shift, ±v_limit)

// 抗积分饱和: 仅在输出未饱和、或饱和但误差在退饱和方向时更新积分
if (output == output_unclamped) ||
   (output_unclamped > output_max && error < 0) ||
   (output_unclamped < output_min && error > 0):
    integral = integral_new
```

### 3.2 差分方程与传递函数

```
u[n] = [Kp × e[n] + Ki × Σe[k]] >> shift

                          Kp + Ki/(1 − z⁻¹)
离散传递函数:  G_pi(z) = -------------------
                               2^shift
```

### 3.3 积分行为

当 Ki ≠ 0 时，积分项每个采样周期累加一次误差。**Ki 的大小决定了积分收敛速度**。

```
每个采样周期积分增量:  Δintegral = Ki × error (after shift)
稳态时积分饱和值:      integral_ss ≈ (output_needed × 2^shift - Kp × error_ss) / Ki
```

Ki 太小 → 积分收敛慢，静差消除慢，可能无法克服静摩擦。
Ki 太大 → 积分超调、振荡、id 抖动增加。

### 3.4 与连续时间的对应关系

连续 PI 输出：

```
u(t) = Kp_phys × e(t) + Ki_phys × ∫e(t)dt
```

前向欧拉离散化（Ts = 50 μs）：

```
u[n] = Kp_phys × e[n] + Ki_phys × Ts × Σe[k]
```

代入单位换算（e 在 I_count 域，u 在 V_count 域）：

```
u_counts = Kp_phys × (V_per_count / I_per_count) × e_counts
         + Ki_phys × Ts × (V_per_count / I_per_count) × Σe_counts

Kp_eff = Kp_phys × V_per_count / I_per_count
Ki_eff = Ki_phys × Ts × V_per_count / I_per_count
```

加上代码中的右移：

```
Kp_code = Kp_eff × 2^shift = Kp_phys × V_per_count / I_per_count × 2^shift
Ki_code = Ki_eff × 2^shift = Ki_phys × Ts × V_per_count / I_per_count × 2^shift
```

代入 12V 母线数值（V_per_count = 128, I_per_count = 182, Ts = 50×10⁻⁶, shift = 3）：

```
Kp_code = Kp_phys × (128/182) × 8 = Kp_phys × 5.626
Ki_code = Ki_phys × 50×10⁻⁶ × (128/182) × 8 = Ki_phys × 2.813×10⁻⁴
```

---

## 4. 连续域设计 — 零极点对消法

### 4.1 原理

PI 控制器的连续传递函数：

```
G_pi(s) = Kp + Ki/s = Kp × (1 + 1/(s·Ti))
其中 Ti = Kp/Ki 是积分时间常数
```

零极点对消：令 PI 的零点 = 电机 RL 极点：

```
Ti = τ_e = L/R = 4.75 μs
→ Ki/Kp = R/L = 4 / (19×10⁻⁶) = 210,526 rad/s ≈ 33.5 kHz
```

消去电机极点后，开环传递函数退化为纯积分器：

```
G_ol(s) = G_pi(s) × G_plant(s)
        = Kp(1 + s·Ti)/(s·Ti) × (1/R)/(1 + s·τ_e)
        = Kp / (sL)           (当 Ti = τ_e)
```

闭环传递函数为一阶低通：

```
G_cl(s) = G_ol / (1 + G_ol) = 1 / (1 + s·L/Kp)
```

**闭环带宽**：

```
ω_c = Kp / L  (rad/s)
f_c = Kp / (2πL)  (Hz)
```

### 4.2 带宽选择

数字控制系统的带宽受限于：

```
总延迟 Td ≈ 1.5 × Ts = 75 μs   (ADC 采样 + 计算 + PWM 更新)
延迟等效极点 f_delay ≈ 1/(2π × Td) ≈ 2.1 kHz
```

工程经验：

```
保守:  f_bw ≤ f_pwm / 20 = 1.0 kHz
适中:  f_bw ≤ f_pwm / 10 = 2.0 kHz
激进:  f_bw ≤ f_pwm / 5  = 4.0 kHz  (需实际验证稳定性)
```

### 4.3 不同带宽的手算结果

| f_bw | ω_c [rad/s] | Kp_phys [V/A] | Ki_phys [V/(A·s)] | **Kp_code** | **Ki_code** |
|------|-------------|---------------|-------------------|-------------|-------------|
| 500 Hz | 3,142 | 0.060 | 12,570 | 0.3 → **0** | 3.5 → **3** |
| 1000 Hz | 6,283 | 0.119 | 25,140 | 0.7 → **1** | 7.1 → **7** |
| 2000 Hz | 12,566 | 0.239 | 50,320 | 1.3 → **1** | 14.2 → **14** |
| 4000 Hz | 25,133 | 0.478 | 100,640 | 2.7 → **3** | 28.4 → **28** |

> **注**：由于 Kp_code 采用整数，500-2000 Hz 范围内 Kp_code 差异很小（0 到 1）。这是因为电机 τ_e 极短，从控制器角度 Kp 的作用更像衰减因子而非带宽设定。带宽主要由 Ki 决定。

---

## 5. 离散域直接设计

由于 τ_e << Ts，传统零极点对消的带宽物理意义在这个系统中不准确。更严谨的做法是直接在 z 域设计。

### 5.1 被控对象

```
G_plant(z) = K_plant / z = 0.355 / z
```

### 5.2 开环传递函数

```
G_ol(z) = G_pi(z) × G_plant(z)
        = [Kp + Ki/(1 − z⁻¹)] / 8 × 0.355 / z
```

### 5.3 闭环特征方程

```
1 + G_ol(z) = 0

8z² + [−8 + 0.355(Kp + Ki)]z − 0.355Kp = 0

z² + [0.355(Kp + Ki) − 8]/8 × z − 0.355Kp/8 = 0
```

### 5.4 稳定性分析

**纯 P 控制（Ki = 0）**：

```
特征方程: z² + [0.355Kp − 8]/8 × z − 0.355Kp/8 = 0

稳定条件 (Jury 判据): 0 < Kp < 22.5
边际稳定: Kp = 22.5 (根在单位圆上)
```

**加入 Ki 后**，积分引入了低频极点，使系统从一阶变为二阶。主导极点向 z=1 靠近，系统响应变慢但稳态误差消除。

### 5.5 当前参数的极点分析

**Kp = 4, Ki = 0（纯 P）**：

```
8z² + (−8 + 1.42)z − 1.42 = 0
z² − 0.8225z − 0.1775 = 0
z₁ = 1.0, z₂ = −0.178

稳态误差 ≈ 1/(1 + Kp × K_plant/8) = 1/(1 + 4×0.355/8) = 1/1.178 = 84.9%
```

纯 P 控制下约 85% 的给定-反馈误差无法消除。

**Kp = 4, Ki = 1（当前调试值）**：

```
8z² + (−8 + 1.775)z − 1.42 = 0
z² − 0.778z − 0.178 = 0
z₁ = 0.963, z₂ = −0.185
```

主导极点 0.963 对应：

```
τ_dominant = −Ts / ln(0.963) = −50μs / (−0.0377) = 1.33 ms
f_dominant ≈ 1/(2π × 1.33ms) ≈ 120 Hz
```

**Kp = 4, Ki = 4（推荐调试值）**：

```
8z² + (−8 + 2.84)z − 1.42 = 0
z² − 0.645z − 0.178 = 0
z₁ = 0.856, z₂ = −0.211 → τ ≈ 322 μs, f ≈ 495 Hz
```

### 5.6 离散域推荐参数

| Kp | Ki | 主导极点 | 等效带宽 | 说明 |
|----|-----|---------|---------|------|
| 4 | 1 | 0.963 | ~120 Hz | 当前值，已确认能转 |
| 4 | 2 | 0.915 | ~280 Hz | 积分加速 |
| 4 | 4 | 0.856 | ~500 Hz | **推荐调试值** |
| 4 | 8 | 0.777 | ~800 Hz | 激进值 |
| 8 | 4 | 0.874 | ~430 Hz | 提高比例响应 |
| 12 | 8 | 0.889 | ~600 Hz | 激进值，注意噪声 |

---

## 6. 手动调参路径

### 6.1 调参原则

根据 [CurrentLoopBringupNotes.md](CurrentLoopBringupNotes.md) 中记录的调试经验：

```
1. current_ki = 0 时，电机可能不容易起转
2. 加入 current_ki 后能克服摩擦、死区和绕组压降
3. 但 iq 可能超调，id 抖动也会变大
```

因此调参顺序：

```
小 iq_ref (20/30) → 验证 Kp → 加入 Ki → 验证起转 → 观察 id/iq → 调整
```

### 6.2 推荐步骤

**阶段 1：确认 Ki 范围**（固定 Kp=4）

```
Ki=1  → 当前值，已确认能转            → 基准
Ki=2  → 积分加速一倍                  → 观察 iq 超调量
Ki=4  → 接近 500 Hz 离散域理论值       → 推荐
Ki=8  → 接近 800 Hz                   → 激进值，注意 id 抖动
```

**阶段 2：调整 Kp**（固定 Ki=4）

```
Kp=2  → 降低比例增益，更依赖积分       → 如果 Kp=4 声音粗糙
Kp=4  → 当前值                        → 基准
Kp=8  → 提高比例响应                  → 如果 iq 跟踪不够快
Kp=12 → 激进值                        → 注意高频噪声和电压限幅
```

**阶段 3：观察判断**

| 现象 | 含义 | 建议 |
|------|------|------|
| iq 能跟上 iq_ref，稳态误差小 | Ki 足够 | 保持 |
| iq 明显过冲后回落 | Ki 太大 | 降低 Ki |
| 电机声音粗糙、高频嘶嘶声 | Kp 太大 | 降低 Kp |
| v_limited 长期为 1 | PI 输出饱和 | 降低 Kp 或提高母线电压 |
| id 转动时明显变大 | 非 PI 问题 | 检查动态角度延迟/MA600 侧轴非线性 |
| 正反 iq_ref 都能转 | 基本方向正确 | ✓ |

### 6.3 进入速度环的前置条件

在切入速度环前，电流环至少应满足：

```
- iq 与 iq_ref 同号且可控
- id 均值没有长期偏大
- v_limited 不长期为 1
- 电机声音平顺
- 正反 iq_ref 都能转
```

---

## 7. 关键数值关系速查

```
┌────────────────────────────────────────────────────────────┐
│  电机电气时间常数   τ_e = L/R = 4.75 μs                     │
│  控制周期           Ts = 50 μs (20 kHz)                    │
│  τ_e / Ts = 0.095  ← 电机比控制快 10+ 倍                    │
│                                                            │
│  电流增益:  1A = 182 counts                                │
│  电压增益:  1V = 128 counts (12V bus)                       │
│  对象增益:  K_plant = 0.355 count/count (含一拍延迟)          │
│                                                            │
│  纯 P 稳定上限:  Kp < 22.5                                  │
│  当前 Kp=4 纯 P: ~250 Hz 比例带宽，85% 静差                   │
│  当前 Kp=4 Ki=1: 主导极点 0.963，~120 Hz 有效带宽             │
│                                                            │
│  连续域理论值 (1kHz):  Kp=1, Ki=7                           │
│  离散域推荐值:         Kp=4, Ki=4  (~500 Hz)                 │
│  离散域激进值:         Kp=4, Ki=8  (~800 Hz)                 │
└────────────────────────────────────────────────────────────┘
```

### 核心公式

```
电气时间常数:       τ_e = L/R
零极点对消条件:     Ki_phys / Kp_phys = R/L
闭环带宽:           ω_c = Kp_phys / L
代码 Kp:            Kp_code = Kp_phys × (V_scale/A_scale) × 2^shift
代码 Ki:            Ki_code = Ki_phys × Ts × (V_scale/A_scale) × 2^shift
离散对象增益:       K_plant = (1/R) × (A_scale⁻¹ / V_scale⁻¹)
离散特征方程:       8z² + [−8 + 0.355(Kp+Ki)]z − 0.355Kp = 0
```

### 符号速查

| 符号 | 含义 | 当前值 |
|------|------|--------|
| A_scale | I_per_A, 1A 对应多少 counts | 182 |
| V_scale | V_per_V, 1V 对应多少 counts (12V bus) | 128 |
| K_plant | 离散对象增益 | 0.355 |
| shift | PI 定点右移 | 3 (即 ÷8) |
| Ts | 电流环采样周期 | 50 μs |

---

## 8. 相关文档

- [CurrentLoopBringupNotes.md](CurrentLoopBringupNotes.md) — 调试经验、零位、方向、MA600 侧轴
- [BoardConfig.h](../Firmware/Board/Config/BoardConfig.h) — 硬件基线参数
- [TuneConfig.h](../Firmware/Board/Config/TuneConfig.h) — 当前 PI 参数和调试开关
- [foc_math.c](../Firmware/MotorControl/Algorithm/foc_math.c) — PI 定点实现
