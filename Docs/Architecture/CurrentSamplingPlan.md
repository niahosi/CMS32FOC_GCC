# Current Sampling Plan

本文设计 `cms32foc` 下一版三电阻电流采样方案。该方案按 TI 应用笔记的思路，以 PWM 占空比分布决定 CMPB/ADC 触发点，并明确每一拍哪些相电流有效、哪些相必须忽略。

重点不是固定 `UV/UW/VW` 某一组，也不是任意移动采样点，而是按本拍 PWM 矢量区间分三种情况处理：

```text
情况一：T1 > TMinWidth
    在 T1 区域触发 ADC，U/V/W 三相电流均有效，直接使用。

情况二：T1 <= TMinWidth 且 T2 >= TMinWidth
    在 T2 区域触发 ADC，U/V 有效，W 无效。
    W = -(U + V)。

情况三：T1 <= TMinWidth 且 T2 < TMinWidth
    在 T3 区域触发 ADC，只有 U 有效，V/W 无效。
    V/W 使用软件后台低通滤波结果，不能用 KCL 重构。
```

## Why Rewrite

前一版设计把 TI 方案简化成“动态选择两相低边窗口”。这不能完整表达 TI 的三段逻辑，尤其漏掉了情况三：只有一相有效时，不能通过 `Ia + Ib + Ic = 0` 得到另外两相，必须忽略无效相并使用软件滤波结果。

当前波形中的“重合段、贴零段、马鞍波”正说明采样点经常落在某些相无效的 PWM 区间。下一版必须让采样层输出“本拍有效相掩码”，不能让控制环误以为三相都可信。

## Terms

`PWM_PERIOD` 是中心对齐半周期计数，例如 16 kHz 下为 `2000`。

先把三相占空比按大小排序：

```text
Dmax >= Dmid >= Dmin
```

定义三个可采区间宽度：

```text
T1 = PWM_PERIOD - Dmax
T2 = Dmax - Dmid
T3 = Dmid - Dmin
```

这里采用当前工程已经验证更接近真实波形的 duty/低边模型来落地。若后续用示波器确认 CMS32 的比较极性相反，排序关系和 T 区间需要整体翻转，但三种情况的判断结构保持不变。

最小采样窗口：

```text
TMinWidth = CS_TMIN_WIDTH_TICK
```

它必须覆盖：

```text
死区时间 + MOS/采样放大器恢复时间 + ADC 扫描两相或三相所需时间 + 安全边距
```

初始建议：

```c
#define CS_TMIN_WIDTH_TICK (PWM_DEADTIME_TICKS + 160U)
```

16 kHz、`PWM_PERIOD = 2000` 下，这个值先保守一点，后续通过 `sample_spread` 和示波器缩小。

## Phase Identity

TI 文档中的 U/V/W 是排序后的相，不一定等于物理 U/V/W 固定名称。实现时需要保存排序映射：

```c
typedef enum {
    CS_PHASE_U = 0,
    CS_PHASE_V = 1,
    CS_PHASE_W = 2,
} CurrPhaseId;

typedef struct {
    CurrPhaseId max_phase;
    CurrPhaseId mid_phase;
    CurrPhaseId min_phase;
    uint16_t dmax;
    uint16_t dmid;
    uint16_t dmin;
} CurrDutyOrder;
```

为了和 TI 三种情况对齐，本文后续称：

```text
U = max_phase
V = mid_phase
W = min_phase
```

但落到 ADC 通道时，必须通过 `max_phase/mid_phase/min_phase` 映射回物理 `ADC_CH_0/2/3`。

## Sample Plan

新增计划结构：

```c
typedef enum {
    CS_REGION_T1 = 1,
    CS_REGION_T2 = 2,
    CS_REGION_T3 = 3,
} CurrSampleRegion;

typedef enum {
    CS_RECON_NONE = 0,       // 三相直接有效
    CS_RECON_ONE_PHASE = 1,  // 两相有效，第三相 KCL 重构
    CS_RECON_FILTER = 2,     // 只有一相有效，其余用低通结果
} CurrReconMode;

typedef struct {
    CurrSampleRegion region;
    CurrReconMode recon_mode;
    uint8_t valid_mask;      // bit0 physical U, bit1 physical V, bit2 physical W
    uint8_t ignore_mask;     // bit0 physical U, bit1 physical V, bit2 physical W
    uint16_t cmpb_tick;
    uint16_t t1;
    uint16_t t2;
    uint16_t t3;
    CurrDutyOrder order;
} CurrSamplePlan;
```

## Case 1: T1 Is Wide Enough

条件：

```text
T1 > TMinWidth
```

采样策略：

```text
CMPB 放在 T1 区域中部。
ADC 扫描三相 U/V/W。
三相采样值全部有效。
```

有效性：

```text
valid_mask = U | V | W
ignore_mask = 0
recon_mode = CS_RECON_NONE
```

解析：

```text
Iu = ADC_U - offset_U
Iv = ADC_V - offset_V
Iw = ADC_W - offset_W
```

这时不要再做 KCL 覆盖三相原始值。可以只计算 `i_sum` 做诊断。

## Case 2: T1 Too Narrow, T2 Wide Enough

条件：

```text
T1 <= TMinWidth
T2 >= TMinWidth
```

采样策略：

```text
CMPB 放在 T2 区域中部。
ADC 采样排序后的 U、V 两相。
排序后的 W 相无效，必须忽略。
```

有效性：

```text
valid_mask = physical(U) | physical(V)
ignore_mask = physical(W)
recon_mode = CS_RECON_ONE_PHASE
```

解析：

```text
Iu = ADC_U - offset_U
Iv = ADC_V - offset_V
Iw = -(Iu + Iv)
```

注意：这里的 U/V/W 是排序后的相，最终要写回对应物理相。

## Case 3: T1 And T2 Too Narrow

条件：

```text
T1 <= TMinWidth
T2 < TMinWidth
```

采样策略：

```text
CMPB 放在 T3 区域中部。
只使用排序后的 U 相 ADC 结果。
排序后的 V/W 相必须忽略。
```

有效性：

```text
valid_mask = physical(U)
ignore_mask = physical(V) | physical(W)
recon_mode = CS_RECON_FILTER
```

解析：

```text
Iu = ADC_U - offset_U
Iv = lowpass_Iv
Iw = lowpass_Iw
```

不能使用：

```text
Iv 或 Iw = -(其他两相之和)
```

因为本拍只有一相是真实有效值，KCL 无法唯一恢复另外两相。低通值由后台一直维护的三相电流滤波器提供。

## Low-Pass Current Backup

采样层维护物理三相低通值：

```c
typedef struct {
    int16_t u;
    int16_t v;
    int16_t w;
} CurrFilter;
```

更新规则：

```text
如果某物理相本拍有效：
    filter_phase += (sample_phase - filter_phase) >> CS_FILTER_SHIFT
如果某物理相本拍无效：
    filter_phase 保持或缓慢衰减，不使用本拍 ADC
```

初始建议：

```c
#define CS_FILTER_SHIFT 4U
```

情况三只应作为高调制区短时兜底。若长期进入情况三，说明调制过深或采样窗口设置太保守，应降额或调整 PWM/SOC。

## CMPB Placement

每个区域都使用“区域中部 + 边距钳位”的原则：

```text
cmpb_tick = region_start + region_width / 2
cmpb_tick >= region_start + CS_EDGE_SETTLE_TICK
cmpb_tick <= region_end - CS_TAIL_MARGIN_TICK
```

需要在 CMS32 上确认 CMPB 的比较方向。当前工程的 `EPWM_ConfigCompareTriger()` 支持 `EPWM_CMPTG_0/1`，实际可以继续用现有 CMP0/CMP1 资源，只是命名上按 TI 叫 CMPB。

建议第一版仍使用单点采样：

```c
#define CS_MULTI_EN 0U
```

等 T1/T2/T3 分区正确后，再恢复双点采样做 spread 诊断。

## ADC Channel Selection

Case 1：

```text
ADC channel mask = U | V | W
last interrupt channel = 最高 ADC channel
```

Case 2：

```text
ADC channel mask = physical(U) | physical(V)
last interrupt channel = U/V 中较高 ADC channel
```

Case 3：

```text
ADC channel mask = physical(U)
last interrupt channel = physical(U)
```

当前 CMS32 ADC 扫描多通道有时间偏移。Case 1 三相扫描必须确认 T1 足够容纳三次转换，否则应把 `TMinWidth` 加大。

## Ozone Watch Fields

必须新增或复用以下字段：

```text
sample_region        // 1=T1, 2=T2, 3=T3
sample_recon_mode    // 0=none, 1=one phase, 2=filter
sample_valid_mask
sample_ignore_mask
sample_cmpb_tick
sample_t1
sample_t2
sample_t3
sample_filter_u/v/w
sample_case3_count
sample_reconstruct_count
```

保留现有字段：

```text
sample_pair
sample_common_window
sample_tick_a/b
sample_spread0/1
sample_switch_count
sample_hold_count
```

## Implementation Phases

### Phase 1: Planner And Watch

只实现排序、T1/T2/T3 计算、case 判定和 watch 字段，暂时不改变 ADC 配置。

完成标准：

```text
sample_region 能随调制区变化
T1/T2/T3 数值与 duty 关系可解释
case3_count 只在高调制区出现
```

### Phase 2: ADC Channel And CMPB Routing

按 case 配置 ADC channel mask 和 CMPB tick。

完成标准：

```text
Case 1 三相都有正弦趋势
Case 2 忽略相不再贴零进入控制电流
Case 3 只用一相真实值，另外两相来自 filter
```

### Phase 3: Filter Backup

实现 `CS_RECON_FILTER`，并让滤波值只由有效采样相更新。

完成标准：

```text
进入 Case 3 时电流波形连续但更平滑
离开 Case 3 后真实 ADC 能重新接管滤波值
```

### Phase 4: Safety Gate

如果 Case 3 连续时间过长或滤波/真实电流偏差过大，限制电压或 fault。

建议初始阈值：

```c
#define CS_CASE3_MAX_CONSECUTIVE_PWM 64U
#define CS_FILTER_ERR_LIMIT_CNT 150
```

## Initial Config

16 kHz 第一版建议：

```c
#define CS_TMIN_WIDTH_TICK (PWM_DEADTIME_TICKS + 160U)
#define CS_MULTI_EN 0U
#define CS_FILTER_SHIFT 4U
#define CS_CASE3_MAX_CONSECUTIVE_PWM 64U
```

先把 TI 三种情况跑通，再打开双点采样、pair 滞回和更激进的电流环。
