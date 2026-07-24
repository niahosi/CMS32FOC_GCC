# ScrewAxis 产品化与位置环动手指南

> 当前主固件状态：`Firmware/App/screw_axis.cpp` 只保留双端回零/行程标定程序；
> 本文中“位置环”和“往返测试”的完整代码是历史实验和后续重新设计时的学习材料，
> 当前不应按本文旧字段去写 Ozone 命令。当前真正接入主固件的是 MotorControl
> 电机级 `MC_MODE_POSITION = 6`，它以 `g_motor.encoder.pos` 为绝对位置反馈。

本文讲回零已经可用以后，怎么把 `ScrewAxis` 从“调试回零程序”升级成量产可维护的轴控制层。当前分层原则已经调整为：MotorControl 可以提供电机级绝对位置闭环；ScrewAxis 仍负责机械回零、相对坐标、软限位和轴级故障。

当前链路应该保持：

```text
ScrewAxis:
  双端回零 / 机械零点 / travel_counts / 软限位 / 轴级故障
  -> 未来把轴相对 target_counts 换算成绝对 position_ref

MotorControl:
  Position: position_ref - encoder.pos -> speed_ref
  Speed: speed_ref -> iq_ref

FOC:
  电流环
  -> PWM
```

这样分层的好处：

```text
MotorControl 不知道螺杆机械结构，只知道 encoder.pos
ScrewAxis 不直接碰 FOC 电流细节
位置环和速度环同 1 kHz 确定性 tick
回零、软限位、位置命令都在一个轴层管理
```

历史实验曾按这个方向落地过一版：

```text
Firmware/App/screw_axis.h
  统一入口：g_screw_axis_cmd / g_screw_axis_watch

Firmware/App/screw_axis.cpp
  HomeConfig
  PositionConfig
  双端回零状态机
  轴层位置 P 环
  安全往返速度递增测试
  ScrewAxis_Run() 统一调度 g_mc_cmd
```

当前主固件已经把位置 P 环和往返测试删除，只保留 `HomeConfig` 和双端回零状态机。
后续如果给 ScrewAxis 重新加入位置命令，不应恢复旧的轴层低频 P 环；更合适的方式是：

```text
relative_target_counts -> zero_encoder_pos + relative_target_counts
写入 g_mc_cmd.position_ref
g_mc_cmd.control_mode = MC_MODE_POSITION
MotorControl 在 1 kHz tick 上执行真正位置环
```

为什么删除旧入口：

```text
旧的独立回零 command/watch 只覆盖回零
g_screw_axis_cmd / g_screw_axis_watch 覆盖回零、位置和往返测试
单入口能避免 Ozone 或串口同时写两个命令源
回零详细阶段并入 g_screw_axis_watch.home_state
```

## 1. 先整理 ScrewAxis 配置

现在 `screw_axis.cpp` 里有一批零散 `constexpr`：

```cpp
kDefaultFastSpeedRpm
kDefaultSlowSpeedRpm
kDefaultBackoffSpeedRpm
kHomeTimeoutMs
kStallSpeedRpm
```

下一步可以先收成配置结构体。这样不是为了“抽象”，而是为了让参数一眼分组：

```cpp
namespace
{
using cms32::motor::SpeedLoopConfig;

static_assert(SpeedLoopConfig::ref_limit_rpm <= 32767L,
              "ScrewAxis rpm command must fit int16_t");

struct HomeConfig
{
    static constexpr int16_t max_speed_rpm = 500;
    static constexpr int16_t fast_retract_rpm = -2000;
    static constexpr int16_t slow_retract_rpm = -50;
    static constexpr int16_t extend_rpm = 2000;
    static constexpr int16_t backoff_rpm = 300;
    static constexpr int16_t iq_limit = 24;
    static constexpr uint16_t timeout_ms = 12000U;

    static constexpr int16_t stall_speed_rpm = 8;
    static constexpr int16_t stall_iq_margin = 2;
    static constexpr uint32_t fast_stall_hold_ms = 100U;
    static constexpr uint32_t slow_stall_hold_ms = 250U;
    static constexpr uint32_t stall_clear_hold_ms = 50U;
    static constexpr uint32_t fast_backoff_ms = 300U;
    static constexpr uint32_t final_backoff_ms = 300U;
};

struct PositionConfig
{
    static constexpr int32_t travel_min_counts = 0;
    static constexpr int32_t default_travel_max_counts = 0;
    static constexpr int16_t max_speed_rpm = 150;
    static constexpr int16_t settle_speed_rpm = 3;
    static constexpr int16_t speed_limit_max_rpm =
        static_cast<int16_t>(SpeedLoopConfig::ref_limit_rpm);
    static constexpr int32_t deadband_counts = 15729; // 约 30 um，按 1 mm = 524288 count 估算
    static constexpr int32_t kp_rpm_per_1000_counts = 1;
    static constexpr int16_t iq_limit = SpeedLoopConfig::iq_limit;
    static constexpr int16_t iq_limit_max = SpeedLoopConfig::iq_limit;
};
} // namespace
```

注意：

```text
HomeConfig 是回零流程配置
PositionConfig 是位置闭环配置
两者不要混在一起
PositionConfig::max_speed_rpm 是默认速度
PositionConfig::speed_limit_max_rpm 才是允许 Ozone 写入的最高速度
HomeConfig::iq_limit 默认 12，用于轻推找端
PositionConfig::iq_limit 跟随 CTRL_SPD_IQ_LIMIT，当前是 80，用于位置/往返测试
```

当前 `speed_limit_max_rpm` 直接来自 `SpeedLoopConfig::ref_limit_rpm`，也就是
`TuneConfig.h` 里的 `CTRL_SPD_REF_LIMIT_RPM`。这样速度环上限改成 3000 rpm
时，位置环测试上限也跟着变成 3000 rpm，不会藏一个 300 rpm 的第二限幅。

为什么默认值仍保留 150 rpm：

```text
默认值用于上电后没有特意写命令的情况
3000 rpm 是明确测试命令，不应该变成默认动作
位置环高速度测试前，必须确认回零、方向、软限位和急停都正常
```

## 2. 轴层状态和 MotorControl Position 不是一回事

MotorControl 的状态是电机控制状态：

```text
Idle
ClosedLoop
Fault
```

ScrewAxis 的状态是机械轴状态：

```text
Idle
HomeSeekOuter
HomeFastRetract
HomeSlowRetract
Ready
MovePosition
Fault
```

这两层不是同一种状态。现在可以使用 `MC_MODE_POSITION`，但它的含义是“电机级绝对位置闭环”，不是 ScrewAxis 的机械状态。更适合的关系是：

```text
MotorControl:
  MC_MODE_POSITION = 用 encoder.pos 做绝对位置闭环

ScrewAxis:
  Home / Ready / Fault / Move 这些轴状态继续留在 App 层
  负责把回零后的相对位置换算成绝对 position_ref
```

## 3. 完整参考代码：轴命令和状态

这是后续可以放进 `screw_axis.h` 的公共接口形状。它保留 C ABI，Ozone 也容易看。

```c
typedef enum
{
    ScrewAxisMode_Idle = 0U,
    ScrewAxisMode_Home = 1U,
    ScrewAxisMode_Position = 2U,
} ScrewAxisMode_t;

typedef enum
{
    ScrewAxisState_Idle = 0U,
    ScrewAxisState_Homing = 1U,
    ScrewAxisState_Ready = 2U,
    ScrewAxisState_Position = 3U,
    ScrewAxisState_Fault = 4U,
} ScrewAxisState_t;

typedef struct
{
    uint16_t home_start_seq;
    uint16_t move_start_seq;
    uint8_t stop;
    int32_t target_counts;
    int16_t max_speed_rpm;
    int16_t iq_limit;
    uint16_t sweep_start_seq;
    uint16_t sweep_cycle_limit;
    uint16_t sweep_dwell_ms;
    int32_t safe_margin_counts;
    int16_t sweep_start_speed_rpm;
    int16_t sweep_speed_step_rpm;
    int16_t home_speed_rpm;
    int16_t home_slow_speed_rpm;
    int16_t home_backoff_speed_rpm;
    uint16_t home_timeout_ms;
    int16_t home_iq_limit;
} ScrewAxisCommand_t;

typedef struct
{
    uint8_t state;
    uint8_t homed;
    uint8_t fault_seen;
    uint8_t moving;
    int32_t zero_encoder_pos;
    int32_t outer_encoder_pos;
    int32_t travel_counts;
    int32_t pos_counts;
    int32_t target_counts;
    int32_t pos_err_counts;
    int16_t speed_cmd_rpm;
    uint8_t sweep_active;
    uint8_t sweep_phase;
    uint16_t sweep_cycle_count;
    int16_t sweep_speed_rpm;
    int32_t safe_min_counts;
    int32_t safe_max_counts;
    int32_t safe_margin_counts;
    uint8_t home_state;
    uint16_t home_active_seq;
} ScrewAxisWatch_t;

extern volatile ScrewAxisCommand_t g_screw_axis_cmd;
extern volatile ScrewAxisWatch_t g_screw_axis_watch;
```

当前工程已经采用单入口：

```text
回零：
  写 g_screw_axis_cmd.home_* 参数
  递增 g_screw_axis_cmd.home_start_seq

位置：
  写 g_screw_axis_cmd.target_counts
  写 g_screw_axis_cmd.max_speed_rpm
  写 g_screw_axis_cmd.iq_limit
  递增 g_screw_axis_cmd.move_start_seq

往返测试：
  写 g_screw_axis_cmd.max_speed_rpm
  写 g_screw_axis_cmd.iq_limit
  写 g_screw_axis_cmd.sweep_start_speed_rpm
  递增 g_screw_axis_cmd.sweep_start_seq
```

`target_counts` 只对 `move_start_seq` 触发的位置命令有效。`sweep_start_seq` 触发的是
往返测试，它会忽略 `target_counts`，直接在 `safe_min_counts..safe_max_counts` 之间运动。

Ozone 观察优先看：

```text
g_screw_axis_watch.state
g_screw_axis_watch.homed
g_screw_axis_watch.fault_reason
g_screw_axis_watch.home_state
g_screw_axis_watch.travel_counts
g_screw_axis_watch.pos_counts
g_screw_axis_watch.target_counts
g_screw_axis_watch.pos_err_counts
g_screw_axis_watch.speed_cmd_rpm
```

如果 `fault_seen = 1`，先看 `fault_reason`：

```text
1: 底层 MotorControl fault
2: 正向找外端超时
3: 从外端快速收回找内端超时
4: 低速精找内端超时
5: 未回零时请求位置运动
6: 未回零或行程无效时请求往返测试
7: 往返测试安全窗口无效
```

## 4. 历史参考代码：轴层低频位置 P 环

位置环第一版建议只做 P，不加 I。原因：

```text
螺杆有摩擦和机械端点
积分容易在端点附近积累
速度环已经有 PI
第一版先验证方向、单位、限幅和软限位
```

下面这段是历史轴层 P 环参考，不是当前主固件推荐接入方式。当前主固件推荐
ScrewAxis 只做坐标换算和软限位，再写 `g_mc_cmd.position_ref` 交给
MotorControl 的 1 kHz Position 模式。

当前 `ScrewAxis_Run()` 在空闲/Ready 时不会持续覆盖 `g_mc_cmd`。回零完成后，
只要 `g_screw_axis_cmd.stop = 0`，就可以直接在 Ozone 里写 `g_mc_cmd` 测试
MotorControl 位置模式。

参考实现：

```cpp
struct PositionRuntime
{
    bool active{false};
    uint16_t last_move_seq{0U};
    int32_t target_counts{0};
    int16_t max_speed_rpm{PositionConfig::max_speed_rpm};
    int16_t iq_limit{PositionConfig::iq_limit};
};

int32_t current_position_counts() noexcept
{
    if (!s_home.homed)
    {
        return 0;
    }

    return g_motor.encoder.pos - s_home.zero_encoder_pos;
}

int16_t position_speed_command(int32_t error_counts, int16_t max_speed_rpm) noexcept
{
    if ((error_counts > -PositionConfig::deadband_counts) &&
        (error_counts < PositionConfig::deadband_counts))
    {
        return 0;
    }

    const int32_t raw_rpm =
        (error_counts * PositionConfig::kp_rpm_per_1000_counts) / 1000L;
    const int32_t limited =
        cms32::support::clamp<int32_t>(raw_rpm, -max_speed_rpm, max_speed_rpm);

    return static_cast<int16_t>(limited);
}

int16_t normalize_position_speed_limit(int16_t value) noexcept
{
    int16_t speed = abs_i16(value);
    if (speed == 0)
    {
        speed = PositionConfig::max_speed_rpm;
    }
    return cms32::support::clamp<int16_t>(speed,
                                          1,
                                          PositionConfig::speed_limit_max_rpm);
}

int32_t clamp_target_counts(int32_t target) noexcept
{
    const int32_t max_counts = s_home.outer_seen
                                   ? (s_home.outer_encoder_pos - s_home.zero_encoder_pos)
                                   : PositionConfig::default_travel_max_counts;

    if (max_counts <= 0)
    {
        return 0;
    }

    return cms32::support::clamp<int32_t>(target,
                                          PositionConfig::travel_min_counts,
                                          max_counts);
}

void command_motor_speed(int16_t speed_rpm, int16_t iq_limit) noexcept
{
    g_mc_cmd.enable = 1U;
    g_mc_cmd.control_mode = 2U;
    g_mc_cmd.id_ref = 0;
    g_mc_cmd.iq_ref = 0;
    g_mc_cmd.speed_ref = 0;
    g_mc_cmd.speed_ref_rpm = speed_rpm;
    g_mc_cmd.iq_limit = iq_limit;
}

void update_position() noexcept
{
    if (!s_home.homed)
    {
        s_pos.active = false;
        return;
    }

    if (g_screw_axis_cmd.move_start_seq != s_pos.last_move_seq)
    {
        s_pos.last_move_seq = g_screw_axis_cmd.move_start_seq;
        s_pos.target_counts = clamp_target_counts(g_screw_axis_cmd.target_counts);
        s_pos.max_speed_rpm =
            normalize_position_speed_limit(g_screw_axis_cmd.max_speed_rpm);
        s_pos.iq_limit = cms32::support::clamp<int16_t>(g_screw_axis_cmd.iq_limit, 1, 16);
        s_pos.active = true;
    }

    if (!s_pos.active)
    {
        return;
    }

    const int32_t pos_counts = current_position_counts();
    const int32_t error_counts = s_pos.target_counts - pos_counts;
    const int16_t speed_rpm = position_speed_command(error_counts, s_pos.max_speed_rpm);

    command_motor_speed(speed_rpm, s_pos.iq_limit);

    if (speed_rpm == 0)
    {
        s_pos.active = false;
    }
}
```

这段代码的关键点：

```text
位置环只产生速度命令
目标位置先按 travel_counts 做软限位
没有 homed 不允许位置运动
接近目标后速度命令归零
```

当前实现还有两个小保护：

```text
max_speed_rpm 写 0 时回退默认速度
iq_limit 写 0 时回退默认电流限幅
target_counts 永远限制在 0..travel_counts
```

如果还没有回零，位置命令会被消费但不会运动。这样可以避免“先写了位置命令，后来一回零完成突然自己跑起来”。

## 5. 完整参考代码：ScrewAxis_Run 优先级

轴层命令必须有优先级。推荐：

```text
stop
MotorControl fault
home
sweep
position
idle
```

参考结构：

```cpp
extern "C" void ScrewAxis_Run(void)
{
    if (g_screw_axis_cmd.stop != 0U)
    {
        stop_motor(0U);
        s_pos.active = false;
        publish_axis_watch();
        return;
    }

    if ((g_motor.runtime.state == MC_STATE_FAULT) || (g_motor.runtime.fault != 0U))
    {
        stop_motor(0U);
        s_pos.active = false;
        s_axis_state = AxisState::Fault;
        publish_axis_watch();
        return;
    }

    if (home_is_active_or_requested())
    {
        update_home();
        publish_axis_watch();
        return;
    }

    if (sweep_is_active_or_requested())
    {
        update_sweep();
        publish_axis_watch();
        return;
    }

    if (position_is_active_or_requested())
    {
        update_position();
        publish_axis_watch();
        return;
    }

    stop_motor(0U);
    publish_axis_watch();
}
```

不要让位置命令、往返测试和回零命令同时写 `g_mc_cmd`。谁写速度命令必须由
`ScrewAxis_Run()` 统一调度。

## 6. 安全往返速度递增测试

当前程序新增了一个 sweep 入口，用于你现在要做的“找到两端以后，在安全范围内来回跑，
并且逐级提高速度”。

它不是重新找端点。它的顺序是：

```text
1. 先执行双端回零
2. 回零记录 zero_encoder_pos 和 outer_encoder_pos
3. 计算 travel_counts = outer_encoder_pos - zero_encoder_pos
4. 计算安全运动范围 safe_min_counts..safe_max_counts
5. 在 safe_min_counts 和 safe_max_counts 之间往返
6. 每完成一个完整来回后增加 sweep_speed_rpm
```

为什么要这样做：

```text
高速位置运动不应该用堵转当刹车
堵转只用于低速标定两端位置
高速阶段只允许跑到离两端有余量的位置
实际能否做到 1.4 s 一个来回，要看速度斜坡、导程、负载和减速距离
```

新增命令字段在 `g_screw_axis_cmd` 末尾：

```c
uint16_t sweep_start_seq;
uint16_t sweep_cycle_limit;
uint16_t sweep_dwell_ms;
int32_t safe_margin_counts;
int16_t sweep_start_speed_rpm;
int16_t sweep_speed_step_rpm;
```

这些字段的含义：

```text
sweep_start_seq:
  每递增一次，启动一次往返测试。

sweep_cycle_limit:
  完整来回次数。写 0 使用默认 2 次。

sweep_dwell_ms:
  到达每个安全端点后的停顿时间。写 0 使用默认 120 ms。

safe_margin_counts:
  离两端堵转点保留多少 encoder count。
  写 0 时自动计算，默认取 travel_counts / 10，并且不少于 4 倍位置死区。

sweep_start_speed_rpm:
  起始速度。写 0 使用默认 300 rpm。

sweep_speed_step_rpm:
  每完成一个完整来回后增加多少 rpm。写 0 使用默认 300 rpm。
```

这里最容易混淆的是 `max_speed_rpm` 和 `sweep_start_speed_rpm`：

```text
max_speed_rpm:
  位置 P 环和往返测试的速度上限。
  它不是“本次往返测试立刻用多少速度”。

sweep_start_speed_rpm:
  往返测试第一趟真正锁存的速度。
  如果你只把 max_speed_rpm 写成 1500，但 sweep_start_speed_rpm 仍是默认 300，
  那么 g_screw_axis_watch.speed_cmd_rpm 看到 300 是正常的。
```

新增观察字段：

```text
g_screw_axis_watch.sweep_active
g_screw_axis_watch.sweep_phase
g_screw_axis_watch.sweep_cycle_count
g_screw_axis_watch.sweep_speed_rpm
g_screw_axis_watch.safe_min_counts
g_screw_axis_watch.safe_max_counts
g_screw_axis_watch.safe_margin_counts
```

往返阶段枚举：

```c
ScrewSweepPhase_Idle = 0
ScrewSweepPhase_ToOuter = 1
ScrewSweepPhase_DwellOuter = 2
ScrewSweepPhase_ToInner = 3
ScrewSweepPhase_DwellInner = 4
ScrewSweepPhase_EnterWindow = 5
```

`EnterWindow` 是高速往返前的准备阶段。比如回零结束后 `pos_counts` 可能只有几万
count，而 `safe_min_counts` 约为 50 万 count；这时如果直接用 1500 rpm 冲向
`safe_max_counts`，很容易在内端附近大电流起步并触发保护。当前程序会先用 300 rpm
进入最近的安全端点，再开始正式高速往返。

往返测试新增两个故障原因：

```c
ScrewAxisFault_SweepOuterStall = 8
ScrewAxisFault_SweepInnerStall = 9
```

这两个 fault 的意思不是“程序完成找端”，而是“安全往返测试本来不该碰到机械端，
但现在已经堵转了”。发生后程序会停机，优先检查：

```text
safe_min_counts / safe_max_counts 是否真的落在机械行程内部
travel_counts 是否为正且数值可信
pos_counts 的方向是否和实际伸出方向一致
target_counts 是否越过了安全端点
g_motor.speed.iq_ref.value 是否长期顶到 iq_limit
```

一次安全测试可以这样写：

```text
1. 先回零，等 g_screw_axis_watch.homed = 1。
2. 记录 g_screw_axis_watch.travel_counts，判断两端实测行程。
3. 设置 g_screw_axis_cmd.max_speed_rpm = 1500。
4. 设置 g_screw_axis_cmd.iq_limit = 80。
5. 如果想第一趟就是 1500 rpm，设置 g_screw_axis_cmd.sweep_start_speed_rpm = 1500。
6. 如果想从低速逐级上升，设置 sweep_start_speed_rpm = 300，sweep_speed_step_rpm = 300。
7. 设置 g_screw_axis_cmd.sweep_cycle_limit = 2。
8. 设置 g_screw_axis_cmd.sweep_dwell_ms = 120。
9. safe_margin_counts 先写 0，让程序自动留 10% 行程。
10. 递增 g_screw_axis_cmd.sweep_start_seq。
```

如果只是想固定 1500 rpm 来回测试，可以这样写：

```text
g_screw_axis_cmd.max_speed_rpm = 1500
g_screw_axis_cmd.iq_limit = 80
g_screw_axis_cmd.sweep_start_speed_rpm = 1500
g_screw_axis_cmd.sweep_speed_step_rpm = 300
g_screw_axis_cmd.sweep_cycle_limit = 2
g_screw_axis_cmd.safe_margin_counts = 0
g_screw_axis_cmd.sweep_start_seq++
```

如果刚回零后立刻启动 sweep，观察到 `sweep_phase = 5` 是正常的：

```text
sweep_phase = 5:
  低速进入 safe_min/safe_max 安全窗口。

sweep_phase = 1/3:
  正式高速向外/向内运动。
```

这里 `sweep_speed_step_rpm = 300` 不会继续超过 1500，因为每次加速后都会被
`max_speed_rpm` 截住。

如果 10 mm 是总行程，默认安全边距约等于：

```text
safe_margin = travel_counts / 10
内端安全目标 = 1 mm 附近
外端安全目标 = 9 mm 附近
实际运动距离 = 8 mm 单程
```

这样高速测试不会跑到堵转点的前提是速度斜坡足够快。当前速度目标斜坡已经从
`2000 rpm/s` 提高到 `30000 rpm/s`，原因是短行程丝杠没有足够距离给慢斜坡减速。

1500 rpm 的减速距离估算：

```text
counts_per_rev = 262144
decel_time = rpm / ramp
decel_counts = rpm * counts_per_rev / 60 * decel_time / 2

旧 ramp = 2000 rpm/s:
  decel_time = 1500 / 2000 = 0.75 s
  decel_counts ~= 2,457,600 count
  约等于 4.9 mm，远大于 1 mm 安全边距，必然容易顶头。

当前 ramp = 30000 rpm/s:
  decel_time = 1500 / 30000 = 0.05 s
  decel_counts ~= 163,840 count
  约等于 0.31 mm，适合先验证 3000 rpm 位置往返能力。
```

真正要压缩到 1.4 s 一个来回时，应该同时检查 `safe_margin_counts`、
`max_speed_rpm`、`CTRL_SPD_REF_RAMP_RPM_PER_S` 和机械冲击，但每次只改一个变量。

推荐逐级：

```text
max_speed_rpm = 500，确认方向和停止
max_speed_rpm = 1000，确认无明显过冲
max_speed_rpm = 1500，确认 iq_ref 不长期顶限
max_speed_rpm = 2000，观察机械冲击
max_speed_rpm = 3000，只在急停和余量都确认后测试
```

判断当前限制来自哪里：

```text
speed_cmd_rpm 达不到 sweep_speed_rpm:
  位置误差不够大，P 环没有打满速度。

g_motor.speed.ref_active.value 上升慢:
  速度给定斜坡 CTRL_SPD_REF_RAMP_RPM_PER_S 限制。

g_motor.speed.feedback 跟不上 ref_active:
  速度 PI、电流限幅、负载或机械摩擦限制。

g_motor.speed.iq_ref.value 长期等于 iq_limit:
  扭矩已经打满，不要继续加速。

fault_reason = 8 或 9:
  程序已经在安全往返阶段检测到堵转并停机。
  如果 pos_counts 还远小于 target_counts 就 fault，多半是 iq_limit 太小、负载卡住或加速太猛。
  如果 pos_counts 已接近 safe_min/safe_max 才 fault，再怀疑安全边距太小、行程记录不可信、
  方向约定反了，或者当前速度斜坡下减速距离仍然不够。
```

往返测试的到达判定和普通位置定位不完全一样：

```text
普通 position:
  使用较小 deadband_counts，目标是停到指定位置附近。

sweep:
  使用 SweepConfig::target_tolerance_counts。
  当前为 PositionConfig::deadband_counts * 12。
  原因是高速往返验证关注的是“安全范围内能否来回”，不是最后几十微米的定位精度。
  接近 safe_min/safe_max 后如果继续用很小速度爬行，容易被静摩擦卡住并误判堵转。

堵转保护:
  sweep 使用 SweepConfig::stall_hold_ms。
  当前为 800 ms，比回零粗找端的 100 ms 更宽。
  原因是高速往返时短时间速度掉到 0 或 iq_ref 顶限，不一定代表机械端点，可能只是加减速、
  摩擦或负载瞬态。
```

## 7. 上电调试顺序

建议这样测：

```text
1. 只跑双端回零，确认 homed=1，并记录 travel_counts。
2. target_counts = travel_counts / 4，小速度移动。
3. target_counts = travel_counts / 2。
4. target_counts = 0，确认能回到内端附近但不撞死。
5. target_counts = travel_counts，确认软限位不会超过外端。
```

Ozone 重点看：

```text
g_screw_axis_watch.homed
g_screw_axis_watch.travel_counts
g_screw_axis_watch.pos_counts
g_screw_axis_watch.target_counts
g_screw_axis_watch.pos_err_counts
g_screw_axis_watch.speed_cmd_rpm
g_motor.command.speed.speed_ref
g_motor.speed.feedback
g_motor.speed.iq_ref.value
g_motor.current.dq.q
```

当前第一版位置测试建议：

```text
1. 递增 g_screw_axis_cmd.home_start_seq。
2. 等 g_screw_axis_watch.homed = 1。
3. 记录 g_screw_axis_watch.travel_counts。
4. 设置 target_counts = travel_counts / 4。
5. 设置 max_speed_rpm = 50，iq_limit = 12 到 30。
6. 递增 g_screw_axis_cmd.move_start_seq。
7. 看 speed_cmd_rpm 是否为正，pos_err_counts 是否逐渐变小。
8. 再测试 target_counts = 0 和 target_counts = travel_counts。
```

现在普通位置命令也会套用软限位：

```text
target_counts = 0:
  实际会被锁到 safe_min_counts，不会直接顶内端机械零点。

target_counts = travel_counts:
  实际会被锁到 safe_max_counts，不会直接顶外端机械端点。

如果你只是想回到内端附近：
  不要递增 sweep_start_seq。
  设置 target_counts = 0。
  递增 move_start_seq。
```

如果低速方向和软限位都确认正确，可以逐步提高 `max_speed_rpm`：

```text
第一步：300 rpm，确认不会越界，停止点附近不明显振荡
第二步：1000 rpm，确认速度反馈能跟上，iq_ref 没有长期顶限
第三步：2000 rpm，观察减速距离和机械冲击
第四步：3000 rpm，只在行程足够长、急停可靠、端点不参与减速时测试
```

注意：`max_speed_rpm = 3000` 只是允许位置环输出最高 3000 rpm。真实速度还会受
速度环斜坡 `CTRL_SPD_REF_RAMP_RPM_PER_S`、速度 PI、电流限幅 `iq_limit`、负载和
机械行程影响。

当前代码把回零和位置/往返的电流限幅分开：

```text
回零:
  默认 home_iq_limit = 12
  最大允许到 16
  目的：轻推端点，减少机械压紧和冲击。

位置/往返:
  默认 iq_limit = CTRL_SPD_IQ_LIMIT，当前是 80
  目的：给速度环足够转矩，不要把正常加速误判成堵转。
```

## 8. 完成标准

```text
未回零时，位置命令不生效
回零后，target_counts 会被限制在 0..travel_counts
往返测试只在 safe_min_counts..safe_max_counts 内运动
位置环只写 speed_ref_rpm，不写 iq_ref/id_ref
MotorControl fault 时 ScrewAxis 立刻停机
回零优先级高于位置运动
Ozone watch 能解释每一个停机原因
```
