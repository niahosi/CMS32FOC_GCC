# Cpp Stage Implementation Guide

本文是 C++ 重构的分阶段练习文档。它和 `CppRefactorRoadmap.md` 的关系是：

```text
CppRefactorRoadmap.md         讲路线和模块顺序
CppStageImplementationGuide.md 讲每一阶段怎么写、为什么这么拆、完整代码长什么样
SupportCppHandsOnGuide.md      专门讲 Firmware/Support/*.hpp
```

这里的代码是“参考形态”，不是要求马上全量替换当前 C 文件。当前原则仍然是：

```text
保持 C ABI
不碰动态内存
不引入异常/RTTI
不在快环使用 virtual
不为了封装而封装
```

## 模块层级判断

先回答一个很重要的问题：`ScrewAxis` 为什么单独成模块？回零程序和 `ScrewAxis` 是不是同等级？

推荐判断：

```text
ScrewAxis 是“轴级应用模块”
Home/Homing 是 ScrewAxis 里面的一个“行为小组件”
MotorControl 是底层电机控制模块
Comm 是通信模块
Board 是板级硬件模块
Support 是无业务工具模块
```

所以它们不是同等级：

```text
App/ScrewAxis
  -> HomeRuntime
  -> home_update()
  -> command/status helper
  -> later: ScrewHomeController / ScrewPositionTracker

MotorControl
  -> CurrentLoop
  -> SpeedLoop
  -> EncoderEstimator

Comm
  -> UartRx
  -> FrameParser
  -> CommandRouter
```

`ScrewAxis` 单独存在的原因是：它描述的是“这根螺杆轴”这个业务对象。它知道当前轴怎么回零、怎么记录零位、怎么把轴级命令转换成 `MotorControl` 的速度/电流命令。

回零不是和 `ScrewAxis` 同级。回零是 `ScrewAxis` 的一种行为。现在只有回零时，可以先放在 `screw_axis.cpp` 内部；当出现下面任意情况，就值得拉成小模块：

```text
回零状态机超过 150~200 行
还有软限位/位置跟踪/通信命令仲裁
Home 的状态、参数、watch 开始影响 ScrewAxis 可读性
某段逻辑能用一句话命名，例如 ScrewHomeController
```

不建议一开始就把所有小函数都拆文件。拆模块的目的不是目录好看，而是让依赖关系清楚：

```text
ScrewAxis 负责协调
HomeRuntime 只保存回零运行态
command/status helper 只负责集中读写对外快照
CommandRouter 以后只负责命令来源仲裁
```

## Stage 1: Support 小组件

这一阶段已经单独写在：

```text
Docs/Architecture/SupportCppHandsOnGuide.md
```

你当前正在写的 `Firmware/Support/*.hpp` 就属于这一阶段。

这一阶段要练熟这些 C++ 特性：

```text
constexpr
template
static_assert
enum class helper
固定数组
RAII 小对象
```

最小验收标准：

```text
每个 .hpp 都能单独被 C++ 编译器 include
不出现 new/delete/throw/typeinfo/vtable
不依赖业务模块内部状态
```

## Stage 2: ScrewAxis 单文件轻量 C++ 化

这一阶段不要写复杂架构。目标只是把 `screw_axis.c` 变成一个更清楚的
`screw_axis.cpp`：

```text
screw_axis.h    保持 C ABI、Command/Status 结构体、Ozone 可见入口
screw_axis.cpp  内部用少量 C++ 特性整理状态机
```

对外入口继续不变：

```cpp
extern "C" void ScrewAxis_Init(void);
extern "C" void ScrewAxis_Run(void);
extern "C" void ScrewAxis_OnAdcSample(void);
```

第一版只做这些事：

```text
宏参数 -> constexpr
SCREW_HOME_STATE_* 内部映射成 enum class HomeState
clamp_s16/clamp_u16 -> support::clamp
内部状态集中到 HomeRuntime
Status 只保留稳定字段，调试遥测先不放
重复写 command/status 的地方收成少量 helper
```

第一版不要做这些事：

```text
不要拆 ScrewHomeController.hpp/.cpp
不要写 HomeCommandView / MotorFeedbackView / Adapter
不要给每个字段写 getter/setter
不要把 g_motor_command 封装成复杂对象
不要动 MotorControl 快环和 main.c 入口
```

### Status 先瘦身

`ScrewHomeStatus_t` 不要一开始就放很多观测字段。第一版建议只保留外部真正要依赖的稳定状态：

```c
typedef struct
{
    uint8_t busy;
    uint8_t homed;
    uint8_t fault_seen;
    uint8_t state;
    uint16_t active_seq;
    int32_t zero_encoder_pos;
    int32_t pos_counts;
} ScrewHomeStatus_t;
```

先不要放这些：

```text
active_speed_rpm
active_slow_speed_rpm
active_backoff_speed_rpm
elapsed_ms
remaining_ms
stall_elapsed_ms
```

这些属于调试遥测，不是稳定状态。真需要时以后单独加：

```c
typedef struct
{
    int16_t active_speed_rpm;
    uint16_t elapsed_ms;
    uint16_t remaining_ms;
    uint16_t stall_elapsed_ms;
} ScrewHomeTelemetry_t;
```

这样 `screw_axis.cpp` 不会被一堆 watch 赋值淹没。

### 文件结构

当前推荐只有两个文件：

```text
Firmware/App/screw_axis.h
Firmware/App/screw_axis.cpp
```

以后真的复杂了，再考虑拆：

```text
Firmware/App/ScrewAxis/
├── screw_axis.cpp
├── screw_home_controller.hpp
└── screw_position_tracker.hpp
```

拆文件的条件不是“看起来更面向对象”，而是：

```text
回零状态机超过 150~200 行
开始加入软限位/位置模式/多命令来源仲裁
某段逻辑能独立命名，并且有自己的状态
```

### screw_axis.cpp 推荐骨架

下面只保留第一版需要的最小骨架，不再展开完整状态机：

```cpp
namespace {
enum class HomeState : uint8_t { Idle, FastRetract, FastBackoff, SlowRetract, FinalBackoff, Done, Fault };

struct HomeRuntime {
    HomeState state{HomeState::Idle};
    uint16_t last_start_seq{0U};
    uint32_t phase_start_ms{0U};
};

HomeRuntime s_home;
volatile uint32_t s_adc_samples;

uint32_t app_millis() noexcept;
void publish_state(HomeState state) noexcept;
void command_speed(int16_t speed_rpm) noexcept;
void stop_motor(uint8_t keep_enabled) noexcept;
void start_home(uint32_t now_ms) noexcept;
bool update_home() noexcept;
}

extern "C" void ScrewAxis_Init(void);
extern "C" void ScrewAxis_Run(void);
extern "C" void ScrewAxis_OnAdcSample(void);
```

这就是第一版该长的样子：

```text
只有一个 .cpp，没有额外类文件
没有 View/Adapter 层
没有 getter/setter
全局 command/status 仍然直接可见，方便 Ozone 调试
Status 字段少，不会在每个状态里反复写一堆观测量
重复的电机命令和状态发布被收成小 helper
状态机仍然是 switch，嵌入式工程师一眼能看懂
```

### 什么时候再拆模块

先不要拆。等出现下面情况再拆：

```text
回零以外又加入位置模式/软限位
串口、Ozone、自动流程都要抢写命令，需要命令仲裁
HomeRuntime 字段继续增加，update_home() 超过 200 行
某段逻辑可以独立测试，例如 PositionTracker 或 CommandRouter
```

那时再把：

```text
HomeRuntime + update_home()
```

抽成：

```text
ScrewHomeController
```

现在不用。

## Stage 3: 串口 Comm 第一版

串口模块建议从一开始就拉出来，原因是它的实时边界和业务边界都很清楚：

```text
UART ISR 只接字节
RingBuffer 缓冲
主循环解析协议
CommandRouter 写 g_motor_cmd/g_screw_home_cmd
```

### serial_protocol.hpp 完整示例

第一版协议尽量固定长度或小 payload，不用动态内存。

```cpp
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cms32::comm {

enum class CommandId : uint8_t {
    SetSpeedRpm = 0x01U,
    StartHome = 0x10U,
    StopHome = 0x11U,
    ReadMotorWatch = 0x80U,
    ReadHomeWatch = 0x81U,
};

enum class ParseResult : uint8_t {
    NeedMore,
    FrameReady,
    BadHeader,
    BadLength,
    BadChecksum,
};

template <size_t MaxPayload>
struct Frame {
    static_assert(MaxPayload <= 64U, "keep protocol payload small");

    CommandId id{};
    uint8_t payload[MaxPayload]{};
    uint8_t length{0U};
};

template <size_t MaxPayload>
class FrameParser {
public:
    ParseResult feed(uint8_t byte, Frame<MaxPayload>& out) noexcept
    {
        switch (state_) {
        case State::WaitHeader0:
            if (byte != kHeader0) {
                return ParseResult::BadHeader;
            }
            checksum_ = byte;
            state_ = State::WaitHeader1;
            return ParseResult::NeedMore;

        case State::WaitHeader1:
            if (byte != kHeader1) {
                reset();
                return ParseResult::BadHeader;
            }
            checksum_ ^= byte;
            state_ = State::ReadCommand;
            return ParseResult::NeedMore;

        case State::ReadCommand:
            out.id = static_cast<CommandId>(byte);
            checksum_ ^= byte;
            state_ = State::ReadLength;
            return ParseResult::NeedMore;

        case State::ReadLength:
            if (byte > MaxPayload) {
                reset();
                return ParseResult::BadLength;
            }
            out.length = byte;
            index_ = 0U;
            checksum_ ^= byte;
            state_ = (byte == 0U) ? State::ReadChecksum : State::ReadPayload;
            return ParseResult::NeedMore;

        case State::ReadPayload:
            out.payload[index_++] = byte;
            checksum_ ^= byte;
            if (index_ >= out.length) {
                state_ = State::ReadChecksum;
            }
            return ParseResult::NeedMore;

        case State::ReadChecksum:
            if (byte != checksum_) {
                reset();
                return ParseResult::BadChecksum;
            }
            reset();
            return ParseResult::FrameReady;
        }

        reset();
        return ParseResult::BadHeader;
    }

    void reset() noexcept
    {
        state_ = State::WaitHeader0;
        index_ = 0U;
        checksum_ = 0U;
    }

private:
    enum class State : uint8_t {
        WaitHeader0,
        WaitHeader1,
        ReadCommand,
        ReadLength,
        ReadPayload,
        ReadChecksum,
    };

    static constexpr uint8_t kHeader0 = 0xA5U;
    static constexpr uint8_t kHeader1 = 0x5AU;

    State state_{State::WaitHeader0};
    uint8_t index_{0U};
    uint8_t checksum_{0U};
};

} // namespace cms32::comm
```

### command_router.hpp 完整示例

Router 是协议和业务之间的唯一桥。串口解析器不应该直接到处写全局变量。

```cpp
#pragma once

#include "MotorControl.h"
#include "screw_axis.h"
#include "serial_protocol.hpp"

namespace cms32::comm {

class CommandRouter {
public:
    bool route(const Frame<16>& frame) noexcept
    {
        switch (frame.id) {
        case CommandId::SetSpeedRpm:
            return handle_set_speed(frame);

        case CommandId::StartHome:
            g_screw_home_cmd.start_seq++;
            g_screw_home_cmd.stop = 0U;
            return true;

        case CommandId::StopHome:
            g_screw_home_cmd.stop = 1U;
            return true;

        case CommandId::ReadMotorWatch:
        case CommandId::ReadHomeWatch:
            // 第一版可以先只设置标志，发送侧在主循环里处理。
            return true;
        }

        return false;
    }

private:
    static int16_t read_i16_le(const uint8_t* p) noexcept
    {
        return static_cast<int16_t>(
            static_cast<uint16_t>(p[0]) |
            (static_cast<uint16_t>(p[1]) << 8U));
    }

    bool handle_set_speed(const Frame<16>& frame) noexcept
    {
        if (frame.length != 2U) {
            return false;
        }

        g_motor_cmd.enable = 1U;
        g_motor_cmd.control_mode = 2U;
        g_motor_cmd.speed_ref_rpm = read_i16_le(frame.payload);
        return true;
    }
};

} // namespace cms32::comm
```

### serial_app.cpp 完整示例

```cpp
#include "ring_buffer.hpp"
#include "serial_protocol.hpp"
#include "command_router.hpp"

namespace {

cms32::support::RingBuffer<uint8_t, 128> s_uart_rx;
cms32::comm::FrameParser<16> s_parser;
cms32::comm::Frame<16> s_frame;
cms32::comm::CommandRouter s_router;

} // namespace

extern "C" void Serial_OnRxByteFromIrq(uint8_t byte)
{
    (void)s_uart_rx.push_isr(byte);
}

extern "C" void Serial_Run(void)
{
    uint8_t byte = 0U;
    while (s_uart_rx.pop(byte)) {
        const cms32::comm::ParseResult result = s_parser.feed(byte, s_frame);
        if (result == cms32::comm::ParseResult::FrameReady) {
            (void)s_router.route(s_frame);
        }
    }
}
```

注意：

```text
ISR 入口只 push byte
Serial_Run 在主循环执行
第一版不在 ISR 里路由命令
发送 watch 可以后续另写 TxBuilder，不要混在 Parser 里
```

## Stage 4: MotorControl 慢环外壳

这一阶段先不动 ADC 快环。目标是把命令复制、模式判断、状态枚举变清楚。

### motor_control_types.hpp 完整示例

```cpp
#pragma once

#include <stdint.h>

namespace cms32::motor {

enum class ControlMode : uint8_t {
    Current = 1U,
    Speed = 2U,
    Vf = 3U,
};

enum class ControlState : uint8_t {
    Idle = 0U,
    ClosedLoop = 3U,
    Fault = 4U,
};

enum class FaultReason : uint8_t {
    None = 0U,
    Encoder = 1U,
    Current = 2U,
    Pwm = 3U,
    Command = 4U,
};

constexpr bool is_closed_loop_mode(ControlMode mode) noexcept
{
    return (mode == ControlMode::Current) || (mode == ControlMode::Speed);
}

} // namespace cms32::motor
```

### command_sanitizer.hpp 完整示例

这个小组件只做命令限幅。它不读硬件、不改状态机、不关中断。

```cpp
#pragma once

#include "MotorControl.h"
#include "clamp.hpp"

namespace cms32::motor {

class CommandSanitizer {
public:
    MotorControlCommand_t sanitize(const volatile MotorControlCommand_t& input) const noexcept
    {
        MotorControlCommand_t out{};

        out.enable = input.enable;
        out.control_mode = input.control_mode;
        out.id_ref = support::clamp<int16_t>(input.id_ref, -1000, 1000);
        out.iq_ref = support::clamp<int16_t>(input.iq_ref, -1000, 1000);
        out.speed_ref = input.speed_ref;
        out.speed_ref_rpm = support::clamp<int16_t>(input.speed_ref_rpm,
                                                    -5000, 5000);
        out.iq_limit = support::clamp<int16_t>(input.iq_limit, 0, 1000);
        out.current_kp = input.current_kp;
        out.current_ki = input.current_ki;
        out.speed_kp = input.speed_kp;
        out.speed_ki = input.speed_ki;
        out.current_v_limit = input.current_v_limit;
        out.open_loop_speed_ref = input.open_loop_speed_ref;
        out.vf_voltage = input.vf_voltage;
        out.if_id_ref = input.if_id_ref;
        out.if_iq_ref = input.if_iq_ref;
        out.open_loop_timeout_ms = input.open_loop_timeout_ms;
        out.elec_zero_trim = input.elec_zero_trim;
        out.voltage_theta_offset = input.voltage_theta_offset;
        return out;
    }
};

} // namespace cms32::motor
```

注意：

```text
CommandSanitizer 是纯值处理
它可以被单元测试或单独编译检查
真正写入 s_mc.command 时再用 AdcIrqGuard 包很短临界区
```

## Stage 5: 快环小组件，而不是整条快环模板化

FOC 快环先不要整条 C++ 化。更稳的方式是替换内部小算法：

```text
速度目标斜坡 -> SlewLimiter<int16_t, Step>
速度反馈滤波 -> LowPassI32<Shift>
角度值 -> Angle16
PI 参数 -> FixedPiConfig
```

### fixed_pi.hpp 完整示例

```cpp
#pragma once

#include <stdint.h>

#include "clamp.hpp"

namespace cms32::motor {

template <uint8_t Shift, int16_t OutputLimit>
class FixedPi {
public:
    static_assert(Shift < 15U, "PI shift too large");
    static_assert(OutputLimit > 0, "output limit must be positive");

    int16_t update(int16_t error, int16_t kp, int16_t ki) noexcept
    {
        integral_ += static_cast<int32_t>(error) * ki;

        const int32_t raw =
            (static_cast<int32_t>(error) * kp + integral_) >> Shift;
        const int32_t limited =
            support::clamp<int32_t>(raw, -OutputLimit, OutputLimit);
        const int16_t out = static_cast<int16_t>(limited);

        // 最小 anti-windup：输出已经打满且误差还在同向推动时，回退本次积分。
        if (((out >= OutputLimit) && (error > 0)) ||
            ((out <= -OutputLimit) && (error < 0))) {
            integral_ -= static_cast<int32_t>(error) * ki;
        }

        return out;
    }

    void reset() noexcept
    {
        integral_ = 0;
    }

    int32_t integral() const noexcept
    {
        return integral_;
    }

private:
    int32_t integral_{0};
};

} // namespace cms32::motor
```

注意：

```text
这个示例用于理解模板和状态封装
真正替换现有 PI 前，要逐项对齐当前 C 算法的限幅和 anti-windup 行为
```

## Stage 6: Encoder/速度估算

Encoder 适合用强类型，但不要让类型层太厚。

### encoder_estimator.hpp 完整示例

```cpp
#pragma once

#include <stdint.h>

#include "low_pass.hpp"
#include "units.hpp"

namespace cms32::motor {

template <int32_t CountsPerRev, int32_t SampleHz, uint8_t FilterShift>
class SpeedEstimator {
public:
    static_assert(CountsPerRev > 0, "CountsPerRev must be positive");
    static_assert(SampleHz > 0, "SampleHz must be positive");

    void reset(support::EncoderRaw raw) noexcept
    {
        prev_raw_ = raw;
        speed_filter_.reset(0);
        initialized_ = true;
    }

    support::SpeedCounts update(support::EncoderRaw raw) noexcept
    {
        if (!initialized_) {
            reset(raw);
            return support::SpeedCounts{0};
        }

        const int16_t delta =
            static_cast<int16_t>(raw.value - prev_raw_.value);
        prev_raw_ = raw;

        const int32_t counts_per_second =
            static_cast<int32_t>(delta) * SampleHz;
        return support::SpeedCounts{speed_filter_.update(counts_per_second)};
    }

private:
    bool initialized_{false};
    support::EncoderRaw prev_raw_{0U};
    support::LowPassI32<FilterShift> speed_filter_{};
};

} // namespace cms32::motor
```

注意：

```text
这个估算器只关心 raw delta 和滤波
坏角过滤、MA600 重读、启动 blank 可以放在上层 validator
不要把 SPI 读传感器也塞进估算器
```

## 拆模块的最终建议

当前最合理的 App 层结构是先保持简单：

```text
Firmware/App/
├── main.c
├── screw_axis.h       C ABI，对 main/通信/Ozone 暴露
└── screw_axis.cpp     轴级协调、回零状态机、少量 helper
```

我的建议是：

```text
先把 HomeState enum class 和 constexpr 写进 screw_axis.cpp
先用 support::clamp 替掉本地 clamp_s16/clamp_u16
先把重复写 command/status 的代码收成小 helper
暂时不要拆目录
```

等状态机继续长大，再考虑：

```text
ScrewHomeController      回零状态机确实独立变大后再拆
ScrewPositionTracker     需要软限位/位置换算后再拆
CommandRouter            串口、Ozone、自动流程开始抢命令后再拆
```

判断一个模块该不该拉出来，用这三句话：

```text
它能不能用一个清楚的名词命名？
它有没有自己独立的状态？
它能不能只通过少量输入/输出和外界交互？
```

三个都是“是”，就可以拉出来。否则先留在原文件里，别为了漂亮目录制造迷宫。
