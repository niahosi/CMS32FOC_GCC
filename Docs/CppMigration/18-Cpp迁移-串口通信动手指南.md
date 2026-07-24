# 串口通信 C++ 动手指南

> 当前最终落地形态见 `25-Cpp迁移-C_Cpp混编最小封装落地指南.md`。串口后续只应写
> `g_mc_cmd` 或轴层命令入口，不直接访问 `g_mc_*` 快环内部状态。

本文是后续系统层 C++ 教学文档：在 Board UART 已经 bring-up 的基础上，用 C++ 写协议解析和命令路由。

第一版建议教学目录：

```text
Firmware/Comm/
├── serial_protocol.hpp
├── serial_protocol.cpp
├── command_router.hpp
└── command_router.cpp
```

Board 层 `board_uart.c` 继续负责 UART 寄存器、P06/P07 pinmux、ISR 字节收发。

## 总规则

Comm C++ 必须满足：

```text
ISR 只收字节，不解析业务
主循环解析完整帧
不使用动态内存
不使用异常/RTTI/virtual
不直接写 MotorControlCState
不阻塞等待 UART 发送完成
```

你写的时候心里只记一句话：

```text
Board 管字节，Comm 管协议，Router 管命令归属。
```

## serial_protocol.hpp 教学程序

这个示例协议只表达结构，不要求你立刻把它接入主循环。

```cpp
#pragma once

#include <stdint.h>

#include "ring_buffer.hpp"

namespace cms32::comm
{

enum class CommandId : uint8_t
{
    SetMotorMode = 1U,
    SetCurrent = 2U,
    SetSpeedRpm = 3U,
    StartHome = 4U,
    StopHome = 5U,
    ReadMotorWatch = 6U,
    ReadScrewHomeWatch = 7U,
};

enum class ParseResult : uint8_t
{
    NeedMore,
    FrameReady,
    BadHeader,
    BadLength,
    Overflow,
};

template <uint8_t MaxPayload> struct Frame
{
    static_assert(MaxPayload > 0U, "MaxPayload must be positive");

    CommandId command{CommandId::ReadMotorWatch};
    uint8_t length{0U};
    uint8_t payload[MaxPayload]{};
};

template <uint8_t MaxPayload> class FrameParser
{
public:
    ParseResult feed(uint8_t byte) noexcept
    {
        switch (state_)
        {
        case State::HeaderA:
            state_ = (byte == 0xA5U) ? State::HeaderB : State::HeaderA;
            return ParseResult::NeedMore;

        case State::HeaderB:
            state_ = (byte == 0x5AU) ? State::Command : State::HeaderA;
            return (state_ == State::Command) ? ParseResult::NeedMore : ParseResult::BadHeader;

        case State::Command:
            frame_.command = static_cast<CommandId>(byte);
            state_ = State::Length;
            return ParseResult::NeedMore;

        case State::Length:
            if (byte > MaxPayload)
            {
                reset();
                return ParseResult::BadLength;
            }
            frame_.length = byte;
            index_ = 0U;
            state_ = (byte == 0U) ? State::Done : State::Payload;
            return (byte == 0U) ? ParseResult::FrameReady : ParseResult::NeedMore;

        case State::Payload:
            frame_.payload[index_++] = byte;
            if (index_ >= frame_.length)
            {
                state_ = State::Done;
                return ParseResult::FrameReady;
            }
            return ParseResult::NeedMore;

        case State::Done:
            reset();
            return feed(byte);
        }

        reset();
        return ParseResult::BadHeader;
    }

    const Frame<MaxPayload>& frame() const noexcept
    {
        return frame_;
    }

    void reset() noexcept
    {
        state_ = State::HeaderA;
        index_ = 0U;
        frame_ = Frame<MaxPayload>{};
    }

private:
    enum class State : uint8_t
    {
        HeaderA,
        HeaderB,
        Command,
        Length,
        Payload,
        Done,
    };

    State state_{State::HeaderA};
    Frame<MaxPayload> frame_{};
    uint8_t index_{0U};
};

} // namespace cms32::comm
```

## command_router.hpp 教学程序

Router 不解析字节，不访问 UART，只把已经解析好的命令写到公共 C ABI。

```cpp
#pragma once

#include <stdint.h>

extern "C"
{
#include "MotorControl.h"
#include "screw_axis.h"
}

#include "serial_protocol.hpp"

namespace cms32::comm
{

class CommandRouter
{
public:
    void route(const Frame<8U>& frame) noexcept
    {
        switch (frame.command)
        {
        case CommandId::SetMotorMode:
            if (frame.length >= 2U)
            {
                g_motor_command.enable = frame.payload[0];
                g_motor_command.control_mode = frame.payload[1];
            }
            break;

        case CommandId::SetSpeedRpm:
            if (frame.length >= 2U)
            {
                const int16_t rpm =
                    static_cast<int16_t>(frame.payload[0] |
                                         (static_cast<uint16_t>(frame.payload[1]) << 8));
                g_motor_command.enable = 1U;
                g_motor_command.control_mode = 2U;
                g_motor_command.speed_ref_rpm = rpm;
            }
            break;

        case CommandId::StartHome:
            g_screw_axis_cmd.home_start_seq++;
            break;

        case CommandId::StopHome:
            g_screw_axis_cmd.stop = 1U;
            break;

        default:
            break;
        }
    }
};

} // namespace cms32::comm
```

## 为什么 Parser 和 Router 分开

```text
FrameParser:
  只关心字节流是否形成完整帧。

CommandRouter:
  只关心帧代表什么命令，写哪个 C ABI 入口。
```

这样以后换协议格式，不影响命令入口；以后换命令来源，也不影响 parser。

## 单独编译检查

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/Comm \
  -I Firmware/App \
  -I Firmware/MotorControl/Abi \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_comm_check.o - <<'CPP'
#include "serial_protocol.hpp"

int main()
{
    cms32::comm::FrameParser<8U> parser;
    (void)parser.feed(0xA5U);
    (void)parser.feed(0x5AU);
    return 0;
}
CPP
```

## 什么时候接入主循环

可以接入的条件：

```text
UART RX ring buffer 已经能稳定收字节
Parser 单独验证通过
Router 只写 g_mc_cmd / g_screw_axis_cmd
命令仲裁规则明确
```

不要接入的情况：

```text
想在 UART ISR 里解析完整协议
想让串口直接写 MotorControlCState
还没有决定 ScrewAxis 和串口同时写速度命令时谁优先
```

Comm 的第一目标是让调试命令更规整，不是把所有控制逻辑搬进串口。
