# Board 层薄 C++ 封装动手指南

本文是后续 Board 层 C++ 教学文档。Board 层直接碰寄存器和时序，所以它是最后迁移的部分。

第一版只练习薄 wrapper：

```text
UART policy
PWM duty value object
ADC sample window helper
```

不要第一批迁移：

```text
Firmware/Board/Src/foc_curr.c
Firmware/Board/Src/foc_pwm.c 整体类化
Firmware/ThirdParty/**
vendor driver
```

## 总规则

Board wrapper 必须满足：

```text
不隐藏寄存器副作用
不在快环路径做动态分发
不使用异常/RTTI/virtual
不分配动态内存
不替换 vendor driver
不把 Board 细节推回 MotorControl
```

你写的时候心里只记一句话：

```text
Board C++ 只能变薄，不能变魔法。
```

## pwm_values.hpp 教学程序

PWM duty 最容易混的是单位和范围。先写 value object，不碰寄存器。

```cpp
#pragma once

#include <stdint.h>

#include "BoardConfig.h"
#include "clamp.hpp"

namespace cms32::board
{

struct PwmDuty
{
    uint16_t value;
};

struct ThreePhaseDuty
{
    PwmDuty u;
    PwmDuty v;
    PwmDuty w;
};

constexpr PwmDuty clamp_pwm_duty(uint16_t duty) noexcept
{
    return PwmDuty{static_cast<uint16_t>(cms32::support::clamp<uint16_t>(
        duty, PWM_DUTY_MIN, PWM_DUTY_MAX))};
}

constexpr ThreePhaseDuty safe_center_duty() noexcept
{
    return ThreePhaseDuty{
        PwmDuty{PWM_DUTY_50},
        PwmDuty{PWM_DUTY_50},
        PwmDuty{PWM_DUTY_50},
    };
}

static_assert(PWM_DUTY_MIN < PWM_DUTY_50, "invalid PWM min");
static_assert(PWM_DUTY_50 < PWM_DUTY_MAX, "invalid PWM max");

} // namespace cms32::board
```

这个文件不应该调用 `pwm_set_duty()`。它只表达 duty 数值规则。

## adc_sample_window.hpp 教学程序

ADC 采样窗口先只做边界检查，不调寄存器。

```cpp
#pragma once

#include <stdint.h>

#include "BoardConfig.h"

namespace cms32::board
{

struct AdcTriggerTick
{
    uint16_t value;
};

constexpr bool adc_trigger_inside_pwm_period(AdcTriggerTick tick) noexcept
{
    return (tick.value > PWM_DEADTIME_TICKS) && (tick.value < PWM_PERIOD);
}

constexpr AdcTriggerTick default_adc_trigger_tick() noexcept
{
    return AdcTriggerTick{PWM_ADC_TRIGGER_TICK_DEFAULT};
}

static_assert(adc_trigger_inside_pwm_period(default_adc_trigger_tick()),
              "default ADC trigger is outside PWM period");

} // namespace cms32::board
```

这类 helper 的价值是把板级固定约束写成编译期检查。它不应该变成复杂 HAL。

## uart_policy.hpp 教学程序

UART policy 只表达“哪个实例、哪个波特率”，不要在构造函数里改寄存器。

```cpp
#pragma once

#include <stdint.h>

#include "BoardConfig.h"

namespace cms32::board
{

enum class UartInstance : uint8_t
{
    Uart0,
};

template <UartInstance Instance> struct UartPolicy;

template <> struct UartPolicy<UartInstance::Uart0>
{
    static constexpr uint32_t baud = BOARD_UART_BAUD;
    static constexpr bool enabled = BOARD_UART_ENABLE != 0U;
};

static_assert(UartPolicy<UartInstance::Uart0>::baud > 0U, "invalid UART baud");

} // namespace cms32::board
```

后续如果写 `uart_driver_cms32.cpp`，它可以使用这个 policy，但寄存器初始化仍然要显式、可读。

## 单独编译检查

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/Board/Cpp \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_board_wrapper_check.o - <<'CPP'
#include "pwm_values.hpp"
#include "adc_sample_window.hpp"
#include "uart_policy.hpp"

static_assert(cms32::board::adc_trigger_inside_pwm_period(
                  cms32::board::default_adc_trigger_tick()),
              "bad adc trigger");

int main()
{
    const auto duty = cms32::board::safe_center_duty();
    return duty.u.value;
}
CPP
```

## 什么时候可以接入 Board

可以接入的条件：

```text
wrapper 只表达值和配置，不隐藏寄存器写
完整固件构建通过
上板前后 PWM/ADC/UART 行为一致
```

不要接入的情况：

```text
想把 foc_curr.c 整体模板化
想把 vendor driver 改成 C++
想用大型寄存器 DSL
想让 MotorControl 直接 include Board C++ 细节
```

Board 层 C++ 的目标是减少单位和配置错误，不是把底层寄存器访问包得看不见。
