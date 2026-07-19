# MotorControl 数学小组件 C++ 动手指南

本文是 Stage 5 的第一篇动手文档：把 MotorControl 里最小、最纯、最容易验证的数学规则先写成 C++ 教学组件。

它不是让你立刻改 `motor_control_current.c`，而是先练习两个未来会用到的文件：

```text
Firmware/MotorControl/Cpp/fixed_pi.hpp
Firmware/MotorControl/Cpp/speed_math.hpp
```

这两个文件第一版都应该是 header-only。写完先单独编译检查，不接入 20 kHz 快环。

## 总规则

这些数学组件必须满足：

```text
不使用 new/delete
不使用 malloc/free
不使用异常
不使用 RTTI
不使用 virtual
不使用 std::vector/std::string/std::function/iostream
不访问寄存器
不读 MA600
不改 PWM
不直接写 MotorControlCState
```

你写的时候心里只记一句话：

```text
先把纯规则类型化，不改变当前控制行为。
```

## 为什么 fixed_pi.hpp 先包装 C PI

当前 C 算法已经有稳定接口：

```text
FocPi_t
foc_pi_init()
foc_pi_reset()
foc_pi_set_gains()
foc_pi_update()
```

第一版 `FixedPi` 不重写 PI 算法，只包装这些 C 函数。这样做的目的不是炫技，而是把调用点从“裸函数操作结构体”收成一个小对象：

```cpp
cms32::motor::FixedPi speed_pi;
speed_pi.init(kp, ki, -iq_limit, iq_limit, shift);
const int16_t iq = speed_pi.update(ref_rpm, fb_rpm);
```

这样后面迁移速度环时，PI 的状态和行为边界更清楚。

## fixed_pi.hpp 完整教学程序

```cpp
#pragma once

#include <stdint.h>

extern "C"
{
#include "foc_math.h"
}

namespace cms32::motor
{

struct FixedPiConfig
{
    int16_t kp;
    int16_t ki;
    int16_t output_min;
    int16_t output_max;
    uint8_t shift;
};

class FixedPi
{
public:
    void init(FixedPiConfig config) noexcept
    {
        foc_pi_init(&pi_,
                    config.kp,
                    config.ki,
                    config.output_min,
                    config.output_max,
                    config.shift);
    }

    void reset() noexcept
    {
        foc_pi_reset(&pi_);
    }

    void set_gains(FixedPiConfig config) noexcept
    {
        foc_pi_set_gains(&pi_,
                         config.kp,
                         config.ki,
                         config.output_min,
                         config.output_max,
                         config.shift);
    }

    int16_t update(int16_t ref, int16_t feedback) noexcept
    {
        return foc_pi_update(&pi_, ref, feedback);
    }

    FocPi_t& raw() noexcept
    {
        return pi_;
    }

    const FocPi_t& raw() const noexcept
    {
        return pi_;
    }

private:
    FocPi_t pi_{};
};

} // namespace cms32::motor
```

### 这段 C++ 在表达什么

`FixedPiConfig` 是一组运行时参数，不用 `static constexpr`，因为 Ozone/命令入口可能会改 PI 参数。

`FixedPi` 里面只有一个 `FocPi_t`，不会分配堆内存。

`raw()` 暂时保留，是为了混编过渡期能把内部 C 状态拿出来观察或和旧代码对接。等所有 PI 调用都迁完，可以再评估要不要保留它。

## speed_math.hpp 要解决什么

当前速度相关纯数学散在多个 C 文件里：

```text
MotorControl_InternalSpeedCountsToRpm()
speed_ref_ramp_step_counts()
update_speed_loop() 里的 deadband / error clamp
```

这些逻辑不需要访问硬件。适合先写成纯函数：

```text
speed count/s -> rpm
rpm/s 斜坡 -> 每次速度环更新的 speed count/s step
速度命令 deadband
rpm 误差饱和
```

## speed_math.hpp 完整教学程序

```cpp
#pragma once

#include <stdint.h>

#include "BoardConfig.h"
#include "TuneConfig.h"
#include "clamp.hpp"
#include "units.hpp"

namespace cms32::motor
{

struct SpeedMathConfig
{
    static constexpr int32_t counts_per_rev =
        static_cast<int32_t>(MOT_SENSOR_CPR) * static_cast<int32_t>(MOT_SENSOR_POLE_PAIRS);
    static constexpr int32_t estimate_hz = CTRL_SPD_EST_HZ;
    static constexpr int32_t ramp_rpm_per_s = CTRL_SPD_REF_RAMP_RPM_PER_S;
    static constexpr int16_t command_deadband_rpm = CTRL_SPD_CMD_DEADBAND_RPM;
};

static_assert(SpeedMathConfig::counts_per_rev > 0, "invalid speed scale");
static_assert(SpeedMathConfig::estimate_hz > 0, "invalid speed estimate frequency");
static_assert(SpeedMathConfig::ramp_rpm_per_s > 0, "invalid speed ramp");

constexpr int16_t speed_counts_to_rpm_saturated(cms32::support::SpeedCounts speed) noexcept
{
    const int32_t rpm =
        (speed.value * 60L) / static_cast<int32_t>(SpeedMathConfig::counts_per_rev);
    return static_cast<int16_t>(cms32::support::clamp<int32_t>(rpm, -32768, 32767));
}

constexpr cms32::support::SpeedCounts rpm_to_speed_counts_saturated(
    cms32::support::Rpm rpm) noexcept
{
    const int32_t speed =
        (static_cast<int32_t>(rpm.value) * SpeedMathConfig::counts_per_rev) / 60L;
    return cms32::support::SpeedCounts{cms32::support::clamp<int32_t>(
        speed, -CTRL_SPD_REF_LIMIT, CTRL_SPD_REF_LIMIT)};
}

constexpr int32_t speed_ramp_step_counts() noexcept
{
    int32_t step = (SpeedMathConfig::ramp_rpm_per_s * SpeedMathConfig::counts_per_rev) /
                   (60L * SpeedMathConfig::estimate_hz);
    return (step <= 0) ? 1 : step;
}

constexpr bool is_speed_deadband_command(cms32::support::Rpm rpm) noexcept
{
    return (rpm.value > -SpeedMathConfig::command_deadband_rpm) &&
           (rpm.value < SpeedMathConfig::command_deadband_rpm);
}

constexpr int16_t speed_error_rpm(cms32::support::Rpm ref,
                                  cms32::support::Rpm feedback) noexcept
{
    return static_cast<int16_t>(cms32::support::clamp<int32_t>(
        static_cast<int32_t>(ref.value) - static_cast<int32_t>(feedback.value),
        -32768,
        32767));
}

} // namespace cms32::motor
```

### 为什么这里用 static_assert

这些值来自 `BoardConfig.h` 和 `TuneConfig.h`。如果有人把估算频率写成 0，或者编码器比例算成 0，速度环不应该等到上板后才出错。

`static_assert` 的价值是：

```text
配置错误在编译期暴露
不增加运行时代码
让读代码的人知道哪些参数是硬约束
```

## 单独编译检查

写完 `fixed_pi.hpp` 后先跑：

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/MotorControl/Cpp \
  -I Firmware/MotorControl/Algorithm \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_fixed_pi_check.o - <<'CPP'
#include "fixed_pi.hpp"

int main()
{
    cms32::motor::FixedPi pi;
    pi.init(cms32::motor::FixedPiConfig{32, 3, -80, 80, 10});
    return pi.update(10, 2);
}
CPP
```

写完 `speed_math.hpp` 后跑：

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/MotorControl/Cpp \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_speed_math_check.o - <<'CPP'
#include "speed_math.hpp"

static_assert(cms32::motor::speed_ramp_step_counts() > 0, "bad ramp");

int main()
{
    const auto speed = cms32::motor::rpm_to_speed_counts_saturated(
        cms32::support::Rpm{100});
    return cms32::motor::speed_counts_to_rpm_saturated(speed);
}
CPP
```

## 什么时候可以接入真实快环

可以接入的条件：

```text
单文件编译通过
完整固件构建通过
nm 查不到异常/RTTI/堆相关符号
新函数和旧 C 公式逐项等价
```

不要接入的情况：

```text
还没上板验证 C++ core shell
fixed_pi.hpp 自己重写了 PI 算法但没有和 foc_pi_update 对拍
speed_math.hpp 改了单位缩放或限幅语义
为了使用 class 而改变 20 kHz 快环调用顺序
```

这一阶段的目标是把小规则写清楚，不是让固件马上变成全 C++。
