# 编码器与速度估算 C++ 动手指南

本文是 Stage 6 的动手文档：把编码器 raw、坏角判断和速度估算里的纯数学拆成 C++ 小组件。

第一版只写教学文件：

```text
Firmware/MotorControl/Cpp/encoder_math.hpp
Firmware/MotorControl/Cpp/angle_validator.hpp
Firmware/MotorControl/Cpp/speed_estimator.hpp
```

不要在第一版里读 MA600，也不要改 `foc_ma600.c` 或 Board SPI。

## 总规则

这些组件必须满足：

```text
不访问寄存器
不调用 bsp_update_angle_fast()
不调用 bsp_angle_raw()
不修改 MotorControlCState
不分配动态内存
不使用异常/RTTI/virtual
```

你写的时候心里只记一句话：

```text
Board 负责拿 raw，Encoder C++ 只负责判断 raw 和估算速度。
```

## encoder_math.hpp 完整教学程序

```cpp
#pragma once

#include <stdint.h>

#include "BoardConfig.h"
#include "TuneConfig.h"
#include "clamp.hpp"
#include "units.hpp"

namespace cms32::motor
{

struct EncoderMathConfig
{
    static constexpr int32_t sensor_cpr = static_cast<int32_t>(MOT_SENSOR_CPR);
    static constexpr int32_t sensor_pole_pairs = static_cast<int32_t>(MOT_SENSOR_POLE_PAIRS);
    static constexpr int32_t speed_estimate_hz = CTRL_SPD_EST_HZ;
    static constexpr int32_t direction = MOT_SENSOR_DIR;
    static constexpr int32_t spike_rpm = CTRL_SPD_DIFF_SPIKE_RPM;
    static constexpr int32_t zero_snap = CTRL_SPD_ZERO_SNAP;
};

static_assert(EncoderMathConfig::sensor_cpr > 0, "invalid sensor cpr");
static_assert(EncoderMathConfig::sensor_pole_pairs > 0, "invalid sensor pole pairs");
static_assert(EncoderMathConfig::speed_estimate_hz > 0, "invalid estimate frequency");
static_assert((EncoderMathConfig::direction == 1) || (EncoderMathConfig::direction == -1),
              "invalid sensor direction");

constexpr int16_t raw_delta(cms32::support::EncoderRaw previous,
                            cms32::support::EncoderRaw current) noexcept
{
    return static_cast<int16_t>(current.value - previous.value);
}

constexpr uint16_t speed_diff_max_delta_raw() noexcept
{
    int32_t limit = (EncoderMathConfig::spike_rpm * EncoderMathConfig::sensor_cpr *
                     EncoderMathConfig::sensor_pole_pairs) /
                    (60L * EncoderMathConfig::speed_estimate_hz);

    if (limit < 1)
    {
        limit = 1;
    }
    if (limit > 32767L)
    {
        limit = 32767L;
    }
    return static_cast<uint16_t>(limit);
}

constexpr bool raw_delta_plausible(int16_t delta, uint16_t max_delta) noexcept
{
    return (delta <= static_cast<int16_t>(max_delta)) &&
           (delta >= -static_cast<int16_t>(max_delta));
}

constexpr cms32::support::SpeedCounts raw_delta_to_speed_counts(int16_t delta) noexcept
{
    return cms32::support::SpeedCounts{
        static_cast<int32_t>(delta) * EncoderMathConfig::speed_estimate_hz *
        EncoderMathConfig::direction};
}

constexpr cms32::support::SpeedCounts zero_snap_speed(
    cms32::support::SpeedCounts speed) noexcept
{
    if ((speed.value > -EncoderMathConfig::zero_snap) &&
        (speed.value < EncoderMathConfig::zero_snap))
    {
        return cms32::support::SpeedCounts{0};
    }
    return speed;
}

} // namespace cms32::motor
```

## angle_validator.hpp 完整教学程序

`AngleValidator` 只判断 raw 是否可信，不负责读取 raw。

```cpp
#pragma once

#include <stdint.h>

#include "encoder_math.hpp"

namespace cms32::motor
{

template <uint16_t MaxStepRaw> class AngleValidator
{
public:
    static_assert(MaxStepRaw > 0U, "MaxStepRaw must be positive");
    static_assert(MaxStepRaw <= 32767U, "MaxStepRaw must fit int16 delta");

    constexpr bool accept(cms32::support::EncoderRaw raw) noexcept
    {
        if (!initialized_)
        {
            previous_ = raw;
            initialized_ = true;
            last_delta_ = 0;
            return true;
        }

        const int16_t delta = raw_delta(previous_, raw);
        last_delta_ = delta;
        if (!raw_delta_plausible(delta, MaxStepRaw))
        {
            return false;
        }

        previous_ = raw;
        return true;
    }

    constexpr void reset(cms32::support::EncoderRaw raw = cms32::support::EncoderRaw{0U}) noexcept
    {
        previous_ = raw;
        initialized_ = false;
        last_delta_ = 0;
    }

    constexpr int16_t last_delta() const noexcept
    {
        return last_delta_;
    }

private:
    cms32::support::EncoderRaw previous_{0U};
    int16_t last_delta_{0};
    bool initialized_{false};
};

} // namespace cms32::motor
```

## speed_estimator.hpp 完整教学程序

`SpeedEstimator` 只做 raw delta -> speed count/s -> filter -> zero snap。

```cpp
#pragma once

#include <stdint.h>

#include "encoder_math.hpp"
#include "low_pass.hpp"

namespace cms32::motor
{

template <uint8_t FilterShift> class SpeedEstimator
{
public:
    static_assert(FilterShift < 31U, "FilterShift is too large");

    constexpr void reset(cms32::support::EncoderRaw raw = cms32::support::EncoderRaw{0U}) noexcept
    {
        previous_ = raw;
        initialized_ = false;
        filter_.reset(0);
        speed_ = cms32::support::SpeedCounts{0};
        last_delta_ = 0;
    }

    constexpr cms32::support::SpeedCounts update(cms32::support::EncoderRaw raw) noexcept
    {
        if (!initialized_)
        {
            previous_ = raw;
            initialized_ = true;
            return cms32::support::SpeedCounts{0};
        }

        const int16_t delta = raw_delta(previous_, raw);
        previous_ = raw;
        last_delta_ = delta;

        if (!raw_delta_plausible(delta, speed_diff_max_delta_raw()))
        {
            filter_.reset(0);
            speed_ = cms32::support::SpeedCounts{0};
            return speed_;
        }

        const auto sample = raw_delta_to_speed_counts(delta);
        speed_ = zero_snap_speed(cms32::support::SpeedCounts{filter_.update(sample.value)});
        return speed_;
    }

    constexpr int16_t last_delta() const noexcept
    {
        return last_delta_;
    }

private:
    cms32::support::EncoderRaw previous_{0U};
    cms32::support::LowPassI32<FilterShift> filter_{};
    cms32::support::SpeedCounts speed_{0};
    int16_t last_delta_{0};
    bool initialized_{false};
};

} // namespace cms32::motor
```

## 为什么分三个文件

```text
encoder_math.hpp:
  纯公式，最好单独验证。

angle_validator.hpp:
  角度样本可信度，只处理单拍 raw 是否接受。

speed_estimator.hpp:
  速度估算状态机，只处理 prev/raw/filter。
```

不要把 Board 读取、坏角即时重读、watch 计数全部塞进一个 class。那些属于现有 `motor_control_encoder.c` 的系统 glue，后续再逐步接入。

## 单独编译检查

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/MotorControl/Cpp \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_encoder_speed_check.o - <<'CPP'
#include "encoder_math.hpp"
#include "angle_validator.hpp"
#include "speed_estimator.hpp"

static_assert(cms32::motor::speed_diff_max_delta_raw() > 0U, "bad spike limit");

int main()
{
    cms32::motor::AngleValidator<1024U> validator;
    cms32::motor::SpeedEstimator<2U> estimator;
    (void)validator.accept(cms32::support::EncoderRaw{10U});
    const auto speed = estimator.update(cms32::support::EncoderRaw{20U});
    return static_cast<int>(speed.value);
}
CPP
```

## 什么时候可以接入 motor_control_encoder.c

可以接入的条件：

```text
单独编译通过
公式和现有 C 函数对拍一致
坏角 reject/hold/retry 计数语义不变
encoder_ok / encoder_age 行为不变
```

不要接入的情况：

```text
还想同时改 MA600 SPI 读取
想把 retry/hold/watch 计数都包进 estimator
没有确认 raw delta 的 int16 回绕语义
没有上板确认 C++ core shell
```

Encoder 是容易把单位搞混的模块。先让数学更清楚，再动系统行为。
