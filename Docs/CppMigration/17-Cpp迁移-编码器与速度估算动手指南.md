# 编码器与速度估算 C++ 动手指南

本文是 Stage 6 的动手文档：把编码器 raw、坏角判断和速度估算里的纯数学拆成 C++ 小组件。

> 当前最终落地形态见 `25-Cpp迁移-C_Cpp混编最小封装落地指南.md`。本文保留编码器数学
> 组件教学；正式编码器/速度状态现在看 `g_motor.encoder` 和 `g_motor.speed`。

第一版只写教学文件：

```text
Firmware/MotorControl/Math/encoder_math.hpp
Firmware/MotorControl/Core/angle_validator.hpp
Firmware/MotorControl/Core/speed_estimator.hpp
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

#include "config.hpp"
#include "units.hpp"

namespace cms32::motor
{

struct EncoderMathConfig
{
    static constexpr int32_t sensor_cpr = EncoderConfig::sensor_cpr;
    static constexpr int32_t counts_per_rev = EncoderConfig::counts_per_rev;
    static constexpr int32_t speed_estimate_hz = SpeedLoopConfig::estimate_hz;
    static constexpr int32_t direction = EncoderConfig::direction;
    static constexpr int32_t spike_rpm = SpeedLoopConfig::diff_spike_rpm;
    static constexpr int32_t pos_deadband = SpeedLoopConfig::pos_deadband;
    static constexpr int32_t zero_snap = SpeedLoopConfig::zero_snap;
};

static_assert(EncoderMathConfig::sensor_cpr > 0, "invalid sensor cpr");
static_assert(EncoderMathConfig::counts_per_rev > 0, "invalid encoder scale");
static_assert(EncoderMathConfig::speed_estimate_hz > 0, "invalid estimate frequency");
static_assert((EncoderMathConfig::direction == 1) || (EncoderMathConfig::direction == -1),
              "invalid sensor direction");
static_assert((EncoderMathConfig::pos_deadband >= 0) &&
                  (EncoderMathConfig::pos_deadband <= 32767),
              "invalid speed position deadband");

constexpr int16_t raw_delta(cms32::support::EncoderRaw previous,
                            cms32::support::EncoderRaw current) noexcept
{
    return static_cast<int16_t>(current.value - previous.value);
}

constexpr uint16_t speed_diff_max_delta_raw() noexcept
{
    int32_t limit = (EncoderMathConfig::spike_rpm * EncoderMathConfig::counts_per_rev) /
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

constexpr int16_t deadband_delta(int16_t delta) noexcept
{
    if ((delta > -EncoderMathConfig::pos_deadband) &&
        (delta < EncoderMathConfig::pos_deadband))
    {
        return 0;
    }
    return delta;
}

constexpr cms32::support::SpeedCounts raw_delta_to_speed_counts(int16_t delta) noexcept
{
    const int16_t speed_delta = deadband_delta(delta);
    return cms32::support::SpeedCounts{
        static_cast<int32_t>(speed_delta) * EncoderMathConfig::speed_estimate_hz *
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

`SpeedEstimator` 只做 raw delta -> deadband -> speed count/s -> filter -> zero snap。

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
  速度估算状态机，只处理 prev/raw/deadband/filter。
```

不要把 Board 读取、坏角即时重读、watch 计数全部塞进一个 class。那些属于现有 `motor_control_encoder.c` 的系统 glue，后续再逐步接入。

## 单独编译检查

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/MotorControl/Core \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_encoder_speed_check.o - <<'CPP'
#include "encoder_math.hpp"
#include "angle_validator.hpp"
#include "speed_estimator.hpp"

static_assert(cms32::motor::speed_diff_max_delta_raw() == 28398U,
              "unexpected speed spike raw limit");
static_assert(cms32::motor::deadband_delta(1) == 0, "speed delta deadband failed");
static_assert(cms32::motor::deadband_delta(16) == 16, "speed delta edge changed");

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

## 当前第一版：encoder.cpp 薄接入层

Stage 17 现在已经完成第一层：

```text
encoder_math.hpp
  纯公式已经有了。

angle_validator.hpp
  教学版角度可信度模型已经有了。

speed_estimator.hpp
  教学版速度估算模型已经有了。
```

现在不要继续写新的外壳，也不要先做 UART。更应该保持这些已经验证过的数学小组件进入真实控制链路：

```text
Firmware/MotorControl/LegacyC/motor_control_encoder.c
    -> Firmware/MotorControl/Core/encoder.cpp
```

这里的目标不是把编码器模块彻底类化。第一版只做薄接入：

```text
C 文件改成 C++ 编译
MotorControl_* C ABI 函数名不变
MotorControlCState 仍然是正式状态源
watch 字段和 Ozone 观察方式不变
Board/MA600 读取方式不变
```

也就是说，`encoder.cpp` 仍然导出这些函数：

```text
MotorControl_EncoderReset()
MotorControl_InternalUpdateEncoderAngle()
MotorControl_InternalUpdateEncoderSpeed()
MotorControl_InternalSpeedCountsToRpm()
```

为什么先做这个：

```text
core.cpp 已经负责慢环状态机
current.cpp 已经负责电流/速度快环
encoder_math.hpp 已经验证过纯数学

现在缺的是：让真实编码器路径也开始使用 C++ 数学，而不是停留在教学头文件。
```

## 为什么不先写 UART

UART 是通信入口，不在电机闭环主链路上。现在先不做串口通信，就不需要为了一个 UART0 写 `uart_policy.hpp`。

当前更重要的是控制链路本身：

```text
电流环要用 encoder_elec 做 Park/InvPark
速度环要用 encoder_delta / speed_fb
位置环要用 encoder_pos
```

所以 Encoder 接入比 UART 更靠前。它直接影响闭环是否稳定，应该优先把这里的单位、公式、异常样本处理收清楚。

## 为什么不马上把 AngleValidator 接进正式状态

`AngleValidator` 内部有自己的状态：

```text
previous_
last_delta_
initialized_
```

但当前正式状态已经在 `MotorControlCState` 里：

```text
encoder_raw
encoder_prev_raw
encoder_delta
encoder_initialized
encoder_raw_step
encoder_reject_step
encoder_reject_count
encoder_retry_count
encoder_hold_count
encoder_age
encoder_ok
```

如果第一版直接把 `AngleValidator` 对象放进正式路径，会出现两份状态：

```text
MotorControlCState 里一份
AngleValidator 私有成员里一份
```

这会带来两个问题：

```text
Ozone 里看不到完整状态来源
状态同步错误时很难判断到底哪一份错了
```

所以第一版 `encoder.cpp` 只用 `encoder_math.hpp` 里的无状态纯函数：

```text
raw_delta()
raw_delta_plausible()
speed_diff_max_delta_raw()
deadband_delta()
raw_delta_to_speed_counts()
```

这些函数没有私有状态，替换 C 里的局部公式最稳。

## 为什么不马上把 SpeedEstimator 接进正式状态

`SpeedEstimator` 也有自己的状态：

```text
previous_
filter_
speed_
last_delta_
initialized_
```

但正式速度估算现在依赖 `MotorControlCState` 里的字段：

```text
encoder_prev_raw
encoder_delta
encoder_pos
speed_fb_diff
speed_fb
speed_reject_count
speed_reject_delta
speed_startup_blank
```

这些字段不是多余的。它们服务两个目的：

```text
控制行为：
  spike 时更新 prev_raw、清 speed_fb_diff、startup blank 前几拍输出 0。

调试观察：
  Ozone 可以直接看到 delta、pos、speed_fb、reject_count。
```

所以第一版不要让 `SpeedEstimator` 接管正式状态。正确做法是：

```text
先用 encoder_math.hpp 替换公式
继续把状态写回 MotorControlCState
等 C++ 接入行为上板稳定后，再评估要不要把 SpeedEstimator 改成“无状态算法”或“引用外部状态”的形式
```

## encoder.cpp 第一版替换清单

第一版只替换这些安全点：

```text
encoder_raw_delta()
  用 raw_delta(EncoderRaw{previous}, EncoderRaw{current})

encoder_raw_plausible()
  用 raw_delta_plausible(delta, MOT_ENCODER_MAX_STEP_RAW)

speed_diff_max_delta_raw()
  用 speed_diff_max_delta_raw()

速度 deadband
  用 deadband_delta(delta)

speed_sample
  用 raw_delta_to_speed_counts(delta).value

speed count/s -> rpm
  用 SpeedMath::to_rpm(SpeedCounts{speed}).value
```

第一版必须保持这些行为不变：

```text
首帧 encoder 无条件接受
坏角不更新 encoder_raw
坏角先 retry，retry 成功才 accept
retry 失败才 reject 并 hold
hold 时 encoder_age 增长
encoder_age 超过 MOT_ANGLE_MAX_AGE 后 encoder_ok 变 0
speed spike 时更新 encoder_prev_raw
speed spike 时 encoder_delta 清 0
speed spike 时 speed_fb_diff 清 0
startup_blank 前几拍 speed_fb 为 0
encoder_pos += delta 的时机不变
```

第一版不要改这些：

```text
bsp_update_angle_fast()
bsp_angle_raw()
bsp_angle_ok()
MA600 SPI 读取
MotorControlWatch_t 字段
MotorControlCState 字段布局
MotorControl_* C ABI 函数名
```

## 接入后的验证

完成 `encoder.cpp` 后跑：

```sh
./testhpp.sh
cmake --build --preset gcc-debug --target cms32foc
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "operator new|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
```

然后做行为对拍：

```text
手转电机：
  encoder_raw 连续变化
  encoder_delta 正负方向符合 MOT_SENSOR_DIR
  encoder_pos 连续累计
  speed_fb_rpm 大小和方向合理

制造坏角或断开传感器：
  encoder_reject_count / encoder_hold_count 增长
  encoder_age 增长
  超过允许 age 后 encoder_ok 变 0
```

这个阶段做完，MotorControl 主链路会变成：

```text
core.cpp
current.cpp
encoder.cpp
```

这比继续写新的 wrapper 更有价值，因为它把已经通过头文件检查的 C++ 数学真正放进控制路径。
