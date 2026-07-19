# FOC 坐标变换 C++ 动手指南

本文是 Stage 5 的后半篇：把 `foc_math.c` 里的 FOC 变换用 C++ 类型包装起来。

第一版不替换：

```text
Firmware/MotorControl/Algorithm/foc_math.c
Firmware/MotorControl/Algorithm/foc_math.h
```

第一版只练习未来可能出现的教学文件：

```text
Firmware/MotorControl/Cpp/foc_transform.hpp
```

## 总规则

这类 wrapper 必须满足：

```text
不复制 sin 表
不重写 Park/Clarke/SVPWM 公式
不改变 foc_math.c 的 C ABI
不访问 Board
不读 ADC
不写 PWM
不引入浮点
```

你写的时候心里只记一句话：

```text
C++ 先表达单位和坐标系，公式继续复用旧 C。
```

## 为什么不急着重写 foc_math.c

`foc_math.c` 是当前电流快环的稳定基础。它包含：

```text
Q15 sin/cos
Clarke
Park
InvPark
PI
dq voltage limit
SVPWM
```

这些函数已经被 C 快环使用。现在最稳的办法是给 C++ 新代码提供更清楚的入口，而不是马上替换算法本体。

## foc_transform.hpp 完整教学程序

```cpp
#pragma once

#include <stdint.h>

extern "C"
{
#include "foc_math.h"
}

#include "units.hpp"

namespace cms32::motor
{

struct Q15
{
    int16_t value;
};

struct PhaseCurrent
{
    FocPhaseCurrent_t value;
};

struct AlphaBeta
{
    FocAlphaBeta_t value;
};

struct Dq
{
    FocDq_t value;
};

struct Duty
{
    FocDuty_t value;
};

constexpr cms32::support::Angle16 angle16(uint16_t value) noexcept
{
    return cms32::support::Angle16{value};
}

inline Q15 sin_q15(cms32::support::Angle16 angle) noexcept
{
    return Q15{foc_sin_q15(angle.value)};
}

inline Q15 cos_q15(cms32::support::Angle16 angle) noexcept
{
    return Q15{foc_cos_q15(angle.value)};
}

inline AlphaBeta clarke_2phase(int16_t iu, int16_t iv) noexcept
{
    return AlphaBeta{foc_clarke_2phase(iu, iv)};
}

inline AlphaBeta clarke_3phase(PhaseCurrent current) noexcept
{
    return AlphaBeta{foc_clarke_3phase(current.value)};
}

inline Dq park(AlphaBeta input, cms32::support::Angle16 theta) noexcept
{
    return Dq{foc_park(input.value, theta.value)};
}

inline AlphaBeta inv_park(Dq input, cms32::support::Angle16 theta) noexcept
{
    return AlphaBeta{foc_inv_park(input.value, theta.value)};
}

inline bool limit_dq(Dq& voltage, int16_t limit) noexcept
{
    return foc_limit_dq(&voltage.value, limit) != 0U;
}

inline Duty svpwm(AlphaBeta voltage, uint16_t vdc, uint16_t duty_min, uint16_t duty_max) noexcept
{
    return Duty{foc_svpwm(voltage.value, vdc, duty_min, duty_max)};
}

} // namespace cms32::motor
```

## 这些小 struct 有什么意义

`FocAlphaBeta_t` 和 `FocDq_t` 本身都是 C 结构体。C++ wrapper 不是为了增加运行时层级，而是为了让调用点更明确：

```cpp
const auto ab = cms32::motor::clarke_3phase(current);
const auto dq = cms32::motor::park(ab, cms32::support::Angle16{theta});
```

读代码时能看到：

```text
Clarke 输出 alpha/beta
Park 输入 alpha/beta 和角度
Park 输出 d/q
```

而不是一堆裸结构体在函数间传来传去。

## inline 为什么合适

这些 wrapper 都很薄，只是转调 C 函数。`inline` 的含义是：

```text
允许头文件里定义函数
避免多个 .cpp include 后重复定义
给编译器内联机会
```

它不是强迫编译器一定内联。真正是否内联由优化器决定。

## 单独编译检查

写完 `foc_transform.hpp` 后跑：

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
  -x c++ -c -o /tmp/cms32_foc_transform_check.o - <<'CPP'
#include "foc_transform.hpp"

int main()
{
    const auto ab = cms32::motor::clarke_2phase(1, -1);
    const auto dq = cms32::motor::park(ab, cms32::support::Angle16{0U});
    const auto out = cms32::motor::inv_park(dq, cms32::support::Angle16{0U});
    return out.value.alpha;
}
CPP
```

## 什么时候可以替换 foc_math.c

短期不建议替换。只有同时满足下面条件才考虑：

```text
已有 C++ wrapper 使用稳定
有旧 C 函数和新 C++ 函数的对拍测试
Flash/RAM 没有异常增长
上板电流环行为一致
```

不要做：

```text
用 constexpr 重新生成 sin 表
用浮点重写 Park
用模板一次性改完整 SVPWM
让 C 快环直接依赖复杂 C++ 对象
```

这一阶段的价值是让后续 C++ 代码写得更不容易混单位，而不是改变 FOC 数学。
