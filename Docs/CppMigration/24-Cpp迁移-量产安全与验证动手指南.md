# 量产安全与验证动手指南

本文讲 C/C++ 混编工程走向量产时，除了“代码能跑”以外还要补什么。C++ 的作用不是让代码看起来高级，而是让参数、状态、单位和测试更可靠。

量产目标：

```text
故障路径明确
调试符号真实
参数来源清楚
控制周期稳定
上电自检可解释
关键算法可离板测试
```

## 1. 故障码要分层

MotorControl 的故障是电机控制故障：

```text
current fault
encoder fault
unsupported mode
open loop timeout
```

ScrewAxis 的故障是机械轴故障：

```text
home outer timeout
home inner timeout
position before home
soft limit violation
motor fault propagated
```

不要把所有故障都塞进一个 `fault_reason`。更好的做法是：

```text
g_motor.runtime.fault:
  看底层电机控制为什么停

g_screw_axis_watch.fault_reason:
  看轴层业务为什么停
```

### 完整参考代码：轴层故障枚举

```c
typedef enum
{
    ScrewAxisFault_None = 0U,
    ScrewAxisFault_Motor = 1U,
    ScrewAxisFault_HomeOuterTimeout = 2U,
    ScrewAxisFault_HomeInnerTimeout = 3U,
    ScrewAxisFault_PositionBeforeHome = 4U,
    ScrewAxisFault_TargetOutOfRange = 5U,
} ScrewAxisFault_t;
```

对应 watch 增加：

```c
typedef struct
{
    uint8_t state;
    uint8_t fault_reason;
    uint8_t homed;
    uint8_t moving;
    int32_t pos_counts;
    int32_t target_counts;
    int32_t travel_counts;
    int16_t speed_cmd_rpm;
} ScrewAxisProductWatch_t;
```

这样调试时先看：

```text
ScrewAxis fault_reason != 0:
  业务/机械流程停了

g_motor.runtime.fault != 0:
  底层电机控制停了
```

## 2. 参数要有版本

量产时参数会变：

```text
电角零位
电流 PI
速度 PI
回零速度
软限位
串口波特率
```

建议先定义一个参数版本号，即使暂时还不写 Flash：

```c
#define CMS32_PARAM_VERSION 1U
```

后续如果保存到 Flash，结构体可以长这样：

```c
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int16_t elec_zero;
    int16_t current_kp;
    int16_t current_ki;
    int16_t speed_kp;
    int16_t speed_ki;
    int16_t home_fast_rpm;
    int16_t home_slow_rpm;
    int16_t home_extend_rpm;
    int32_t travel_counts;
    uint32_t crc32;
} ProductParamBlock_t;
```

原则：

```text
结构体有 magic/version/size/crc
新增字段只在版本升级时处理
启动时校验失败就用编译期默认值
Ozone/watch 能看到当前参数版本
```

## 3. 关键算法要能离板测试

适合 host-side 测试的东西：

```text
speed_math
fixed_pi
command_sanitizer
angle_validator
speed_estimator
ScrewAxis 状态转移
```

不适合直接 host-side 测试的东西：

```text
真实寄存器
ADC IRQ
PWM 输出
vendor driver
```

所以 C++ 小组件要尽量保持：

```text
不读写硬件寄存器
不依赖全局变量
输入明确
输出明确
```

### 完整参考代码：最小 speed_math 测试

可以先建一个 host-only 测试文件：

```cpp
#include "speed_math.hpp"

#include <cassert>

int main()
{
    using cms32::motor::SpeedMath;
    using cms32::support::Rpm;
    using cms32::support::SpeedCounts;

    const SpeedCounts speed = SpeedMath::to_speed(Rpm{1000});
    assert(speed.value == 4369066);

    const Rpm rpm = SpeedMath::to_rpm(SpeedCounts{4369066});
    assert((rpm.value >= 999) && (rpm.value <= 1000));

    assert(SpeedMath::in_deadband(Rpm{0}));
    assert(!SpeedMath::in_deadband(Rpm{50}));

    return 0;
}
```

这类测试能防止以后改单位类型时把 `rpm` 和 `count/s` 搞反。

### 完整参考代码：最小 CMake host test 形状

先不要接进固件 target，可以独立建 host target：

```cmake
add_executable(cms32_speed_math_host_test
    Tests/Host/speed_math_test.cpp
)

target_include_directories(cms32_speed_math_host_test PRIVATE
    Firmware/MotorControl/Core
    Firmware/MotorControl/Abi
    Firmware/MotorControl/Abi
    Firmware/Support
    Firmware/Board/Config
)

target_compile_features(cms32_speed_math_host_test PRIVATE cxx_std_17)
```

注意：如果某个头 include 了 CMSIS 或 vendor 寄存器，就不适合直接 host test。需要把纯数学部分再切薄一点。

## 4. 固件构建要查 C++ 运行时污染

量产前每次 C++ 重构都建议查：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "operator new|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
```

期望没有输出。

如果出现：

```text
operator new:
  可能用了 new、std::vector、std::string。

typeinfo / vtable:
  可能用了 virtual 或 RTTI。

__cxa_throw:
  可能启用了异常。
```

量产控制路径不应该依赖这些东西。

## 5. 中断和实时性要有边界

当前结构应该坚持：

```text
ADC IRQ:
  采样、FOC 快环、极短 UART TX task

main loop:
  ScrewAxis
  MotorControl_ApplyCommand
  MotorControl_RunSlowLoop

Ozone:
  直接展开 g_mc_* / g_motor.diag / g_motor.debug
```

不要在 ADC IRQ 做：

```text
串口解析长命令
printf
复杂状态机
Flash 写入
动态内存
长循环等待
```

C++ 里 RAII guard 可以用，但必须极短：

```cpp
{
    const cms32::support::AdcIrqGuard guard;
    s_mc.command = next_command;
}
```

这个作用域越短越好。不要把慢逻辑包进关中断区。

## 6. 上电自检清单

量产上电至少要能回答：

```text
电流 offset 是否完成
三相电流是否在安全范围
MA600 是否有角度
PWM 是否默认关闭
MotorControl 是否 Idle
ScrewAxis 是否未 homed
参数版本是否有效
复位原因是什么
```

可以先用 watch 字段表达，不急着做复杂日志：

```c
typedef struct
{
    uint8_t current_offset_ok;
    uint8_t current_ok;
    uint8_t encoder_seen;
    uint8_t pwm_off_safe;
    uint8_t param_valid;
    uint8_t reset_reason;
} ProductBootWatch_t;
```

## 7. 验收命令

每次做一段 C++ 重构，至少跑：

```sh
./testhpp.sh
cmake --build --preset gcc-debug --target cms32foc
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "operator new|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
```

如果改了公共符号，再查：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "g_motor|g_screw|MotorControl_|ScrewAxis_"
```

如果改了 watch 结构体，要在 Ozone 里确认：

```text
真实符号能展开
字段顺序符合头文件
状态值和文档一致
```

## 8. 量产前最后标准

```text
没有调试误导型宏别名
状态和故障码有稳定枚举
C++ 控制路径没有 heap/exception/RTTI/virtual
所有速度/电流/角度/位置单位有清楚类型或字段注释
回零和位置运动有软限位
Motor fault 和 Axis fault 分层
构建、符号、watch、文档同步
```
