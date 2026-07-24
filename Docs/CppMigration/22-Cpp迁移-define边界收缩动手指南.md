# define 边界收缩动手指南

本文讲下一阶段怎么把业务 `#define` 逐步赶到边界上，让 C++ 主体代码使用 `enum class`、`static constexpr` 和强类型配置。目标不是一次删光宏，而是让量产代码更可读、更好调试、更少裸整数混用。

> 当前最终落地形态见 `25-Cpp迁移-C_Cpp混编最小封装落地指南.md`。本文中的
> `g_motor_cmd/g_motor_watch` 例子是当时从宏别名退场的过渡说明；当前命令入口是
> `g_mc_cmd`，观察入口是 `cms32::motor::g_motor`。

当前工程仍然是 C/C++ 混编：

```text
C:
  启动、中断入口、vendor driver、寄存器、C ABI、部分底层算法

C++:
  状态机、命令清洗、配置镜像、数学小组件、轴层业务逻辑
```

所以 `#define` 的退场规则是：

```text
预处理开关可以保留
硬件/C 兼容配置先保留
C++ 内部不直接散落使用宏
业务状态和故障码优先改成 enum
调试符号不要用宏别名伪装
```

## 1. 哪些 define 先不要动

这些宏暂时保留是合理的：

```c
#define BOARD_UART_ENABLE 0U
#define PWM_FREQ_HZ 20000U
#define MOT_SENSOR_CPR 65536ul
#define CTRL_SPD_KP 32
```

原因不同：

```text
BOARD_UART_ENABLE:
  需要给 #if 使用，constexpr 替代不了预处理开关。

PWM_FREQ_HZ / MOT_SENSOR_CPR:
  Board/C 文件仍然要用，先作为 C/C++ 共享源头。

CTRL_SPD_KP:
  当前调试配置仍在 TuneConfig.h，C++ 先通过 config.hpp 镜像使用。
```

不要为了“没有 define”而把 C 文件也一起震荡。量产产品里，稳定边界比形式上纯 C++ 更重要。

## 2. 第一刀：去掉调试误导型宏别名

当前 `MotorControl.h` 里有这种源码别名：

```c
#define g_motor_command g_motor_cmd
#define g_motor_status g_motor_watch
```

它的问题是：

```text
源码里能写 g_motor_status
Ozone/ELF 里没有 g_motor_status 这个真实符号
调试时会以为变量丢了或不能展开
```

量产调试不要靠这种别名。建议统一真实符号：

```text
电机命令入口：g_mc_cmd
电机观察入口：g_motor.*
轴级命令入口：g_screw_axis_cmd
轴级观察入口：g_screw_axis_watch
```

### 完整参考代码：MotorControl.h 尾部

把 `MotorControl.h` 里公共变量部分整理成这样：

```c
/**
 * @brief Ozone/主循环写入的电机控制命令入口。
 *
 * 真实 ELF 符号名就是 g_motor_cmd。调试器/Ozone 里也应该直接观察这个名字。
 */
extern volatile MotorControlCommand_t g_motor_cmd;

/**
 * @brief Ozone 观察用主控制状态快照。
 *
 * 真实 ELF 符号名就是 g_motor_watch。主循环通过
 * MotorControl_UpdateWatch(&g_motor_watch) 刷新。
 */
extern volatile MotorControlWatch_t g_motor_watch;

void MotorControl_Init(void);
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command);
void MotorControl_RunSlowLoop(void);
uint8_t MotorControl_FastLoopFromAdcIrq(void);
void MotorControl_GetWatch(MotorControlWatch_t* out);
void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out);
```

然后源码调用点统一改：

```c
MotorControl_UpdateWatch(&g_motor_watch);
MotorControl_ApplyCommand(&g_motor_cmd);
```

`ScrewAxis` 里也统一读写真实名字：

```cpp
g_motor_cmd.enable = 1U;
g_motor_cmd.control_mode = 2U;
g_motor_cmd.speed_ref_rpm = speed_rpm;

const int16_t fb = g_motor_watch.speed_fb_rpm;
```

这样 Ozone、源码、符号表三者一致。

## 3. 第二刀：状态/模式/故障码改成 C enum

当前内部头文件里是宏：

```c
#define MC_STATE_IDLE 0U
#define MC_MODE_SPEED 2U
#define MC_FAULT_ENCODER 4U
```

宏只是文本替换。更适合量产的是 C enum：

```text
C 文件能继续用
C++ 可以继续包 enum class
Ozone 也更容易显示枚举名
状态值集中在一个类型里
```

### 完整参考代码：motor_control_internal.h 枚举

```c
typedef enum
{
    MC_STATE_IDLE = 0U,
    MC_STATE_CLOSED_LOOP = 3U,
    MC_STATE_FAULT = 4U,
} MotorControlStateRaw_t;

typedef enum
{
    MC_MODE_OFF = 0U,
    MC_MODE_CURRENT = 1U,
    MC_MODE_SPEED = 2U,
    MC_MODE_VF_OPEN_LOOP = 3U,
    MC_MODE_ALIGN_LOCK = 4U,
    MC_MODE_ENCODER_VOLTAGE = 5U,
} MotorControlModeRaw_t;

typedef enum
{
    MC_FAULT_NONE = 0U,
    MC_FAULT_UNSUPPORTED_MODE = 1U,
    MC_FAULT_CURRENT = 2U,
    MC_FAULT_OPEN_LOOP_TIMEOUT = 3U,
    MC_FAULT_ENCODER = 4U,
} MotorControlFaultRaw_t;
```

内部状态结构体可以继续保持 `uint8_t`，这样 ABI 和 watch 数值不变：

```c
typedef struct
{
    uint8_t state;
    uint8_t fault;
    uint8_t enabled;
    uint8_t mode;
    /* 后续字段不变 */
} MotorControlCState;
```

为什么不立刻把字段也改成 enum 类型？

```text
公共 watch 里现在就是 uint8_t
Ozone 表达式和历史文档都按数字看
C/C++ 混编时 uint8_t 更稳定
先改常量定义，后面再决定是否改字段类型
```

## 4. C++ 继续用 enum class

C enum 解决 C 边界，C++ 内部仍然要更严格：

```cpp
#pragma once

#include <stdint.h>

#include "motor_control_internal.h"

namespace cms32::motor
{
enum class ControlState : uint8_t
{
    Idle = MC_STATE_IDLE,
    ClosedLoop = MC_STATE_CLOSED_LOOP,
    Fault = MC_STATE_FAULT,
};

enum class ControlMode : uint8_t
{
    Off = MC_MODE_OFF,
    Current = MC_MODE_CURRENT,
    Speed = MC_MODE_SPEED,
    VfOpenLoop = MC_MODE_VF_OPEN_LOOP,
    AlignLock = MC_MODE_ALIGN_LOCK,
    EncoderVoltage = MC_MODE_ENCODER_VOLTAGE,
};

enum class ControlFault : uint8_t
{
    None = MC_FAULT_NONE,
    UnsupportedMode = MC_FAULT_UNSUPPORTED_MODE,
    Current = MC_FAULT_CURRENT,
    OpenLoopTimeout = MC_FAULT_OPEN_LOOP_TIMEOUT,
    Encoder = MC_FAULT_ENCODER,
};

constexpr ControlMode to_control_mode(uint8_t value) noexcept
{
    return static_cast<ControlMode>(value);
}

constexpr bool is_closed_loop_mode(ControlMode mode) noexcept
{
    return (mode == ControlMode::Current) || (mode == ControlMode::Speed);
}

constexpr bool is_supported_run_mode(ControlMode mode) noexcept
{
    return (mode == ControlMode::VfOpenLoop) || is_closed_loop_mode(mode);
}
} // namespace cms32::motor
```

这样 C++ 里不会写出这种错误：

```cpp
if (mode) { }                 // enum class 不能隐式当 bool
if (fault == mode) { }        // 不同 enum class 不能直接比较
```

这正是我们想要的。编译器越早拦住错误，越适合量产。

## 5. 第三刀：C++ 不直接吃 CTRL/MOT/PWM 宏

`config.hpp` 是 C++ 配置镜像层。宏还在 C 头文件里，但 C++ 业务代码统一走结构体：

```cpp
namespace cms32::motor
{
struct CurrentLoopConfig
{
    static constexpr int16_t ref_limit = CTRL_CUR_REF_LIMIT;
    static constexpr int16_t voltage_limit = CTRL_CUR_V_LIMIT;
    static constexpr uint8_t pi_shift = CTRL_CUR_PI_SHIFT;
    static constexpr int16_t kp = CTRL_CUR_KP;
    static constexpr int16_t ki = CTRL_CUR_KI;
    static constexpr int16_t ref_ramp_step = CTRL_CUR_REF_RAMP_STEP;
};

struct SpeedLoopConfig
{
    static constexpr int32_t estimate_hz = CTRL_SPD_EST_HZ;
    static constexpr uint8_t startup_blank_samples = CTRL_SPD_STARTUP_BLANK_SAMPLES;
    static constexpr int16_t kp = CTRL_SPD_KP;
    static constexpr int16_t ki = CTRL_SPD_KI;
    static constexpr uint8_t err_shift = CTRL_SPD_ERR_SHIFT;
    static constexpr uint8_t filter_shift = CTRL_SPD_FILTER_SHIFT;
    static constexpr int16_t command_deadband_rpm = CTRL_SPD_CMD_DEADBAND_RPM;
    static constexpr int32_t ref_ramp_rpm_per_s = CTRL_SPD_REF_RAMP_RPM_PER_S;
    static constexpr int32_t ref_limit_rpm = CTRL_SPD_REF_LIMIT_RPM;
    static constexpr int16_t iq_limit = CTRL_SPD_IQ_LIMIT;
    static constexpr int16_t iq_slew_step = CTRL_SPD_IQ_SLEW_STEP;
    static constexpr int32_t diff_spike_rpm = CTRL_SPD_DIFF_SPIKE_RPM;
    static constexpr int32_t pos_deadband = CTRL_SPD_POS_DEADBAND;
    static constexpr int32_t zero_snap = CTRL_SPD_ZERO_SNAP;
};

struct EncoderConfig
{
    static constexpr int32_t sensor_cpr = static_cast<int32_t>(MOT_SENSOR_CPR);
    static constexpr int32_t motor_pole_pairs = static_cast<int32_t>(MOT_POLE_PAIRS);
    static constexpr int32_t counts_per_rev = sensor_cpr * motor_pole_pairs;
    static constexpr int8_t direction = MOT_SENSOR_DIR;
    static constexpr int16_t elec_zero = MOT_ELEC_ZERO;
    static constexpr uint16_t max_step_raw = MOT_ENCODER_MAX_STEP_RAW;
};
} // namespace cms32::motor
```

然后 C++ 文件里优先写：

```cpp
using cms32::motor::CurrentLoopConfig;
using cms32::motor::SpeedLoopConfig;

const FixedPiConfig current_pi_config{mc->current_command.current_kp,
                                      mc->current_command.current_ki,
                                      -CurrentLoopConfig::voltage_limit,
                                      CurrentLoopConfig::voltage_limit,
                                      CurrentLoopConfig::pi_shift};

mc->speed_iq_ref =
    slew_step<int16_t>(mc->speed_iq_ref,
                       mc->speed_iq_target,
                       SpeedLoopConfig::iq_slew_step);
```

## 6. 完成标准

这一阶段完成后，应该满足：

```text
Ozone 只看真实符号 g_motor_cmd/g_motor_watch
MC_MODE/MC_STATE/MC_FAULT 不再是 #define
C++ .cpp 中没有散落的 CTRL_SPD_* / CTRL_CUR_* 直接引用
BoardConfig.h/TuneConfig.h 仍作为 C/C++ 共享配置源头
编译后没有 new/delete/exception/RTTI 符号
```

建议检查命令：

```sh
rg "#define g_motor_|MC_MODE_|MC_STATE_|MC_FAULT_" Firmware/MotorControl
rg "CTRL_SPD_|CTRL_CUR_|MOT_SENSOR_|PWM_" Firmware/MotorControl/Core
./testhpp.sh
cmake --build --preset gcc-debug --target cms32foc
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "operator new|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
```
