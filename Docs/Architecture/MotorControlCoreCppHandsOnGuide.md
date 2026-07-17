# MotorControl Core Cpp Hands-On Guide

本文是第四阶段 `MotorControl` 核心调度层 C++ 化的动手文档。

它的写法跟 `SupportCppHandsOnGuide.md` 一样：先讲为什么，再给完整程序。你可以照着一个文件一个文件手写，写完一步就编译一次。

这一阶段只迁移外壳：

```text
命令复制
命令限幅
模式判断
慢环 ready/fault 状态机
ADC IRQ 临界区保护
watch 入口转发
```

这一阶段不迁移：

```text
Current/Speed 快环
Encoder 速度估算
SVPWM 输出
VF 快环内部算法
Watch 填充细节
```

也就是说，下面这些 C 文件先继续保留：

```text
Firmware/MotorControl/C/motor_control_current.c
Firmware/MotorControl/C/motor_control_encoder.c
Firmware/MotorControl/C/motor_control_output.c
Firmware/MotorControl/C/motor_control_vf.c
Firmware/MotorControl/C/motor_control_watch.c
```

## 总规则

第四阶段继续遵守 Support 层的规则：

```text
不使用 new/delete
不使用 malloc/free
不使用异常
不使用 RTTI
不使用 virtual
不使用 std::vector/std::string/std::function/iostream
不把复杂业务逻辑放进 Support
不改变 MotorControl.h 对外 C ABI
不改变 ADC 快环算法
```

你心里只记一句话：

```text
C++ 只让边界更清楚，不改变控制时序。
```

## 目标目录

新增目录：

```text
Firmware/MotorControl/Cpp/
├── motor_control_types.hpp
├── motor_control_config.hpp
├── motor_command_sanitizer.hpp
└── motor_control_core.cpp
```

四个文件分工：

```text
motor_control_types.hpp
  把 MC_STATE_* / MC_MODE_* / MC_FAULT_* 包成 enum class

motor_control_config.hpp
  把 TuneConfig.h / BoardConfig.h 里的业务参数收成 C++ 编译期配置

motor_command_sanitizer.hpp
  只做 volatile 命令复制和参数限幅

motor_control_core.cpp
  导出原来的 MotorControl_* C ABI，内部调 C 快环模块
```

注意：

```text
motor_control_types.hpp 和 motor_command_sanitizer.hpp 属于 MotorControl，不属于 Support。
Support 只提供 clamp、units、irq_guard、enum_utils 这些通用工具。
```

## 为什么先迁核心调度层

当前 `motor_control_c.c` 同时做了很多事：

```text
定义 g_motor_cmd / g_motor_watch
定义 s_mc 全局状态
初始化 PI 和安全态
复制并限幅命令
关 ADC IRQ 后更新 s_mc.command/mode/enabled
执行慢环安全检查
根据 mode/state/fault 决定能不能进入快环
分发 ADC IRQ 快环
转发 watch 填充
```

其中真正高风险的是快环算法：

```text
MotorControl_CurrentRunFastLoop()
MotorControl_InternalUpdateEncoderAngle()
MotorControl_InternalUpdateEncoderSpeed()
MotorControl_InternalApplyVoltageVector()
MotorControlVf_RunFastLoop()
```

这些先不动。核心调度层的好处是：

```text
主循环调用，频率低
主要是状态判断和结构体赋值
可以保留原来的 C ABI
可以继续调用现有 C 快环
出问题容易回退
```

## C ABI 不变是什么意思

`MotorControl.h` 对外还是 C 接口：

```c
void MotorControl_Init(void);
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command);
void MotorControl_RunSlowLoop(void);
uint8_t MotorControl_FastLoopFromAdcIrq(void);
void MotorControl_GetWatch(MotorControlWatch_t* out);
void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out);
```

C++ 文件里要这样导出：

```cpp
extern "C" void MotorControl_Init(void)
{
    // ...
}
```

原因是：

```text
C++ 会改函数名以支持重载
extern "C" 告诉编译器这些函数按 C 名字导出
main.c、ADC_IRQHandler、Ozone 入口都不需要知道里面变成了 C++
```

## 第 1 步：motor_control_types.hpp

这个文件只做一件事：把裸 `uint8_t` 模式值包成有名字的 C++ 枚举。

为什么要写它：

```text
MC_MODE_SPEED 是宏，任何 uint8_t 都能传进去
ControlMode::Speed 一眼能看出是控制模式
is_closed_loop_mode() 这种判断可以集中写
后面慢环状态机不再散落一堆 mode == 1/2/3
```

完整程序：

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

你要注意：

```text
这里没有删除 MC_MODE_* 宏。
C 文件还需要这些宏。
C++ 只是内部换成 enum class。
```

## 第 2 步：motor_control_config.hpp

这个文件只做一件事：把当前 C 配置宏整理成 C++ 侧的编译期配置。

它不是运行时参数表，也不是要代替 Ozone 调参入口。它的第一版只是“镜像”：

```text
当前数值仍然来自 BoardConfig.h / TuneConfig.h / motor_control_internal.h。
C++ 新代码不再到处直接写 CTRL_* / MOT_* / MC_*。
等对应 C 模块迁完，再让这个 config 成为真正的参数源头。
```

### 为什么用 struct

`struct` 在这里不是为了创建对象，而是为了给参数分组。

如果不用 `struct`，参数会散在 namespace 里：

```cpp
static constexpr int16_t current_ref_limit = CTRL_CUR_REF_LIMIT;
static constexpr int16_t current_voltage_limit = CTRL_CUR_V_LIMIT;
static constexpr int16_t speed_kp = CTRL_SPD_KP;
static constexpr int16_t speed_ki = CTRL_SPD_KI;
```

这样一多就难读。用 `struct` 后，含义更清楚：

```cpp
CurrentLoopConfig::ref_limit
CurrentLoopConfig::voltage_limit
SpeedLoopConfig::kp
SpeedLoopConfig::ki
EncoderConfig::counts_per_rev
```

读代码时你能马上知道这个参数属于哪一类控制逻辑。

这里的 `struct` 没有成员变量实例，不需要这样用：

```cpp
CurrentLoopConfig cfg{};
```

我们不会创建对象，所以不会给 RAM 增加一份 `cfg`。

### 为什么用 static

`static` 放在结构体成员上，意思是这个值属于类型本身，不属于某个对象。

也就是说我们这样访问：

```cpp
SpeedLoopConfig::kp
```

而不是：

```cpp
SpeedLoopConfig cfg{};
cfg.kp;
```

这很适合固件配置，因为这些参数不是每个对象一份，而是整个工程一份。

### 为什么用 constexpr

`constexpr` 表示这个值可以在编译期确定。

它带来三个好处：

```text
可以参与 static_assert
可以作为模板参数或数组长度这类编译期输入
优化后通常就是立即数，不需要运行时读取变量
```

例如：

```cpp
static_assert(SpeedLoopConfig::err_shift < 15U, "speed PI shift too large");
```

如果有人把速度环 shift 配错，编译阶段就失败，不用等到上板后才发现 PI 计算溢出。

再例如：

```cpp
cms32::support::rpm_to_speed_counts<EncoderConfig::counts_per_rev>(rpm);
```

`counts_per_rev` 是编译期常量，模板函数能直接按当前编码器比例生成代码。

### 它占不占 RAM

一般不占。

```cpp
struct SpeedLoopConfig
{
    static constexpr int16_t kp = CTRL_SPD_KP;
};
```

这种写法不是在结构体对象里放了一个 `kp` 字段。它是一个编译期常量。正常用法下，编译器会把它直接当数字用。

只有你刻意去取地址，才可能让编译器给它生成可寻址的符号：

```cpp
const int16_t* p = &SpeedLoopConfig::kp;
```

我们不这么写。

### 为什么不继续直接用宏

宏的问题是它没有类型、没有作用域、也不会出现在调试类型系统里。

```c
#define CTRL_SPD_KP 32
```

预处理后只是数字 `32`。你无法表达它属于速度环，也无法限制它只能被速度环使用。

`static constexpr` 至少能表达：

```cpp
SpeedLoopConfig::kp
```

这比裸 `CTRL_SPD_KP` 更接近“这个值属于速度环配置”。

### 为什么现在还要引用宏

因为这是混编阶段。

当前 C 文件仍然在用：

```text
CTRL_CUR_REF_LIMIT
CTRL_SPD_KP
MOT_SENSOR_CPR
MOT_SENSOR_POLE_PAIRS
```

如果现在直接把宏删掉，C 快环也要一起大改，风险太高。

所以现在采用两步走：

```text
第一步：
  C 宏仍是唯一数值源头。
  C++ config 只是镜像和分组。

第二步：
  对应 C 模块迁到 C++ 后，把参数源头移到 C++ config。
  再删除不需要的业务宏。
```

### 完整程序

```cpp
#pragma once

#include <stdint.h>

#include "Config.h"
#include "motor_control_internal.h"

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
};

struct EncoderConfig
{
    static constexpr int32_t counts_per_rev =
        static_cast<int32_t>(MOT_SENSOR_CPR) * static_cast<int32_t>(MOT_SENSOR_POLE_PAIRS);
    static constexpr int8_t direction = MOT_SENSOR_DIR;
    static constexpr int16_t elec_zero = MOT_ELEC_ZERO;
};

static_assert(CurrentLoopConfig::ref_limit > 0, "current ref limit must be positive");
static_assert(CurrentLoopConfig::voltage_limit > 0, "current voltage limit must be positive");
static_assert(CurrentLoopConfig::pi_shift < 15U, "current PI shift too large");
static_assert(SpeedLoopConfig::estimate_hz > 0, "speed estimate frequency must be positive");
static_assert(SpeedLoopConfig::err_shift < 15U, "speed PI shift too large");
static_assert(SpeedLoopConfig::ref_limit_rpm > 0, "speed rpm limit must be positive");
static_assert(EncoderConfig::counts_per_rev > 0, "invalid encoder scale");
static_assert((EncoderConfig::direction == 1) || (EncoderConfig::direction == -1),
              "MOT_SENSOR_DIR must be 1 or -1");

} // namespace cms32::motor
```

你要理解：

```text
struct 是分组，不是一定要创建对象。
static 是属于类型，不属于对象。
constexpr 是编译期常量，不是运行时变量。
static_assert 是编译期护栏。
```

这一层和 SupportCpp 的思路一样：把确定的规则尽量放到编译期，让错误早点暴露，让运行时代码保持简单。

## 第 3 步：motor_command_sanitizer.hpp

这个文件只做命令清洗。

命令清洗包含：

```text
从 volatile MotorControlCommand_t 逐字段复制
限制 iq/current/speed/vf 参数范围
speed_ref_rpm 非 0 时转换成 speed_ref
返回普通 MotorControlCommand_t
```

它不做：

```text
不关中断
不写 s_mc
不读硬件
不改 PWM
不进入安全态
```

为什么要单独拆：

```text
命令复制和限幅是纯值处理
以后串口命令、Ozone 命令、自动流程命令都能复用同一套规则
这部分最适合单独测试或单独编译检查
```

完整程序：

```cpp
#pragma once

#include "BoardConfig.h"
#include "MotorControl.h"
#include "TuneConfig.h"
#include "motor_control_internal.h"

#include "clamp.hpp"
#include "units.hpp"

namespace cms32::motor
{

class CommandSanitizer
{
public:
    MotorControlCommand_t sanitize(const volatile MotorControlCommand_t& input) const noexcept
    {
        MotorControlCommand_t out{};

        out.enable = input.enable;
        out.control_mode = input.control_mode;
        out.id_ref = input.id_ref;
        out.iq_ref = input.iq_ref;
        out.speed_ref = input.speed_ref;
        out.speed_ref_rpm = input.speed_ref_rpm;
        out.iq_limit = input.iq_limit;
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

        out.iq_limit = positive_limit(out.iq_limit, CTRL_CUR_REF_LIMIT);
        out.current_kp = cms32::support::clamp<int16_t>(out.current_kp, 0, 32767);
        out.current_ki = cms32::support::clamp<int16_t>(out.current_ki, 0, 32767);
        out.speed_kp = cms32::support::clamp<int16_t>(out.speed_kp, 0, 32767);
        out.speed_ki = cms32::support::clamp<int16_t>(out.speed_ki, 0, 32767);
        out.current_v_limit =
            positive_limit(out.current_v_limit, static_cast<int16_t>(PWM_SVPWM_V_LIMIT));
        out.id_ref = symmetric_limit(out.id_ref, CTRL_CUR_REF_LIMIT);
        out.iq_ref = symmetric_limit(out.iq_ref, out.iq_limit);
        out.open_loop_speed_ref =
            cms32::support::clamp<int32_t>(out.open_loop_speed_ref,
                                           -CTRL_SPD_REF_LIMIT,
                                           CTRL_SPD_REF_LIMIT);

        if (out.speed_ref_rpm != 0)
        {
            out.speed_ref =
                cms32::support::rpm_to_speed_counts<MC_SPEED_COUNTS_PER_REV>(
                    cms32::support::Rpm{out.speed_ref_rpm})
                    .value;
        }

        out.speed_ref =
            cms32::support::clamp<int32_t>(out.speed_ref, -CTRL_SPD_REF_LIMIT, CTRL_SPD_REF_LIMIT);
        out.vf_voltage = symmetric_limit(out.vf_voltage, CTRL_CUR_V_LIMIT);

        return out;
    }

private:
    static constexpr int16_t symmetric_limit(int16_t value, int16_t limit) noexcept
    {
        return cms32::support::clamp<int16_t>(value, static_cast<int16_t>(-limit), limit);
    }

    static constexpr int16_t positive_limit(int16_t value, int16_t limit) noexcept
    {
        const int32_t magnitude = (value < 0) ? -static_cast<int32_t>(value) : value;
        return static_cast<int16_t>(cms32::support::clamp<int32_t>(magnitude, 0, limit));
    }
};

} // namespace cms32::motor
```

你要理解：

```text
volatile 只存在于输入命令入口
清洗后的 out 是普通结构体
ADC 快环只读取 s_mc.command 里的普通副本
```

## 第 4 步：单独编译检查三个头文件

写完三个头文件后，先不要改 CMake。

先跑一个临时编译检查：

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/MotorControl/Cpp \
  -I Firmware/MotorControl/C \
  -I Firmware/MotorControl/Inc \
  -I Firmware/MotorControl/Algorithm \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/Board/Inc \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_motor_shell_headers.o - <<'CPP'
#include "motor_control_types.hpp"
#include "motor_control_config.hpp"
#include "motor_command_sanitizer.hpp"

int main()
{
    return 0;
}
CPP
```

如果这里失败，先修头文件，不要继续搬 shell。

## 第 4 步：motor_control_core.cpp

这个文件替代 `motor_control_c.c` 参与构建。

它必须继续定义：

```text
g_motor_cmd
g_motor_watch
MotorControl_Init()
MotorControl_ApplyCommand()
MotorControl_RunSlowLoop()
MotorControl_FastLoopFromAdcIrq()
MotorControl_GetWatch()
MotorControl_UpdateWatch()
```

注意 include 写法：

```cpp
extern "C"
{
#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_pwm.h"
#include "motor_control_internal.h"
#include "motor_control_vf.h"
}
```

原因是这些 C 头里有 C 函数声明。放在 `extern "C"` 里面，C++ 链接时才会找 C 符号名。

完整程序：

```cpp
#include "BoardConfig.h"
#include "MotorControl.h"

#include "TuneConfig.h"
#include "foc_math.h"

extern "C"
{
#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_pwm.h"
#include "motor_control_internal.h"
#include "motor_control_vf.h"
}

#include "enum_utils.hpp"
#include "irq_guard.hpp"
#include "motor_command_sanitizer.hpp"
#include "motor_control_types.hpp"

#include <stdint.h>

volatile MotorControlCommand_t g_motor_cmd = {
    0U,
    0U,
    0,
    0,
    0,
    0,
    CTRL_SPD_IQ_LIMIT,
    CTRL_CUR_KP,
    CTRL_CUR_KI,
    CTRL_SPD_KP,
    CTRL_SPD_KI,
    CTRL_CUR_V_LIMIT,
    OL_SPEED_REF,
    OL_VF_VOLTAGE,
    OL_IF_ID_REF,
    OL_IF_IQ_REF,
    OL_TIMEOUT_MS,
    0,
    0,
};

volatile MotorControlWatch_t g_motor_watch;

namespace
{

using cms32::motor::ControlFault;
using cms32::motor::ControlMode;
using cms32::motor::ControlState;
using cms32::motor::is_closed_loop_mode;
using cms32::motor::is_supported_run_mode;
using cms32::motor::to_control_mode;
using cms32::support::to_underlying;

MotorControlCState s_mc;

constexpr bool command_enabled(const MotorControlCommand_t& command) noexcept
{
    return command.enable != 0U;
}

constexpr ControlMode active_mode_for(const MotorControlCommand_t& command) noexcept
{
    return command_enabled(command) ? to_control_mode(command.control_mode) : ControlMode::Off;
}

ControlMode current_mode() noexcept
{
    return to_control_mode(s_mc.mode);
}

ControlState current_state() noexcept
{
    return static_cast<ControlState>(s_mc.state);
}

void set_state(ControlState state) noexcept
{
    s_mc.state = to_underlying(state);
}

void set_fault(ControlFault fault) noexcept
{
    s_mc.fault = to_underlying(fault);
}

bool has_valid_encoder_for_closed_loop() noexcept
{
    return (s_mc.encoder_ok != 0U) || (s_mc.encoder_initialized == 0U);
}

bool ready_for_mode(ControlMode mode) noexcept
{
    if (mode == ControlMode::VfOpenLoop)
    {
        return s_mc.check.current_ok != 0U;
    }

    if (is_closed_loop_mode(mode))
    {
        return (s_mc.check.current_ok != 0U) && has_valid_encoder_for_closed_loop();
    }

    return false;
}

ControlFault fault_for_not_ready(ControlMode mode) noexcept
{
    if (s_mc.check.current_ok == 0U)
    {
        return ControlFault::Current;
    }

    if (is_closed_loop_mode(mode) && (s_mc.check.ma600_ok == 0U))
    {
        return ControlFault::Encoder;
    }

    return ControlFault::Current;
}

bool safe_state_required() noexcept
{
    return (current_state() != ControlState::Fault) || (s_mc.pwm_output != 0U);
}

void reset_for_mode_change(ControlMode next_mode) noexcept
{
    MotorControl_SpeedReset(&s_mc);
    MotorControl_CurrentReset(&s_mc);

    if (next_mode == ControlMode::VfOpenLoop)
    {
        MotorControlVf_ResetForMode(to_underlying(next_mode));
    }
}

void apply_vf_voltage_mirror(const MotorControlCommand_t& command) noexcept
{
    if ((s_mc.enabled != 0U) && (current_mode() == ControlMode::VfOpenLoop))
    {
        curr_set_vf_voltage(command.vf_voltage);
    }
    else
    {
        curr_set_vf_voltage(0);
    }
}

void enter_idle_if_disabled() noexcept
{
    if (s_mc.enabled != 0U)
    {
        return;
    }

    if ((s_mc.pwm_output != 0U) || (current_state() != ControlState::Idle))
    {
        MotorControl_InternalEnterSafeState(&s_mc);
    }

    set_state(ControlState::Idle);
    set_fault(ControlFault::None);
}

void refresh_slow_checks(ControlMode mode) noexcept
{
    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    s_mc.check.current_ok = MotorControl_InternalCurrentOk(&s_mc);
    s_mc.check.pwm_off_safe = (pwm_is_off_safe() != 0U) ? 1U : 0U;
    s_mc.check.ma600_ok = is_closed_loop_mode(mode)
                               ? static_cast<uint8_t>(has_valid_encoder_for_closed_loop())
                               : static_cast<uint8_t>(mode == ControlMode::VfOpenLoop);
    s_mc.check.ready_closed_loop = static_cast<uint8_t>(ready_for_mode(mode));
}

void enter_ready_state(ControlMode mode) noexcept
{
    if (current_state() != ControlState::ClosedLoop)
    {
        if (mode != ControlMode::VfOpenLoop)
        {
            MotorControl_SpeedReset(&s_mc);
        }
        MotorControl_CurrentReset(&s_mc);
    }

    set_state(ControlState::ClosedLoop);
    set_fault(ControlFault::None);
}

void enter_fault_state(ControlFault fault) noexcept
{
    const bool should_enter_safe = safe_state_required();

    set_state(ControlState::Fault);
    set_fault(fault);

    if (should_enter_safe)
    {
        MotorControl_InternalEnterSafeState(&s_mc);
    }
}

} // namespace

extern "C" void MotorControl_Init(void)
{
    set_state(ControlState::Idle);
    set_fault(ControlFault::None);
    s_mc.enabled = 0U;
    s_mc.mode = to_underlying(ControlMode::Off);
    s_mc.pwm_output = 0U;
    s_mc.command_apply_count = 0U;
    s_mc.slow_loop_count = 0U;
    s_mc.fast_loop_count = 0U;
    s_mc.speed_reset_count = 0U;
    s_mc.safe_state_count = 0U;
    s_mc.speed_loop_count = 0U;
    s_mc.speed_deadband_count = 0U;
    s_mc.current_loop_div = 0U;
    s_mc.speed_sample_div = 0U;

    MotorControl_EncoderReset(&s_mc);

    foc_pi_init(&s_mc.speed_pi, CTRL_SPD_KP, CTRL_SPD_KI, -CTRL_SPD_IQ_LIMIT,
                CTRL_SPD_IQ_LIMIT, CTRL_SPD_ERR_SHIFT);
    foc_pi_init(&s_mc.current_pi_d, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);
    foc_pi_init(&s_mc.current_pi_q, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);

    s_mc.current = FocPhaseCurrent_t{0, 0, 0};
    s_mc.current_dq = FocDq_t{0, 0};
    s_mc.id_ref_active = 0;
    s_mc.iq_ref_active = 0;
    s_mc.voltage_ab = FocAlphaBeta_t{0, 0};
    s_mc.voltage_dq = FocDq_t{0, 0};
    s_mc.voltage_theta = 0U;
    s_mc.duty = FocDuty_t{PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    s_mc.voltage_limited = 0U;
    s_mc.check = MotorControlCheck_t{0U, 1U, 1U, 0U};

    MotorControlVf_Init();
    pwm_off();
}

extern "C" void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command)
{
    if (command == nullptr)
    {
        return;
    }

    s_mc.command_apply_count++;

    const cms32::motor::CommandSanitizer sanitizer;
    const MotorControlCommand_t next_command = sanitizer.sanitize(*command);
    const ControlMode next_mode = active_mode_for(next_command);

    {
        const cms32::support::AdcIrqGuard guard;
        (void)guard;

        if (next_mode != current_mode())
        {
            reset_for_mode_change(next_mode);
        }

        s_mc.enabled = static_cast<uint8_t>(command_enabled(next_command));
        s_mc.mode = to_underlying(next_mode);
        s_mc.command = next_command;
    }

    apply_vf_voltage_mirror(next_command);
    enter_idle_if_disabled();
}

extern "C" void MotorControl_RunSlowLoop(void)
{
    const ControlMode mode = current_mode();
    refresh_slow_checks(mode);

    if (s_mc.enabled == 0U)
    {
        s_mc.slow_loop_count++;
        return;
    }

    if (is_supported_run_mode(mode))
    {
        if (s_mc.check.ready_closed_loop != 0U)
        {
            enter_ready_state(mode);
        }
        else
        {
            enter_fault_state(fault_for_not_ready(mode));
        }
    }
    else
    {
        enter_fault_state(ControlFault::UnsupportedMode);
    }

    s_mc.slow_loop_count++;
}

extern "C" uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    const uint8_t sample_ready = bsp_adc_irq();
    if (sample_ready == 0U)
    {
        return 0U;
    }

    if ((s_mc.enabled != 0U) && (current_state() == ControlState::ClosedLoop))
    {
        const ControlMode mode = current_mode();

        if (mode == ControlMode::VfOpenLoop)
        {
            MotorControlVf_RunFastLoop(&s_mc);
        }
        else if (mode == ControlMode::Speed)
        {
            MotorControl_CurrentRunFastLoop(&s_mc, 1U);
        }
        else if (mode == ControlMode::Current)
        {
            MotorControl_CurrentRunFastLoop(&s_mc, 0U);
        }
    }

    return 1U;
}

extern "C" void MotorControl_GetWatch(MotorControlWatch_t* out)
{
    if (out == nullptr)
    {
        return;
    }

    MotorControl_WatchFill(&s_mc, out);
}

extern "C" void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out)
{
    if (out == nullptr)
    {
        return;
    }

    MotorControlWatch_t snapshot;
    MotorControl_WatchFill(&s_mc, &snapshot);
    MotorControl_WatchCopyToVolatile(out, &snapshot);
}
```

## 第 5 步：CMake 接入

先新增 C++ 目录变量：

```cmake
set(CMS32_MOTOR_CPP_DIR ${CMS32_MOTOR_DIR}/Cpp)
```

然后把 MotorControl 源列表里的：

```cmake
${CMS32_MOTOR_C_DIR}/motor_control_c.c
```

替换成：

```cmake
${CMS32_MOTOR_CPP_DIR}/motor_control_core.cpp
```

完整片段如下：

```cmake
set(CMS32_MOTOR_CONTROL_C_SOURCES
    ${CMS32_MOTOR_CPP_DIR}/motor_control_core.cpp
    ${CMS32_MOTOR_C_DIR}/motor_control_current.c
    ${CMS32_MOTOR_C_DIR}/motor_control_encoder.c
    ${CMS32_MOTOR_C_DIR}/motor_control_output.c
    ${CMS32_MOTOR_C_DIR}/motor_control_vf.c
    ${CMS32_MOTOR_C_DIR}/motor_control_watch.c
)

add_library(cms32_motor_control_c STATIC
    ${CMS32_MOTOR_CONTROL_C_SOURCES}
)
target_include_directories(cms32_motor_control_c PUBLIC
    ${CMS32_MOTOR_C_DIR}
    ${CMS32_MOTOR_CPP_DIR}
    ${CMS32_MOTOR_INC_DIR}
)
target_link_libraries(cms32_motor_control_c PUBLIC
    cms32_bsp
    cms32_foc_algorithm
    cms32_support_cpp
)
```

为什么要链接 `cms32_support_cpp`：

```text
motor_control_core.cpp include 了 irq_guard.hpp、enum_utils.hpp
motor_command_sanitizer.hpp include 了 clamp.hpp、units.hpp
cms32_support_cpp 提供 Support include path 和 C++ 编译选项
```

注意：

```text
如果你的 CMake 已经因为 UART 调试加了 uart.c / board_uart.c，不要删它们。
这里只改 MotorControl 这一段。
```

## 第 6 步：编译验证

先跑真实构建：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

如果通过，再看符号：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "MotorControl_|g_motor_"
```

你希望看到：

```text
MotorControl_Init
MotorControl_ApplyCommand
MotorControl_RunSlowLoop
MotorControl_FastLoopFromAdcIrq
MotorControl_GetWatch
MotorControl_UpdateWatch
g_motor_cmd
g_motor_watch
```

不要出现一堆异常/RTTI 相关符号：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "__cxa|typeinfo|vtable|throw|exception"
```

正常情况应该没有输出。

## 第 7 步：上板前检查

上板前先确认慢环行为没有被改坏：

```text
enable = 0 时：
  state 应为 idle
  fault 应为 none
  PWM 应处于 safe/off

enable = 1, control_mode = speed 时：
  current_ok 正常时可以进入 closed_loop
  encoder 未初始化时允许第一次进入
  encoder 明确坏角后应进入 fault encoder

enable = 1, control_mode = vf 时：
  不要求 encoder ready
  current_ok 正常时可以进入 closed_loop
  vf_voltage 会同步到 curr_set_vf_voltage()

control_mode = 4/5 或其它不支持值时：
  state 应进入 fault
  fault_reason 应为 unsupported mode
  PWM 应关闭
```

## 常见错误

### 错误 1：把临界区包太大

错误写法：

```cpp
{
    const cms32::support::AdcIrqGuard guard;
    const MotorControlCommand_t next_command = sanitizer.sanitize(*command);
    curr_set_vf_voltage(next_command.vf_voltage);
    MotorControl_InternalEnterSafeState(&s_mc);
}
```

问题：

```text
sanitize 是普通计算，不需要关 ADC IRQ
curr_set_vf_voltage 会碰板级状态，不应该在这个临界区里
EnterSafeState 会关 PWM 和清状态，也不应该被包进去
```

正确写法：

```cpp
const MotorControlCommand_t next_command = sanitizer.sanitize(*command);

{
    const cms32::support::AdcIrqGuard guard;
    (void)guard;
    s_mc.command = next_command;
}
```

### 错误 2：把 MotorControl 业务放进 Support

不要写：

```text
Firmware/Support/motor_mode.hpp
Firmware/Support/motor_command_sanitizer.hpp
```

原因：

```text
Support 是通用工具层
MotorControl 的 mode/fault/command 是业务层规则
放进 Support 会让底层工具反向依赖业务含义
```

正确位置：

```text
Firmware/MotorControl/Cpp/motor_control_types.hpp
Firmware/MotorControl/Cpp/motor_command_sanitizer.hpp
```

### 错误 3：忘记 extern "C"

如果 `motor_control_core.cpp` 里直接 include C 头，可能链接失败。

正确写法：

```cpp
extern "C"
{
#include "motor_control_internal.h"
#include "motor_control_vf.h"
}
```

### 错误 4：同时编译旧 shell 和新 shell

不要让 CMake 同时包含：

```cmake
${CMS32_MOTOR_C_DIR}/motor_control_c.c
${CMS32_MOTOR_CPP_DIR}/motor_control_core.cpp
```

否则会重复定义：

```text
g_motor_cmd
g_motor_watch
MotorControl_Init()
MotorControl_ApplyCommand()
...
```

### 错误 5：一上来就重写快环

不要在第四阶段改：

```text
motor_control_current.c
motor_control_encoder.c
motor_control_output.c
```

原因：

```text
快环已经和 ADC/PWM/MA600 时序绑定
第四阶段目标只是让外壳边界清楚
快环小组件应放到第五阶段以后
```

## 你应该按这个顺序写

```text
1. 新建 Firmware/MotorControl/Cpp/
2. 写 motor_control_types.hpp
3. 写 motor_control_config.hpp
4. 写 motor_command_sanitizer.hpp
5. 用临时 arm-none-eabi-g++ 命令检查三个头文件
6. 写 motor_control_core.cpp
7. CMake 把 motor_control_c.c 换成 motor_control_core.cpp
8. cmake --build --preset gcc-debug --target cms32foc
9. nm 检查 MotorControl 符号和异常/RTTI 符号
10. 上板只测 enable/mode/state/fault/PWM safe 行为
```

如果第 8 步失败，先不要改快环。优先检查：

```text
include path 是否包含 Firmware/MotorControl/Cpp
cms32_support_cpp 是否链接进 cms32_motor_control_c
C 头是否放进 extern "C"
是否还同时编译 motor_control_c.c
```
