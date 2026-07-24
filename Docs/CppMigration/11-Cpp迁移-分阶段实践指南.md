# C/C++ 混编分阶段实践指南

本文是当前工程的 C/C++ 混编分阶段实践文档。它不再把目标写成“把所有 C 文件改成 C++”，而是按嵌入式 Modern C++ 的思路，把 C 和 C++ 各自放在最适合的位置。

> 当前最终落地形态见 `25-Cpp迁移-C_Cpp混编最小封装落地指南.md`。本文保留阶段教学
> 和历史代码写法；当前主线不再使用 `g_motor_*` 或 `MotorControlCState` 作为正式状态入口。

参考方向：

```text
https://awesome-embedded-learning-studio.github.io/Tutorial_AwesomeModernCPP/vol8-domains/embedded/
```

本文和其它文档的关系：

```text
10-Cpp迁移-路线图.md                  讲长期路线和阶段顺序
11-Cpp迁移-分阶段实践指南.md         讲当前阶段怎么混编、边界怎么落
12-Cpp迁移-Support层动手指南.md              专门讲 Firmware/Support/*.hpp
13-Cpp迁移-MotorControl核心调度层动手指南.md    专门讲第四阶段核心调度层怎么手写
14-Cpp迁移-MotorControl当前状态教学.md  按当前 checkout 状态讲下一步怎么手写
15-Cpp迁移-MotorControl数学小组件动手指南.md     专门讲 Stage 5 数学小组件怎么手写
16-Cpp迁移-FOC坐标变换动手指南.md         专门讲 FOC transform 类型包装
17-Cpp迁移-编码器与速度估算动手指南.md         专门讲 Stage 6 编码器和速度估算
18-Cpp迁移-串口通信动手指南.md                 专门讲后续串口协议和命令路由
19-Cpp迁移-Board薄封装动手指南.md     专门讲 Board 层薄 C++ wrapper
02-当前程序-理论与参数指南.md 讲当前程序和参数数学
```

## 总方向：不是全 C++，是混编

这个工程最适合的方向是：

```text
C 负责硬件边界、C ABI、已验证快环和调试可见结构
C++ 负责类型安全、状态机、配置校验、命令解析和纯算法封装
```

不要把“用 C++”理解成：

```text
所有 .c 立刻改成 .cpp
所有 #define 立刻删除
所有结构体都包 getter/setter
所有模块都抽 class
```

更准确的目标是：

```text
寄存器访问仍直接
中断入口仍清楚
Ozone/watch 数字仍稳定
快环行为先不变
新写的规则和状态逐步类型安全
业务宏最终退场
```

## 当前阶段状态

当前工程已经走到第四阶段接入后：

```text
Stage 1 Support:
  Firmware/Support/*.hpp 已有 clamp / units / irq_guard / ring_buffer 等基础工具。

Stage 2 ScrewAxis:
  screw_axis.cpp 已经使用 enum class、constexpr 和 helper，保留 ScrewAxis_* C ABI。

Stage 3 UART bring-up:
  Board 层 UART 调通，底层 board_uart.c 保持 C。
  后续协议 parser / router 再用 C++。

Stage 4 MotorControl 核心调度层:
  core.cpp 已经接入 CMake，替代 motor_control_c.c。
  命令 snapshot/sanitize、模式判断、状态枚举、慢环状态机已迁到 C++。
  Current/Speed/Encoder/Output/VF/Watch 快环相关文件仍保持 C。
  下一步是上板验证 Stage 4 行为，然后进入数学小组件。
```

当前 `Firmware/MotorControl/Core/` 里的第四阶段文件：

```text
types.hpp
config.hpp
command_sanitizer.hpp
core.cpp
```

CMake 变量也统一为：

```cmake
set(CMS32_MOTOR_CPP_DIR ${CMS32_MOTOR_DIR}/Cpp)
```

不要使用大小写混杂的 `CMS32_MOTOR_Cpp_DIR`。

## 混编边界

### C 继续负责什么

这些部分继续优先用 C：

```text
startup / IRQ vector
CMSIS / vendor driver
底层寄存器访问
PWM / ADC / PGA / MA600 最底层驱动
已验证 Current/Speed 快环 C 实现
Ozone 可见的 C ABI 结构体和全局变量
```

原因：

```text
寄存器访问要透明
中断入口要明确
调试器看 C 结构体最直接
vendor 头文件和驱动天然是 C
快环在没有对照测试前不适合大改
```

### C++ 负责什么

这些部分优先用 C++：

```text
状态机 enum class
单位类型 Rpm / CurrentCount / VoltageCount / SpeedCounts / Angle16
命令清洗 CommandSanitizer
配置镜像 constexpr + static_assert
串口协议 parser / router
固定容量 ring buffer
低频固定函数指针表
纯数学小组件 PI / slew / filter / estimator
短临界区 RAII guard
```

原因：

```text
这些地方容易因为裸整数混用出错
这些地方不需要动态内存
这些地方可以零开销内联
这些地方适合编译期检查
函数指针只用于低频命令路由或后端表，不进入 FOC 快环内层
```

### 对外边界继续用 C ABI

对外入口继续保留：

```c
void MotorControl_Init(void);
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command);
void MotorControl_RunSlowLoop(void);
uint8_t MotorControl_FastLoopFromAdcIrq(void);
void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out);

void ScrewAxis_Init(void);
void ScrewAxis_Run(void);
void ScrewAxis_OnAdcSample(void);
```

C++ 文件里这样导出：

```cpp
extern "C" void MotorControl_RunSlowLoop(void)
{
    // C++ implementation, C symbol.
}
```

这样 `main.c`、中断入口、Ozone/watch 和旧 C 模块都不用知道内部已经换成 C++。

## define 怎么退场

最终确实要减少业务 `#define`，但不能一刀切。分三类处理。

### 1. vendor / CMSIS / 寄存器宏：保留

这些宏不属于业务规则：

```text
UART0
ADC_IRQn
GPIO register bit
CMSIS intrinsic
vendor driver constants
```

没必要强行替换。

### 2. C/C++ 过渡协议宏：先保留，再收敛

例如：

```c
#define MC_STATE_IDLE 0U
#define MC_MODE_SPEED 2U
#define MC_FAULT_ENCODER 4U
```

只要 C 文件还在用，就先保留。C++ 里先包装：

```cpp
enum class ControlMode : uint8_t
{
    Off = MC_MODE_OFF,
    Current = MC_MODE_CURRENT,
    Speed = MC_MODE_SPEED,
    VfOpenLoop = MC_MODE_VF_OPEN_LOOP,
};
```

这一步的目的不是“保留 define”，而是保证迁移期间只有一套数字协议。等使用这些宏的 C 文件都迁完，再把宏删除，让 `enum class` 成为唯一来源。

### 3. 业务配置宏：迁成 constexpr config

例如：

```c
#define CTRL_SPD_KP 32
#define CTRL_SPD_KI 3
#define CTRL_SPD_ERR_SHIFT 10u
```

迁移第一步不是删除，而是镜像：

```cpp
struct SpeedLoopConfig
{
    static constexpr int16_t kp = CTRL_SPD_KP;
    static constexpr int16_t ki = CTRL_SPD_KI;
    static constexpr uint8_t shift = CTRL_SPD_ERR_SHIFT;
};

static_assert(SpeedLoopConfig::shift < 15U);
```

等速度环实现迁到 C++ 后，再把值写成 C++ 原生来源：

```cpp
struct SpeedLoopConfig
{
    static constexpr int16_t kp = 32;
    static constexpr int16_t ki = 3;
    static constexpr uint8_t shift = 10U;
};
```

最后删除旧宏。

## Stage 1: Support 层

Support 是无业务工具层。它不应该知道 MotorControl、ScrewAxis、Board 的业务含义。

当前目录：

```text
Firmware/Support/
├── clamp.hpp
├── enum_utils.hpp
├── irq_guard.hpp
├── low_pass.hpp
├── ring_buffer.hpp
├── slew_limiter.hpp
├── static_asserts.hpp
└── units.hpp
```

适合放这里：

```text
template clamp
unit wrapper
SPSC ring buffer
RAII IRQ guard
low pass
slew limiter
enum to_underlying
通用 static_assert helper
```

不适合放这里：

```text
ControlMode
MotorControlCommand sanitizer
ScrewHomeState
UART command id
FOC speed loop config
Board pin map
```

这些是业务层含义，应该放在自己的模块里。

验收：

```sh
arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m0plus -mthumb \
  -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit \
  -I Firmware/Support \
  -I Firmware/Board/Config \
  -I Firmware/MotorControl/Abi \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -x c++ -c -o /tmp/cms32_support_check.o - <<'CPP'
#include "clamp.hpp"
#include "enum_utils.hpp"
#include "irq_guard.hpp"
#include "low_pass.hpp"
#include "ring_buffer.hpp"
#include "slew_limiter.hpp"
#include "units.hpp"
int main() { return 0; }
CPP
```

## Stage 2: ScrewAxis 应用层 C++ 化

这一阶段已经基本完成，方向继续保持：

```text
screw_axis.h:
  保持 C ABI、Command/Watch 结构体、Ozone 可见入口。

screw_axis.cpp:
  内部用 enum class / constexpr / support::clamp 整理回零状态机。
```

对外入口不变：

```cpp
extern "C" void ScrewAxis_Init(void);
extern "C" void ScrewAxis_Run(void);
extern "C" void ScrewAxis_OnAdcSample(void);
```

当前还可以继续收敛的点：

```text
kMotorModeSpeed = 2U 这种裸数字，后续改成 MotorControl 类型层提供的 ControlMode::Speed。
speed_rpm / iq_limit 可以逐步使用 Rpm / CurrentCount。
当位置模式、软限位或多命令来源出现后，再拆 ScrewHomeController。
```

暂时不要做：

```text
不要给 g_motor_cmd 包复杂 setter/getter。
不要把一个清楚的 switch 状态机拆成一堆小类。
不要为了目录好看把 Home 过早拆出去。
```

## Stage 3: UART / Comm 混编

这一阶段分两层。

底层 Board UART 继续 C：

```text
Firmware/Board/Src/board_uart.c
Firmware/Board/Inc/board_uart.h
```

原因：

```text
寄存器访问多
IRQ 入口简单
已经调通
需要保留 END 访问结束要求
```

协议层和命令路由用 C++：

```text
Firmware/Comm/
├── serial_protocol.hpp
├── command_router.hpp
└── serial_app.cpp
```

建议接口：

```cpp
extern "C" void Serial_OnRxByteFromIrq(uint8_t byte);
extern "C" void Serial_Run(void);
```

UART ISR 里只做：

```text
读 RBR
push byte
清中断 / END
```

主循环里做：

```text
pop byte
FrameParser::feed()
CommandRouter::route()
```

这对应 embedded 教程里的 UART 路线：

```text
中断基础
lock-free SPSC ring buffer
IRQ handler 很薄
command processor 放主循环
compile-time fixed payload
```

第一版 CommandRouter 只能写公共命令：

```text
g_mc_cmd
g_screw_axis_cmd
```

不要写 MotorControl 内部 `s_mc`。

## Stage 4: MotorControl 核心调度层

这是当前已接入的阶段，仍需要上板验收。

目标：

```text
用 `core.cpp` 替代 `motor_control_c.c` 的核心调度层。
保留 MotorControl.h 的 C ABI。
不动 Current/Speed ADC 快环实现。
不动 Encoder/Output/VF C 文件。
```

### Stage 4.0 命名清理

当前统一文件名：

```text
Firmware/MotorControl/Types/types.hpp
Firmware/MotorControl/Config/config.hpp
Firmware/MotorControl/Types/command_sanitizer.hpp
Firmware/MotorControl/Core/core.cpp
```

不要再使用旧草稿名：

```text
motor_control_type.hpp
motor_contrl_core.cpp
```

CMake 统一：

```cmake
set(CMS32_MOTOR_CPP_DIR ${CMS32_MOTOR_DIR}/Cpp)
```

### Stage 4.1 types.hpp

先让 C++ 代码不再直接传裸 `uint8_t`。

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

这里仍然引用 `MC_*` 宏，是因为 C ABI 状态结构和部分 C 对照文件还没退场。当前主线已接入 `current.cpp`、`watch.cpp`、`output.cpp`、`vf.cpp` 等 C++ 文件，但部分头文件和旧 C 对照文件仍会看到这些底层数值；后续公共 ABI 收口后，再让 enum 成为唯一数字源。

### Stage 4.2 config.hpp

把业务配置宏先镜像成 C++ `constexpr`。

为什么用 `struct + static constexpr`：

```text
struct:
  只负责分组，例如 CurrentLoopConfig / SpeedLoopConfig / EncoderConfig。
  这里不创建对象，不给每个对象存一份参数。

static:
  表示参数属于类型本身，用 CurrentLoopConfig::ref_limit 访问。
  不是运行时实例字段。

constexpr:
  表示值可在编译期确定。
  可以参与 static_assert、模板参数和编译期换算。
  正常使用时通常不会占 RAM。
```

这和 `12-Cpp迁移-Support层动手指南.md` 里的思路一致：

```text
能在编译期确定的规则，不放到运行时判断。
能按类型分组的参数，不散落成一堆裸宏。
能让编译器检查的约束，不等上板后才查。
```

当前阶段还不能直接删除宏，因为 C 快环仍然使用 `CTRL_*` / `MOT_*`。所以第一版 `config.hpp` 是“镜像层”：

```text
数值源头仍是 BoardConfig.h / TuneConfig.h / motor_control_internal.h。
C++ 新代码优先使用 CurrentLoopConfig::kp 这种名字。
等对应 C 模块迁完，再把参数源头移到 C++ config 并删除业务宏。
```

```cpp
#pragma once

#include <stdint.h>

#include "Config.h"

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

static_assert(CurrentLoopConfig::pi_shift < 15U, "current PI shift too large");
static_assert(SpeedLoopConfig::err_shift < 15U, "speed PI shift too large");
static_assert(SpeedLoopConfig::estimate_hz > 0, "invalid speed estimate frequency");
static_assert(SpeedLoopConfig::diff_spike_rpm > 0, "invalid speed spike limit");
static_assert((SpeedLoopConfig::pos_deadband >= 0) &&
                  (SpeedLoopConfig::pos_deadband <= 32767),
              "speed position deadband must fit int16 delta");
static_assert(EncoderConfig::counts_per_rev > 0, "invalid encoder scale");
static_assert(EncoderConfig::motor_pole_pairs > 0, "invalid motor pole pairs");
static_assert((EncoderConfig::direction == 1) || (EncoderConfig::direction == -1),
              "MOT_SENSOR_DIR must be 1 or -1");
static_assert((EncoderConfig::max_step_raw > 0U) && (EncoderConfig::max_step_raw <= 32767U),
              "encoder max step must fit int16 delta");

} // namespace cms32::motor
```

这一步不会删宏，只是让 C++ 新代码开始依赖 C++ config。等 C 文件迁移完，再把这些值变成真正源头。

### Stage 4.3 command_sanitizer.hpp

只做纯值处理：

```text
从 volatile command 逐字段复制
限幅
rpm -> speed count/s
返回普通 MotorControlCommand_t
```

它不做：

```text
不关中断
不写 s_mc
不读硬件
不关 PWM
```

这样它可以单独编译检查，也方便以后 Comm/Ozone/自动流程共用同一套命令规则。

### Stage 4.4 core.cpp

替代 `motor_control_c.c` 参与构建，但只迁核心调度层：

```text
MotorControl_Init()
MotorControl_ApplyCommand()
MotorControl_RunSlowLoop()
MotorControl_FastLoopFromAdcIrq()
MotorControl_GetWatch()
MotorControl_UpdateWatch()
```

仍然调用这些 C 快环函数：

```text
MotorControl_CurrentRunFastLoop()
MotorControl_InternalUpdateEncoderAngle()
MotorControl_InternalUpdateEncoderSpeed()
MotorControl_InternalApplyVoltageVector()
MotorControlVf_RunFastLoop()
MotorControl_WatchFill()
MotorControl_WatchCopyToVolatile()
```

临界区原则：

```cpp
const MotorControlCommand_t next_command = sanitizer.sanitize(sanitizer.snapshot(*command));

{
    const cms32::support::AdcIrqGuard guard;
    (void)guard;
    s_mc.enabled = static_cast<uint8_t>(next_command.enable != 0U);
    s_mc.mode = to_underlying(next_mode);
    s_mc.current_command = sanitizer.current_command(next_command);
    s_mc.speed_command = sanitizer.speed_command(next_command);
    s_mc.vf_command = sanitizer.vf_command(next_command);
}
```

`AdcIrqGuard` 只包共享状态写入。不要把限幅、串口解析、watch 填充、FOC 计算包进去。

### Stage 4 验收

构建：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

符号：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "MotorControl_|g_motor_"
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "__cxa|typeinfo|vtable|throw|exception"
```

第二条正常应无输出。

行为：

```text
enable=0:
  state idle，PWM safe。

Speed mode:
  current_ok 和 encoder_ok 正常时进入 closed loop。

VF mode:
  不依赖 encoder ready，但仍检查 current_ok。

不支持 mode:
  fault_reason = unsupported mode，PWM safe。
```

## Stage 5: FOC 纯数学小组件

这一阶段才考虑 `foc_math.c`。

适合迁：

```text
Q15 sin/cos 包装
Angle16
Clarke / Park / InvPark 的类型包装
FixedPi
SlewLimiter
LowPassI32
SVPWM duty 计算
```

不建议一开始迁：

```text
整个 Current fast loop
整个 ADC IRQ 流程
Board 寄存器访问
```

推荐方式是保留 C ABI 包装：

```cpp
extern "C" FocDq_t foc_park(FocAlphaBeta_t input, uint16_t theta)
{
    return cms32::motor::park(input, cms32::support::Angle16{theta});
}
```

这样 C 文件还能继续调用旧函数，内部逐步变成 C++ 实现。

## Stage 6: Encoder 和速度估算

Encoder 适合用强类型，因为这里最容易混：

```text
MA600 raw
电角度
累计位置
speed count/s
rpm
方向
零点
```

建议拆成两层：

```text
AngleValidator:
  只判断 raw 是否可信、是否要 hold/retry。

SpeedEstimator:
  只做 raw delta -> speed count/s -> filter。
```

示例方向：

```cpp
template <int32_t SampleHz, uint8_t FilterShift>
class SpeedEstimator
{
public:
    support::SpeedCounts update(support::EncoderRaw raw) noexcept;
    void reset(support::EncoderRaw raw) noexcept;

private:
    support::EncoderRaw prev_{0U};
    support::LowPassI32<FilterShift> filter_{};
    bool initialized_{false};
};
```

不要把 SPI 读角也塞进 estimator。SPI 读取仍属于 Board/MA600 边界。

当前 Stage 6 不是继续加新 wrapper，而是把已有数学接进真实编码器路径：

```text
Firmware/MotorControl/LegacyC/motor_control_encoder.c
    -> Firmware/MotorControl/Core/encoder.cpp
```

第一版 `encoder.cpp` 只做薄接入：

```text
保留 MotorControl_* C ABI
保留 MotorControlCState 字段布局
保留 MotorControlWatch_t 的裸整数观察字段
保留 bsp_update_angle_fast() / bsp_angle_raw() / bsp_angle_ok()
只把 raw delta、速度 spike limit、deadband、speed count/s -> rpm 这些公式替换为 C++ 小组件
```

为什么不直接把 `AngleValidator` / `SpeedEstimator` 对象放进正式路径：

```text
它们有自己的 previous/filter/initialized 状态
当前正式状态已经在 MotorControlCState 里
直接接入会形成两份状态
Ozone 观察和问题定位会变差
```

所以 Stage 6.1 的原则是：

```text
无状态纯函数先进入生产路径
有状态 class 先保留为教学/对拍模型
正式状态仍放在 MotorControlCState
```

完成后，MotorControl 主链路应变成：

```text
core.cpp
current.cpp
encoder.cpp
```

## Stage 7: Board 层薄 C++ wrapper

Board 层最后做。原因：

```text
它直接碰寄存器
它和 ADC/PWM/MA600 时序绑定
改错会直接影响上板调试
```

适合先包：

```text
UART policy class
GPIO pin enum
PWM duty value object
ADC sample window helper
```

不建议马上改：

```text
foc_curr.c 整体模板化
foc_pwm.c 整体类化
vendor driver 改 C++
寄存器宏全部替换
```

Board C++ 的目标不是“看起来像 HAL”，而是：

```text
把容易混的参数类型化
把编译期固定的外设实例用模板绑定
把运行时代码保持和 C 一样直接
```

## 最终目标结构

目标不是所有文件都 `.cpp`，而是边界稳定：

```text
Firmware/Support/
  通用零开销工具，header-only 为主。

Firmware/App/
  ScrewAxis 等业务应用，可用 C++。

Firmware/Comm/
  串口协议和命令路由，C++。

Firmware/MotorControl/Abi/
  C ABI 和 Ozone 可见结构体。

Firmware/MotorControl/Core/
  MotorControl 类型、配置、慢环 shell、纯算法组件。

Firmware/MotorControl/LegacyC/
  尚未迁移的快环 C 文件，逐步减少。

Firmware/Board/
  底层寄存器和已验证外设链路，C 为主，薄 C++ wrapper 后置。
```

最终 `#define` 退场顺序：

```text
1. 新 C++ 代码不再直接使用业务宏，而用 enum class / constexpr config。
2. C 文件迁移到 C++ 后，删除对应业务宏。
3. vendor/CMSIS/寄存器宏保留。
```

## 每个阶段都要做的检查

每次改完至少跑：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

如果改了 C++ 构建路径，再查：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "__cxa|__gxx|typeinfo|vtable|throw|exception|operator new|operator delete"
```

期望无输出。

如果改了命令、状态或 watch，再上板看：

```text
g_motor_cmd 是否还能写
g_motor_watch 是否还能更新
state / control_mode / fault_reason 数字是否保持旧语义
fast_loop_count 是否继续增长
PWM safe 行为是否不变
```

混编的底线是：

```text
先保持行为，再提高类型安全。
```
