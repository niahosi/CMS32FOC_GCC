# C++ 迁移路线图

本文给当前工程的 C++ 重写顺序和模块落点。参考方向是 Awesome Modern C++ embedded 教程：

> 当前最终落地形态见 `25-Cpp迁移-C_Cpp混编最小封装落地指南.md`。本文保留路线演进
> 思路；旧 `g_motor_*` 和 `MotorControlCState` 写法不再代表当前主线。

```text
https://awesome-embedded-learning-studio.github.io/Tutorial_AwesomeModernCPP/vol8-domains/embedded/
```

如果要看每个阶段“怎么拆、完整 `.hpp/.cpp` 怎么写、代码里每个 C++ 点是什么意思”，读：

```text
Docs/CppMigration/11-Cpp迁移-分阶段实践指南.md
```

如果要按当前 checkout 的实际状态继续手写 `MotorControl` C++ shell，读：

```text
Docs/CppMigration/14-Cpp迁移-MotorControl当前状态教学.md
```

后续阶段的动手教学文档：

```text
Docs/CppMigration/15-Cpp迁移-MotorControl数学小组件动手指南.md
Docs/CppMigration/16-Cpp迁移-FOC坐标变换动手指南.md
Docs/CppMigration/17-Cpp迁移-编码器与速度估算动手指南.md
Docs/CppMigration/18-Cpp迁移-串口通信动手指南.md
Docs/CppMigration/19-Cpp迁移-Board薄封装动手指南.md
```

采用的核心原则：

```text
zero-overhead abstraction
resource and realtime constraints
type-safe register/config access
lock-free SPSC ring buffer for UART
CRTP / compile-time polymorphism
enum class / constexpr / template / static_assert
```

不采用：

```text
动态内存
异常
RTTI
iostream
std::function
std::vector / std::string
快环 virtual dispatch
复杂运行时工厂
```

## 总体边界

当前可稳定保留的 C ABI：

```text
MotorControl_Init()
MotorControl_ApplyCommand()
MotorControl_RunSlowLoop()
MotorControl_FastLoopFromAdcIrq()
MotorControl_UpdateWatch()

ScrewAxis_Init()
ScrewAxis_Run()
ScrewAxis_OnAdcSample()
```

第一阶段 C++ 重构不要破坏这些入口。`main.c` 和 `ADC_IRQHandler()` 可以继续保持 C，等 App/Comm 层稳定后再考虑 `main.cpp`。

## 第一阶段：Support 层

新建目录：

```text
Firmware/Support/
├── units.hpp
├── enum_utils.hpp
├── clamp.hpp
├── slew_limiter.hpp
├── low_pass.hpp
├── ring_buffer.hpp
├── irq_guard.hpp
└── static_asserts.hpp
```

适合使用的 C++ 特性：

```text
constexpr
template
static_assert
enum class helper
固定容量数组
```

要求：

```text
header-only
不包含 STL 动态容器
不定义全局对象
不依赖寄存器
不调用 MotorControl 内部函数
```

优先写的类型：

```cpp
struct Rpm {
    int16_t value;
};

struct CurrentCount {
    int16_t value;
};

struct VoltageCount {
    int16_t value;
};

struct SpeedCounts {
    int32_t value;
};
```

收益：

```text
避免 speed_rpm / iq_limit / voltage_count 混传
把限幅、斜坡、滤波写成可复用零开销小组件
给后续串口和 ScrewAxis 提供稳定基础
```

验收：

```sh
cmake --build build/gcc-debug --target cms32foc
cmake --build build/gcc-minsize --target cms32foc
arm-none-eabi-nm -C build/gcc-minsize/cms32foc | rg "new|delete|__cxa|__gxx|vtable|typeinfo|throw"
```

期望最后一条没有异常/RTTI/堆相关符号。

## 第二阶段：ScrewAxis

当前文件：

```text
Firmware/App/screw_axis.c
Firmware/App/screw_axis.h
```

建议迁移为：

```text
Firmware/App/screw_axis.cpp
Firmware/App/screw_axis.h
```

对外继续保留 C ABI：

```cpp
extern "C" void ScrewAxis_Init(void);
extern "C" void ScrewAxis_Run(void);
extern "C" void ScrewAxis_OnAdcSample(void);
```

内部重写点：

```text
SCREW_HOME_STATE_* -> enum class HomeState : uint8_t
SCREW_HOME_* 参数 -> constexpr
speed_rpm -> Rpm
iq_limit -> CurrentCount
home_update switch -> HomeController::run()
clamp_s16/clamp_u16 -> template clamp
app_millis -> Milliseconds wrapper 或 constexpr tick conversion
```

推荐形态：

```cpp
class ScrewHomeController {
public:
    void init();
    void run();
    void on_adc_sample();

private:
    HomeState state_{HomeState::Idle};
    uint16_t last_start_seq_{0U};
    uint32_t phase_start_ms_{0U};
    uint32_t stall_start_ms_{0U};
};
```

注意：

```text
不要 new ScrewHomeController
用一个 static 对象即可
不要把 g_motor_cmd 封装成复杂 setter/getter
当前主线已经统一到 g_screw_axis_cmd/g_screw_axis_watch
```

收益：

```text
状态机类型安全
回零参数单位清楚
后续加入限位/软限位时不继续堆 if/宏
```

验收：

```text
符号表仍只暴露 ScrewAxis_Init/Run/OnAdcSample 和 g_screw_axis_*
回零状态和 zero_encoder_pos 行为不变
MinSizeRel Flash 不异常增长
```

## 第三阶段：串口 Comm 模块

Board 层 UART bring-up 先保持 C：寄存器初始化、P06/P07 pinmux、RX ring buffer
和 `UART0_IRQHandler()` 都放在 `Firmware/Board/Src/board_uart.c`。后续真正的 Comm
协议层再用 C++ 写 parser/router，保持硬件 ISR 很薄。

建议目录：

```text
Firmware/Comm/
├── uart_driver_cms32.cpp
├── uart_driver_cms32.hpp
├── serial_protocol.cpp
├── serial_protocol.hpp
├── command_router.cpp
└── command_router.hpp
```

分层：

```text
UART ISR
  -> RingBuffer<uint8_t, RxN>::push_isr()

main loop
  -> pop bytes
  -> FrameParser<MaxPayload>::feed()
  -> CommandDecoder
  -> CommandRouter
  -> write g_mc_cmd / g_screw_axis_cmd
```

适合使用的 C++ 特性：

```text
RingBuffer<T, Capacity>
static_assert(Capacity is power of two)
enum class ParserState
enum class CommandId
enum class ParseResult
if constexpr 选择 CRC/无 CRC 协议配置
policy class 选择 UART0/UART1 driver
```

推荐 ring buffer 约束：

```text
单生产者：UART RX ISR
单消费者：main loop
ISR 只收字节，不解析协议
主循环解析完整帧
溢出计数进入 watch
不在 ISR 回调业务逻辑
```

命令仲裁：

```text
串口不要直接写 MotorControlCState
串口不要和 ScrewAxis/Ozone 随机抢写 g_motor_cmd
CommandRouter 负责把解码命令转换为当前唯一有效命令
```

第一版建议支持：

```text
SetMotorMode
SetCurrent
SetSpeedRpm
StartHome
StopHome
ReadMotorWatch
ReadScrewHomeWatch
```

不建议第一版支持：

```text
Flash 写参数
复杂日志字符串
变长动态内存 payload
阻塞发送
```

收益：

```text
串口协议从一开始就类型安全
ISR 实时性可控
后续协议扩展不污染 MotorControl 快环
```

## 第四阶段：MotorControl 核心调度层

当前文件：

```text
Firmware/MotorControl/LegacyC/motor_control_c.c
```

先不要动快环，先把核心调度层迁成 C++：

```text
Firmware/MotorControl/Core/core.cpp
```

仍然导出同名 C ABI：

```cpp
extern "C" void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command);
extern "C" void MotorControl_RunSlowLoop(void);
```

适合重写点：

```text
MC_STATE_* -> enum class ControlState
MC_MODE_* -> enum class ControlMode
MC_FAULT_* -> enum class ControlFault
copy_command -> VolatileCommandCopy helper
clamp_ref / abs_limit -> template clamp/abs_limit
NVIC_DisableIRQ/EnableIRQ -> AdcIrqGuard
mode ready/fault decision -> 小的 constexpr/state helpers
```

注意：

```text
关 ADC IRQ 的临界区只能复制 s_mc.command/mode/enabled
不要在 RAII guard 范围内做限幅、协议解析、watch 填充
```

收益：

```text
模式/故障不再是裸 uint8_t
命令限幅逻辑更可读
后续多命令源仲裁更容易接入
```

## 第五阶段：FOC 数学小组件

当前文件：

```text
Firmware/MotorControl/Algorithm/foc_math.c
Firmware/MotorControl/Algorithm/foc_math.h
```

建议不要一次性 C++ 化全部数学。先抽小组件：

```text
FocPi -> FixedPi / FixedPiConfig
slew_s16/slew_s32 -> SlewLimiter<T, Step>
speed filter -> LowPass<Shift>
Angle16 wrapper
```

适合使用：

```text
constexpr
template<int Shift>
template<typename T, T Min, T Max>
static_assert
```

暂时不建议：

```text
用模板重写完整 Park/Clarke/SVPWM
把 sin 表改成复杂 constexpr 生成
引入 std::array 之前没有体积对比
```

收益：

```text
PI/滤波/斜坡复用到速度环、串口参数处理、回零控制
不动核心 FOC 公式，降低风险
```

## 第六阶段：Encoder 与速度估算

当前文件：

```text
Firmware/MotorControl/LegacyC/motor_control_encoder.c
```

适合迁移点：

```text
Angle16
RawAngle
SpeedCounts
EncoderSampleValidator<MaxStepRaw>
SpeedEstimator<SampleHz, FilterShift>
if constexpr 选择不同 raw diff 滤波策略
```

建议保留：

```text
MotorControl_InternalUpdateEncoderAngle()
MotorControl_InternalUpdateEncoderSpeed()
```

这两个 C ABI/内部入口先不改，内部可以转调 C++ 对象。

收益：

```text
角度单位和速度单位清楚
坏角过滤参数可用 static_assert 检查
raw diff 速度估算和滤波参数的编译期检查更干净
```

## 第七阶段：Board 层

当前文件：

```text
Firmware/Board/Src/foc_bsp.c
Firmware/Board/Src/foc_pwm.c
Firmware/Board/Src/foc_curr.c
Firmware/Board/Src/foc_ma600.c
```

不建议第一批迁移 `foc_curr.c`。它当前承载低边采样窗口选择和 ADC 时序，风险最高。

推荐顺序：

```text
1. 新串口 driver
2. MA600 薄 wrapper
3. PWM 配置常量 static_assert
4. 电流采样策略最后动
```

可用特性：

```text
type-safe register wrapper 只用于新增串口或很薄的 helper
policy class 表达 UART 实例
constexpr pin/config
static_assert 检查 PWM/ADC tick 边界
```

禁止：

```text
用大型寄存器 DSL 重写原厂库
把 ADC/PWM 快环路径改成深模板栈
隐藏硬件时序
```

## 不建议迁移的部分

短期不迁：

```text
Firmware/ThirdParty/**
startup_CMS32M6510_gcc.S
cms32m6510_flash.ld
原厂 adc/epwm/gpio/pga/ssp 驱动
```

只在外层包薄 C++ wrapper，不改第三方输入。

## 推荐时间线

```text
Step 1: CMake 增加 cms32_cpp_options 和 Firmware/Support header-only
Step 2: ScrewAxis 改 C++，保持 C ABI
Step 3: 新建 Comm 串口模块，固定 ring buffer + parser
Step 4: MotorControl 核心调度层改 C++，快环仍 C
Step 5: PI/斜坡/滤波小组件模板化
Step 6: Encoder/速度估算内部 C++ 化
Step 7: 最后评估 Current fast loop 是否值得迁移
```

每一步验收：

```text
Debug 构建通过
MinSizeRel 构建通过
Flash/RAM 对比记录
nm 检查没有 new/delete/throw/typeinfo/vtable
Ozone watch 入口不破坏
ADC IRQ 快环没有引入不可控临界区
```

## 当前最推荐先做的文件

```text
Firmware/Support/clamp.hpp
Firmware/Support/units.hpp
Firmware/Support/ring_buffer.hpp
Firmware/App/screw_axis.cpp
Firmware/Comm/serial_protocol.hpp
Firmware/Comm/serial_protocol.cpp
```

理由：

```text
风险低
收益直接
不影响 FOC 快环稳定性
能把教程里的 enum class / constexpr / template / ring buffer 落到实际代码
为后续串口调试准备好结构
```
