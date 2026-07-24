# C++ 分层与嵌入式规范审查

本文用 Awesome Modern C++ 嵌入式教程的思路，回顾当前工程的 C++ 结构是否走在正确方向上。

> 当前最终落地形态见 `25-Cpp迁移-C_Cpp混编最小封装落地指南.md`。本文保留规范审查
> 过程；当前主线已经收敛为“C ABI 外壳 + C++ 类型安全 + `MotorController g_motor`”。

参考入口：

```text
https://awesome-embedded-learning-studio.github.io/Tutorial_AwesomeModernCPP/vol8-domains/embedded/
```

重点参考的主题：

```text
零开销抽象
资源与实时约束
动态内存的代价
静态存储与栈上分配
编译期多态 vs 运行时多态
中断安全
UART SPSC ring buffer
UART driver template
```

这份文档不是要求把当前固件一次性改成“纯 C++”。当前目标是：在不破坏 C ABI、不破坏 Ozone 观察、不破坏 20 kHz 快环确定性的前提下，让 C++ 帮我们收紧类型、单位、配置和模块边界。

## 1. 教程里的核心规则

教程反复强调的不是“多写 class”，而是这几件事：

```text
资源可预算
实时路径可预测
内存生命周期明确
抽象尽量零运行时开销
中断里不阻塞、不分配、不做复杂逻辑
外设选择和策略选择尽量编译期确定
```

换成我们项目的话，就是：

```text
FOC 快环要能估算最坏执行时间。
MotorControl 的状态要能在 Ozone 里直接看。
命令入口可以整理，但不能隐藏实时状态。
C++ 只能减少错误来源，不能增加不可见行为。
```

## 2. 当前工程的 C++ 分层

当前主要结构：

```text
Firmware/MotorControl/
├── Abi/
│   └── MotorControl.h
│   ├── motor_control_internal.h
│   ├── motor_control_state.h
│   └── motor_control_vf.h
├── Core/
│   ├── motor_controller.hpp
    ├── core.cpp
    ├── current.cpp
    ├── encoder.cpp
    ├── output.cpp
│   └── vf.cpp
├── Config/
│   └── config.hpp
├── Math/
    ├── fixed_pi.hpp
    ├── speed_math.hpp
│   └── encoder_math.hpp
├── Types/
│   ├── types.hpp
│   ├── debug_state.hpp
│   └── command_sanitizer.hpp
└── FrozenCpp/
    ├── foc_transform.hpp
    ├── pwm_values.hpp
    └── adc_sample_window.hpp

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

当前分层判断：

```text
Abi/MotorControl.h:
  对外唯一公共控制 API。main/App/Ozone 只应该依赖这个入口。

Abi/motor_control_internal.h:
  内部 C ABI 桥和兼容声明。它不是长期公共 API。

Core/motor_controller.hpp 和 Core/*.cpp:
  真实控制链实现。MotorController 持有状态，导出的 MotorControl_* 函数名继续保持 C ABI。

Types/*.hpp / Config/config.hpp:
  当前 C++ 侧的领域类型和参数镜像。

Math/*.hpp:
  可测试、无状态或薄状态的数学小组件。

FrozenCpp/*.hpp:
  已冻结的教学/候选包装，不接入当前生产路径。

Support/*.hpp:
  与 MotorControl 业务无关的通用工具。
```

这个方向是对的。它符合教程里“用 C++ 做零开销抽象和编译期约束，不把运行时行为藏起来”的要求。

## 3. 公共边界规则

长期规则：

```text
App 层只能 include MotorControl.h。
Board 层不能反向 include MotorControl 业务头。
Support 不能依赖 Board 或 MotorControl。
MotorControl/Cpp 里的头默认是内部头，不是公共 SDK。
```

正确依赖方向：

```text
App
  -> MotorControl public API
    -> MotorControl internal implementation
      -> Algorithm / Support / Board BSP
```

错误方向：

```text
Support -> MotorControl
Board   -> MotorControl command/state
App     -> MotorControl/Cpp/current.cpp 内部 helper
App     -> MotorControl/Cpp/encoder_math.hpp 直接改控制行为
```

判断一个 `.hpp` 是否应该公开，只问一句：

```text
这个类型是否是模块边界的一部分？
```

如果不是，就不要让外部 include。

## 4. `.cpp` 和 `.hpp` 怎么分

C 里常见的问题是：

```text
每个 .c 都甩一个 .h
每个 .h 又暴露一堆内部函数
最后所有模块都能互相调用
```

C++ 不应该重复这个问题。

### 4.1 应该放 `.cpp` 的内容

这些内容优先留在 `.cpp` 的匿名 namespace：

```text
只在一个文件里用的 helper
状态机内部判断
故障进入安全态的局部步骤
某个模块私有的转换函数
临时诊断逻辑
```

示例：

```cpp
namespace
{

bool ready_for_mode(ControlMode mode) noexcept
{
    ...
}

void enter_fault_state(ControlFault fault) noexcept
{
    ...
}

} // namespace
```

这样做的原因：

```text
链接符号不外泄
外部不能误调用
读代码时边界清楚
编译器仍然可以内联优化
```

### 4.2 应该放 `.hpp` 的内容

这些内容适合放 `.hpp`：

```text
enum class
小 value object
template
constexpr 纯函数
static constexpr config
static_assert 编译期检查
跨多个 .cpp 使用的轻量工具
```

示例：

```cpp
struct SpeedLoopConfig
{
    static constexpr int32_t estimate_hz = CTRL_SPD_EST_HZ;
    static constexpr int16_t kp = CTRL_SPD_KP;
    static constexpr int16_t ki = CTRL_SPD_KI;
};

static_assert(SpeedLoopConfig::estimate_hz > 0, "invalid speed estimate frequency");
```

这样做的原因：

```text
编译期可见
模板需要定义在头文件
constexpr 可被调用点折叠
类型和单位能跨模块统一
```

### 4.3 有 `.cpp` 不代表必须有 `.hpp`

例如：

```text
core.cpp
current.cpp
encoder.cpp
```

它们现在不需要对应的：

```text
core.hpp
current.hpp
encoder.hpp
```

因为外部不应该直接调用这些实现文件里的内部 helper。对外入口已经由 `MotorControl.h` 和内部 C ABI 负责。

## 5. 零开销抽象规则

教程里的零开销抽象，可以落成项目规则：

```text
优先使用 enum class 表达有限状态。
优先使用 constexpr 做参数和单位换算。
优先使用 template 表达编译期确定的容量、引脚、外设、比例。
优先使用 RAII 管非常短的资源区间。
避免 runtime polymorphism 进入快环。
```

当前符合的例子：

```text
types.hpp:
  ControlMode / ControlState / ControlFault 用 enum class。

config.hpp:
  把 BoardConfig.h / TuneConfig.h 的参数镜像成 static constexpr。

speed_math.hpp / encoder_math.hpp:
  把 rpm、speed count/s、raw delta 的公式收成可检查的 constexpr 函数。

irq_guard.hpp:
  用 RAII 管 ADC IRQ 的短临界区。

ring_buffer.hpp:
  用固定容量模板实现 SPSC ring buffer。
```

注意：零开销不是“写了 C++ 就自动零开销”。它依赖：

```text
不开异常
不开 RTTI
不使用虚函数热路径
不使用动态分配
编译器优化开启
链接后检查符号和体积
```

## 6. 禁用特性清单

当前控制路径禁止：

```text
new / delete
malloc / free
throw / try / catch
RTTI / typeinfo
virtual 快环分发
std::function
std::vector
std::string
iostream
std::mutex
condition_variable
递归
大局部数组
```

原因：

```text
动态内存会带来碎片和时序不确定。
异常会引入不可控展开路径和 runtime 支持。
virtual 会引入 vtable/vptr 和间接跳转。
std::function 可能隐藏分配或类型擦除成本。
mutex/condition_variable 不适合 bare-metal ISR。
递归和大局部数组会让栈使用不可预测。
```

允许但要谨慎：

```text
std::type_traits
std::underlying_type
简单 constexpr helper
固定容量模板容器
小型聚合结构体
函数指针表
```

函数指针表只能用于低频路径，例如串口命令路由；不要放进 FOC 快环内层或安全停机路径。

## 7. 中断安全规则

教程对 ISR 的规则很明确：

```text
ISR 不能阻塞。
ISR 不能动态分配。
ISR 不能抛异常。
ISR 不能做长时间计算。
ISR 和主循环共享数据必须有明确同步策略。
```

当前工程对应规则：

```text
ADC_IRQHandler:
  只跑必须的采样和快环，不做串口解析，不做文档里那种 command dispatch。

UART IRQ:
  后续只收字节、放 ring buffer、计数溢出，不直接改 MotorControl 内部状态。

main slow loop:
  消费命令、sanitize、更新 watch、做低频状态机。
```

SPSC ring buffer 适用条件：

```text
单生产者
单消费者
生产者只推进 head
消费者只推进 tail
固定容量
满了可丢弃并计数
```

当前 `Firmware/Support/ring_buffer.hpp` 就是这个方向。后续 UART parser 应该基于它，而不是在 ISR 里解析完整协议。

## 8. 内存分配规则

当前项目应该采用这个优先级：

```text
小的临时对象:
  放栈上。

长期存在的控制状态:
  放静态区或 MotorControlCState。

DMA / UART / 采样缓冲:
  固定容量静态缓冲。

动态生命周期对象:
  当前阶段不做。

必须动态生命周期时:
  先设计固定对象池，再讨论 placement new。
```

当前 `MotorControlCState` 继续作为正式状态源是合理的。它的好处是：

```text
内存布局固定
Ozone 可直接观察
重置路径明确
不会出现隐藏 C++ 对象状态
```

这也是为什么 `encoder.cpp` 第一版没有直接把 `AngleValidator` / `SpeedEstimator` 的私有状态接进生产路径。当前先用无状态数学函数，避免出现两份状态。

## 9. 编译期多态与运行时多态

教程里讲 CRTP / template / if constexpr 的重点是：当硬件选择或策略选择在编译期已知时，不要用运行时多态。

适合编译期确定的内容：

```text
PWM period
ADC trigger tick 默认值
encoder counts per rev
ring buffer capacity
UART 实例
GPIO pin policy
固定滤波 shift
固定 PI shift
```

适合运行时保留的内容：

```text
g_motor_cmd.enable
g_motor_cmd.control_mode
speed_ref / current_ref
Ozone 调试参数
fault/state
运行时命令值
```

不要把运行时命令硬塞进 template。比如：

```text
ControlMode 是运行时命令，不要做成 template 参数。
UART0 是固定硬件实例，可以做成 template 参数。
CTRL_SPD_FILTER_SHIFT 是编译期配置，可以做成 template 参数。
```

## 10. 当前审查结果

### 10.1 已经符合的地方

```text
CMake 已关闭 exceptions / RTTI / threadsafe statics / cxa_atexit。
MotorControl 对外仍是 C ABI。
核心 C++ 文件没有引入 heap、异常、virtual、std 容器。
Support 组件基本都是固定容量、constexpr、template、RAII。
命令 sanitize 在慢环做，不在 ADC 快环里做复杂解析。
encoder.cpp 保留 MotorControlCState 作为正式状态源。
```

验证命令：

```sh
./testhpp.sh
cmake --build --preset gcc-debug --target cms32foc
arm-none-eabi-nm -C build/gcc-debug/cms32foc \
  | rg "operator new|operator delete|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
```

期望：

```text
testhpp.sh 通过
固件构建通过
nm 禁用符号扫描无输出
```

### 10.2 需要继续收口的地方

第一，`Firmware/MotorControl/Core` 当前作为 include 目录公开得偏宽。

当前风险：

```text
外部 App 理论上可以直接 include MotorControl/Cpp 内部头。
这样会破坏 MotorControl.h 作为唯一公共入口的边界。
```

后续目标：

```text
MotorControl/Inc 作为 PUBLIC。
MotorControl/C 和 MotorControl/Cpp 尽量 PRIVATE。
Support 只给需要它的 C++ target。
```

第二，`Firmware/MotorControl/Core` 里混有生产头和教学/待接入头。

建议分类：

```text
生产共享:
  types.hpp
  config.hpp
  command_sanitizer.hpp
  fixed_pi.hpp
  speed_math.hpp
  encoder_math.hpp

生产实现:
  core.cpp
  current.cpp
  encoder.cpp

已冻结教学/候选:
  foc_transform.hpp
  pwm_values.hpp
  adc_sample_window.hpp
```

这些文件已经移到 `Firmware/MotorControl/FrozenCpp/`，不再接入当前生产 include 路径。

第三，C++ config 当前只是宏镜像。

当前是对的，因为 C 文件还没有完全退场。但长期目标是：

```text
硬件基线:
  BoardConfig.h 继续作为硬件事实来源。

控制策略:
  逐步收敛到 C++ config 或明确参数结构。

C ABI:
  只保留外部需要观察和写入的命令/状态结构。
```

第四，少量临时痕迹要清理。

例如：

```text
重复 include
学习期注释
旧草稿文件名
文档里“当前状态”和“教学示例”混在一起
```

这些不影响当前行为，但会影响长期可读性。

## 11. 后续迁移顺序建议

不要按“哪个文件还没 C++ 化”来迁移。应该按风险和收益排序：

```text
1. 固定 C++ 分层规则和 include 可见性。
2. 清理 MotorControl/Cpp 中生产文件和教学文件的状态标记。
3. 继续把数学小组件接入真实路径，但不隐藏 MotorControlCState。
4. UART 只先做 SPSC ring buffer + 主循环 parser，不做 ISR parser。
5. Board wrapper 只做薄 value object，不类化寄存器副作用。
6. Output/VF/Watch 这种边界清楚的 C 文件，再逐步迁到 C++。
7. 最后才考虑 Board 底层和 vendor 边界。
```

最重要的原则：

```text
先收边界，再加抽象。
先保确定性，再谈优雅。
先保可观察，再谈封装。
```

## 12. 每次 C++ 改动后的验收清单

每次接入新的 C++ 文件，都检查：

```text
是否保持 MotorControl.h C ABI 不变
是否新增动态分配
是否新增异常/RTTI/virtual
是否新增 std::function/std::vector/std::string/iostream
是否让 ISR 执行路径变长或不可预测
是否让 Ozone 关键状态不可见
是否让 App 直接 include 内部 C++ 头
是否能解释每个 template 参数为什么必须是编译期
是否能解释每个结构体字段的单位
```

命令：

```sh
./testhpp.sh
cmake --build --preset gcc-debug --target cms32foc
arm-none-eabi-nm -C build/gcc-debug/cms32foc \
  | rg "operator new|operator delete|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
```

如果 `nm` 有输出，不要继续往后迁移，先解释清楚符号来自哪里。

## 13. 当前一句话结论

当前工程的 C++ 路线符合教程方向：

```text
C ABI 保外壳
C++ 收类型和单位
constexpr/template 做编译期计算
RAII 只管短资源区间
ISR 和快环保持确定性
不引入 heap/异常/RTTI/virtual
```

还需要改进的是分层边界：

```text
不要让 MotorControl/Cpp 变成新的公共 include 垃圾桶。
不要每写一个 .cpp 就甩一个 .hpp。
不要让教学组件和生产组件长期混在一起不标状态。
```

真正好的嵌入式 C++，不是“看起来很 C++”，而是编译后仍然像手写 C 一样直接，同时源码里更难写错单位、状态和边界。
