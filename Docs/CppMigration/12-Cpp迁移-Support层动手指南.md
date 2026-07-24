# Support 层 C++ 动手指南

本文不是 API 手册，而是给你自己动手写 `Firmware/Support/*.hpp` 的练习路线。目标是理解这些 C++ 组件为什么是零开销、为什么适合嵌入式，以及怎么验证它们没有偷偷引入堆、异常或奇怪运行时。

> 当前最终落地形态见 `25-Cpp迁移-C_Cpp混编最小封装落地指南.md`。本文中的旧状态源
> 名称只代表当时教学阶段；当前正式观察入口是 `cms32::motor::g_motor`。

当前 Support 目录建议包含：

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

你可以按本文顺序一个一个重写。每写完一个，就用文末的 `arm-none-eabi-g++` 命令单独编译检查。

## 总规则

这些文件都应该满足：

```text
header-only
不使用 new/delete
不使用 malloc/free
不使用异常
不使用 RTTI
不使用 std::vector/std::string/std::function/iostream
不定义复杂全局对象
不访问 MotorControl 内部状态
不隐藏耗时操作
```

你写的时候心里只记一句话：

```text
模板只表达编译期约束；运行时仍然是普通整数和普通结构体。
```

## 先把 C++ 特性想明白

这一节先不写具体文件，先讲你在 Support 里会频繁看到的 C++ 特性。你不用一次全背下来，但每写一个 `.hpp`，都可以回来看对应小节。

### header-only 是什么意思

Support 里的大部分组件都写在 `.hpp` 里，不写 `.cpp`，这叫 header-only。

原因是：

```text
模板函数/模板类通常需要把完整定义放在头文件里
编译器看到完整定义后才能为 int16_t、int32_t、uint8_t 等类型生成对应代码
函数很小，放头文件更容易内联
不需要额外链接一个库对象
```

比如：

```cpp
template <typename T>
constexpr T clamp(T value, T min_value, T max_value) noexcept
{
    return (value < min_value) ? min_value
                               : ((value > max_value) ? max_value : value);
}
```

当你写：

```cpp
int16_t a = clamp<int16_t>(x, -10, 10);
int32_t b = clamp<int32_t>(y, -1000, 1000);
```

编译器可以理解成帮你生成了两份普通函数：

```text
一份处理 int16_t
一份处理 int32_t
```

这不是运行时“动态选择类型”。类型选择发生在编译期。

### #pragma once 为什么每个头文件都有

头文件可能被多个 `.cpp` include，也可能 A include B，B 又 include C。`#pragma once` 的作用是：

```text
同一个编译单元里，这个头文件只展开一次
避免重复定义 struct/class/template
避免 include 链复杂后出现奇怪重复报错
```

它不产生运行时代码，只影响编译器读文件。

### namespace 为什么要写 cms32::support

C 项目里函数名经常靠前缀防冲突：

```c
motor_control_init()
screw_axis_init()
```

C++ 可以用 namespace：

```cpp
namespace cms32::support {

constexpr int clamp_value = 1;

} // namespace cms32::support
```

作用是：

```text
把 Support 组件放进 cms32::support 这个名字空间
避免和第三方库、原厂库、后续 Comm/Motor 模块重名
读代码时一眼知道这个工具属于哪一层
```

使用时可以写完整名字：

```cpp
cms32::support::RingBuffer<uint8_t, 128> rx;
```

在小 `.cpp` 内部也可以局部写：

```cpp
using cms32::support::clamp;
```

不要在公共头文件里写 `using namespace cms32::support;`，否则 include 这个头的人会被迫污染命名空间。

### template <typename T> 是类型参数

`template <typename T>` 的意思是：`T` 是一个编译期类型占位符。

```cpp
template <typename T>
constexpr T abs_limit(T value, T limit) noexcept;
```

你调用：

```cpp
abs_limit<int16_t>(iq, 16);
abs_limit<int32_t>(speed, 5000);
```

编译器会分别检查：

```text
T = int16_t 时，value/limit/return 都是 int16_t
T = int32_t 时，value/limit/return 都是 int32_t
```

这比宏安全：

```c
#define CLAMP(x, lo, hi) ...
```

宏只是文本替换，不知道类型，也不会给你类型错误提示。模板是“编译器理解的泛型”。

### template <int32_t CountsPerRev> 是值参数

模板不只能传类型，也能传编译期常量值：

```cpp
template <int32_t CountsPerRev>
constexpr SpeedCounts to_speed(Rpm rpm) noexcept
{
    static_assert(CountsPerRev > 0, "CountsPerRev must be positive");
    return SpeedCounts{(rpm.value * CountsPerRev) / 60L};
}
```

调用：

```cpp
auto speed = to_speed<262144>(Rpm{60});
```

这里 `262144` 是模板值参数。它的好处是：

```text
编译期检查 CountsPerRev > 0
编译器知道这个常量，可以做常量折叠
函数调用点能看出换算基准
不需要在对象里保存 CountsPerRev
```

适合模板值参数的东西：

```text
RingBuffer 容量
LowPass Shift
SlewLimiter Step
编码器一圈 count 数
协议最大 payload 长度
```

不适合模板值参数的东西：

```text
Ozone 运行时会改的参数
串口命令传进来的速度
调试时频繁改的 PI 参数
```

一句话：编译期确定的，用模板值参数；运行时要改的，放变量。

### constexpr 不等于一定在编译期运行

`constexpr` 的意思是：这个函数“允许”在编译期计算。

```cpp
constexpr uint16_t add(uint16_t a, int16_t d) noexcept
{
    return static_cast<uint16_t>(a + d);
}
```

如果输入是常量：

```cpp
static_assert(add_angle(10, 2) == 12);
```

编译器必须在编译期算出来。

如果输入是运行时变量：

```cpp
angle = add_angle(angle, delta);
```

它就是普通加法。`constexpr` 不会让代码变慢，也不会自动产生表格或奇怪运行时对象。

在 Support 里，`constexpr` 的价值是：

```text
小函数可以被编译期测试
常量输入时可以被折叠
告诉读代码的人：这个函数没有依赖全局状态
```

### noexcept 是对嵌入式的承诺

`noexcept` 表示这个函数不抛异常：

```cpp
bool push_isr(const T& value) noexcept;
```

我们本来就禁用异常：

```text
-fno-exceptions
```

但 `noexcept` 仍然有价值：

```text
写接口时明确告诉别人这里不会抛异常
编译器可以省掉异常路径考虑
ISR/快环代码读起来更安心
```

Support 里的小函数、小类成员，默认都应该 `noexcept`。

### static_assert 是编译期保险丝

`static_assert` 是编译期检查：

```cpp
static_assert(Capacity >= 2U, "Capacity must leave one empty slot");
```

如果你写：

```cpp
RingBuffer<uint8_t, 3> rx;
```

编译直接失败，而不是上板后发现 ring buffer 回绕错。

适合 `static_assert` 的检查：

```text
RingBuffer 容量必须是 2 的幂
LowPass Shift 不能大到移位未定义
SlewLimiter Step 必须大于 0
MOT_SENSOR_DIR 只能是 1 或 -1
PWM_DUTY_MIN < PWM_DUTY_50 < PWM_DUTY_MAX
```

不适合 `static_assert` 的检查：

```text
串口收到的 payload 长度
Ozone 写进来的速度命令
运行中电流是否过大
```

运行时才知道的东西，还是要用普通 if 检查。

### type_traits 是给模板用的编译期工具

你会看到：

```cpp
#include <type_traits>

static_assert(std::is_signed<T>::value, "requires signed type");
```

`std::is_signed<T>::value` 是编译期布尔值。它能判断 `T` 是不是有符号整数。

为什么需要它？

```cpp
template <typename T>
constexpr T abs_limit(T value, T limit) noexcept
{
    return clamp<T>(value, -limit, limit);
}
```

如果 `T = uint16_t`，`-limit` 就会变成无符号回绕，结果非常危险。所以我们写：

```cpp
static_assert(std::is_signed<T>::value, "abs_limit requires signed type");
```

这样误用会在编译期爆出来。

### enum class 比普通 enum 安全在哪里

C 里常写：

```c
#define STATE_IDLE 0U
#define STATE_FAULT 4U
uint8_t state;
```

问题是 `state` 可以随便被写成任何 `uint8_t`。

C++ 可以写：

```cpp
enum class HomeState : uint8_t {
    Idle = 0U,
    Fault = 4U,
};
```

好处：

```text
HomeState 不会自动和 uint8_t 混用
HomeState 不会和 MotorState 混用
switch(state) 时枚举值更清楚
底层类型仍然是 uint8_t，不浪费 RAM
```

需要给 Ozone watch 写数字时，再显式转出去：

```cpp
watch.state = to_underlying(HomeState::Fault);
```

这就是“内部类型安全，对外仍保持 C ABI”。

### struct wrapper 为什么不写 getter/setter

Support 里会写：

```cpp
struct Rpm {
    int16_t value;
};
```

故意不写成：

```cpp
class Rpm {
public:
    int16_t get() const;
    void set(int16_t);
private:
    int16_t value_;
};
```

原因是当前项目更需要“单位类型安全”，不是复杂封装。

`struct Rpm { int16_t value; };` 的好处：

```text
运行时就是一个 int16_t
可以 Rpm{60} 简单初始化
调试器里容易看
不会制造一堆没意义 getter/setter
函数签名能防止单位混传
```

比如：

```cpp
void set_speed(Rpm rpm);
void set_iq(CurrentCount iq);
```

`set_speed(CurrentCount{12})` 会编译失败。这就是它的主要价值。

### class 不是一定代表重封装

Support 里也会有 `class`：

```cpp
template <uint8_t Shift>
class LowPassI32 {
public:
    int32_t update(int32_t sample) noexcept;

private:
    int32_t value_{0};
};
```

这里用 `class` 是因为它有内部状态 `value_`。我们希望外部只能通过 `update()` 改它，避免随手把滤波器状态写坏。

这个类运行时是什么？

```text
一个 int32_t 成员
update() 里一个减法、一个移位、一个加法
没有堆
没有虚函数
没有隐藏线程
```

所以不要看到 `class` 就害怕。关键是看它有没有：

```text
virtual
new/delete
复杂构造函数
全局副作用
不可控耗时
```

没有这些，它通常就是“带函数的结构体”。

### private 是为了保护状态，不是为了仪式感

比如 ring buffer：

```cpp
private:
    T data_[Capacity]{};
    volatile size_t head_{0U};
    volatile size_t tail_{0U};
```

如果 `head_` 和 `tail_` 公开，外面任何代码都能乱改：

```cpp
rx.head_ = 123; // 直接毁掉 buffer
```

所以它们放 private。外面只能：

```cpp
rx.push_isr(byte);
rx.pop(byte);
```

private 的判断标准：

```text
这是对象内部一致性的一部分 -> private
这是给别人传参/观察的纯数据 -> struct public
```

这能避免“为了封装而封装”，也能避免“什么都 public 导致状态乱飞”。

### = delete 不是释放内存

文档里会看到这种写法：

```cpp
RingBuffer(const RingBuffer&) = delete;
RingBuffer& operator=(const RingBuffer&) = delete;
```

这里的 `delete` 不是：

```cpp
delete ptr;
delete[] buffer;
```

它不是释放堆内存，也不会调用 `operator delete`。`= delete` 的意思是：

```text
把这个函数入口删掉
谁调用它，谁编译失败
```

上面两行分别表示：

```text
禁止拷贝构造 RingBuffer
禁止拷贝赋值 RingBuffer
```

为什么要禁止？

```cpp
RingBuffer<uint8_t, 128> rx1;
RingBuffer<uint8_t, 128> rx2 = rx1; // 不希望允许
```

`RingBuffer` 内部有固定数组、`head_`、`tail_` 和溢出计数。复制它通常不是你真正想要的操作：你可能以为复制了同一个串口队列，实际得到两个状态相似但互相独立的队列，ISR 和主循环很容易用乱。

所以要这样记：

```text
delete ptr;       禁止，释放动态内存
func() = delete;  允许，编译期禁用某个函数
```

我们说“嵌入式不使用 delete”，指的是不使用 `delete ptr` 这种动态内存释放；`= delete` 是类型安全工具，适合用。

### RAII 是把成对操作绑到生命周期

RAII 听起来很大，其实在我们项目里先只用一种：临界区 guard。

C 写法容易漏：

```c
NVIC_DisableIRQ(ADC_IRQn);
if (bad) {
    return; // 忘记 EnableIRQ
}
NVIC_EnableIRQ(ADC_IRQn);
```

C++ 写法：

```cpp
{
    AdcIrqGuard guard;
    command = next_command;
} // 离开作用域，析构函数自动 EnableIRQ
```

RAII 的核心是：

```text
构造函数做开始动作
析构函数做结束动作
对象活多久，资源就占用多久
```

它适合：

```text
短临界区
临时关中断
临时锁一个硬件资源
```

不适合：

```text
长时间持有
复杂业务流程
FOC 快环里绕来绕去的隐式操作
```

### volatile 在 ring buffer 里能解决什么，不能解决什么

当前 ring buffer 是给这种模型用的：

```text
UART RX ISR 写 head/data
main loop 读 tail/data
单生产者
单消费者
单核 MCU
```

`volatile size_t head_` 的意思是：每次读写 `head_` 都真的访问内存，不要被编译器缓存成寄存器里的旧值。

它能帮我们处理：

```text
ISR 和 main loop 都会访问 head/tail
编译器不能假设 head/tail 没变
```

它不能保证：

```text
多个生产者同时写
多个消费者同时读
复杂多核内存顺序
任意类型 T 的写入都是原子的
```

所以文档里一直强调：

```text
RingBuffer 只用于单 ISR 写、单主循环读
不要多个中断一起 push
不要多个任务一起 pop
```

### 为什么不用 std::array / std::optional / std::span

这些东西本身不一定错，但第一阶段 Support 先不用，原因是：

```text
你现在更需要理解 C++ 基础机制
项目还在建立体积和符号检查习惯
裸数组 data_[Capacity] 对嵌入式调试器最直观
C++ 标准库在交叉工具链上的实现差异需要确认
```

后续可以评估：

```text
std::array<T, N> 通常是零开销，可以考虑
std::span 是视图，C++20 才标准化，当前 C++17 不用
std::optional 可能可用，但第一版协议解析先用 bool/enum 更直观
```

先把项目基础打稳，比一开始把现代 C++ 全部堆上去更重要。

## Support 设计总思路

Support 层不要变成“万能工具箱”。它只放三类东西：

```text
1. 无业务含义的小算法：clamp、slew、low_pass
2. 类型安全的小类型：Rpm、CurrentCount、Angle16
3. 嵌入式边界工具：RingBuffer、IrqGuard、static_asserts
```

不应该放：

```text
MotorControl 状态机
ScrewAxis 回零逻辑
串口协议命令含义
板级初始化流程
Ozone watch 业务字段
```

你每写一个 Support 组件，都问四个问题：

```text
它能不能一句话说明运行时代价？
它有没有偷偷访问全局变量？
它是不是不依赖业务模块？
它能不能通过 static_assert 或类型系统防一个真实错误？
```

四个都答得清楚，就比较适合放 Support。

## 第 1 步：clamp.hpp

先写最简单的限幅函数。

这个组件的设计理由：

```text
限幅在电机控制里到处都有，不能每个文件都手写 if
用模板是为了支持 int16_t/int32_t/uint16_t，不是为了炫技
用 constexpr 是为了让常量限幅能被编译期计算
用 static_assert 是为了把错误范围在编译期拦住
```

你想替代的是这种 C 代码：

```c
if (value < min_value) {
    value = min_value;
}
if (value > max_value) {
    value = max_value;
}
```

C++ 写法：

```cpp
template <typename T>
constexpr T clamp(T value, T min_value, T max_value) noexcept
{
    return (value < min_value) ? min_value
                               : ((value > max_value) ? max_value : value);
}
```

你要理解：

```text
template <typename T> 让 int16_t/int32_t/uint16_t 都能用
constexpr 允许常量输入时编译期计算
noexcept 明确不抛异常
返回值仍然是普通 T，没有额外对象
```

再加一个编译期范围版本：

```cpp
template <typename T, T MinValue, T MaxValue>
constexpr T clamp_static(T value) noexcept
{
    static_assert(MinValue <= MaxValue, "invalid clamp range");
    return clamp<T>(value, MinValue, MaxValue);
}
```

这个版本适合：

```cpp
auto iq = clamp_static<int16_t, -16, 16>(cmd_iq);
```

练习：

```text
1. 先只写 clamp()
2. 编译通过后再写 clamp_static()
3. 故意把 MinValue 写大于 MaxValue，看 static_assert 报错
4. 再写 abs_limit() 和 clamp_symmetric()
```

## 第 2 步：units.hpp

这一步是为了避免裸整数乱传。

这个组件的设计理由：

```text
电机控制里很多量底层都是 int16_t/int32_t
裸整数无法表达单位，Rpm 和 CurrentCount 很容易被传反
单位 wrapper 只包一个 value，不做复杂封装
目标是让函数签名自己说明单位
```

现在项目里这些量都可能是 `int16_t`：

```text
speed_rpm
iq_limit
vf_voltage
current count
```

裸整数的问题是：编译器不知道你把 `iq_limit` 传给了速度函数。

先写这种简单 wrapper：

```cpp
struct Rpm {
    int16_t value;
};

struct CurrentCount {
    int16_t value;
};

struct SpeedCounts {
    int32_t value;
};
```

它们没有构造函数、没有虚函数、没有私有封装，就是一个字段。这样做的好处是类型不同：

```cpp
void set_speed(Rpm rpm);
void set_iq_limit(CurrentCount iq);
```

下面这种误用会直接编译不过：

```cpp
CurrentCount iq{12};
set_speed(iq);
```

再写单位换算：

```cpp
template <int32_t CountsPerRev>
constexpr SpeedCounts to_speed(Rpm rpm) noexcept
{
    static_assert(CountsPerRev > 0, "CountsPerRev must be positive");
    return SpeedCounts{(static_cast<int32_t>(rpm.value) * CountsPerRev) / 60L};
}
```

你要理解：

```text
CountsPerRev 是模板参数，编译期固定
Rpm 是输入单位
SpeedCounts 是输出单位
函数内部仍然只是 int32_t 乘除
```

练习：

```text
1. 写 Rpm、CurrentCount、VoltageCount、SpeedCounts
2. 写 to_speed()
3. 写 to_rpm()
4. 加 static_assert 测试 60 rpm 一圈每秒
```

## 第 3 步：enum_utils.hpp

`enum class` 类型安全，但 Ozone watch 和 C ABI 常常需要整数。

这个组件的设计理由：

```text
内部状态机应该用 enum class 保证类型安全
外部 watch/C ABI 仍然需要 uint8_t 数字
to_underlying() 把“枚举转整数”集中到一个显式出口
代码里少写裸 static_cast，状态转换更好审查
```

先写：

```cpp
template <typename Enum>
constexpr auto to_underlying(Enum value) noexcept
    -> typename std::underlying_type<Enum>::type
{
    static_assert(std::is_enum<Enum>::value, "to_underlying requires enum");
    return static_cast<typename std::underlying_type<Enum>::type>(value);
}
```

使用：

```cpp
enum class HomeState : uint8_t {
    Idle = 0U,
    FastRetract = 1U,
};

watch.state = to_underlying(HomeState::FastRetract);
```

你要理解：

```text
enum class 防止状态和普通 uint8_t 混用
to_underlying 是唯一允许出界到整数的地方
```

## 第 4 步：slew_limiter.hpp

斜率限制本质是：

```text
当前值每次最多向目标值走 Step
```

这个组件的设计理由：

```text
速度给定、电流给定都需要斜坡，避免一步跳变
无状态 slew_step() 方便直接替换 C 函数
有状态 SlewLimiter 保存当前值，适合目标值平滑
Step 用模板参数，是因为很多斜坡步长在编译期固定
```

先写无状态函数：

```cpp
template <typename T>
constexpr T slew_step(T current, T target, T step) noexcept
{
    if (step <= static_cast<T>(0)) {
        return target;
    }

    const T delta = static_cast<T>(target - current);
    if (delta > step) {
        return static_cast<T>(current + step);
    }
    if (delta < static_cast<T>(-step)) {
        return static_cast<T>(current - step);
    }
    return target;
}
```

再写有状态版本：

```cpp
template <typename T, T Step>
class SlewLimiter {
public:
    static_assert(Step > static_cast<T>(0), "Step must be positive");

    constexpr T update(T target) noexcept
    {
        value_ = slew_step<T>(value_, target, Step);
        return value_;
    }

private:
    T value_{0};
};
```

这个类没有动态内存，只有一个 `value_`。

当前已经用于替代：

```text
current.cpp 里的本地 slew_s16/slew_s32
速度目标斜坡
iq 输出斜坡
```

为什么用无状态 `slew_step()`，而不是马上用有状态 `SlewLimiter`：

```text
当前正式状态仍保存在 MotorControlCState 里，例如 id_ref_active、iq_ref_active、speed_ref_active。
如果再让 SlewLimiter 自己保存一份 value_，状态会重复，Ozone 也不容易看清。
所以现阶段用无状态函数复用算法，状态仍放在原来的 watch/state 结构里。
```

## 第 5 步：low_pass.hpp

当前速度反馈里有这种滤波：

```c
speed_fb += (sample - speed_fb) >> SHIFT;
```

这个组件的设计理由：

```text
一阶低通在速度估算和调试信号里很常见
Shift 是编译期参数，滤波强度固定且没有运行时配置成本
if constexpr 可以让 Shift==0 的版本完全没有滤波分支
类里只保存一个 value_，运行时代价很好估
```

C++ 可以写成：

```cpp
template <uint8_t Shift>
class LowPassI32 {
public:
    constexpr int32_t update(int32_t sample) noexcept
    {
        if constexpr (Shift == 0U) {
            value_ = sample;
        } else {
            value_ += (sample - value_) >> Shift;
        }
        return value_;
    }

private:
    int32_t value_{0};
};
```

你要理解：

```text
Shift 是编译期参数
if constexpr 会在编译期删除不用的分支
Shift != 0 时就是一个减法、移位、加法
```

练习：

```text
1. 写 LowPassI32<2>
2. 连续喂 100，观察 value 怎么逼近
3. 写 LowPassI32<0>，确认它直接等于 sample
```

## 第 6 步：ring_buffer.hpp

这是给串口准备的。

这个组件的设计理由：

```text
串口 ISR 不能阻塞，也不应该解析协议
固定容量 ring buffer 可以把 ISR 收字节和主循环解析解耦
容量用模板参数，编译期检查 2 的幂
保留一个空槽，让 empty/full 判断简单可靠
```

场景：

```text
UART RX ISR 只负责收字节
主循环负责解析协议
ISR 是单生产者
主循环是单消费者
```

容量建议强制 2 的幂：

```cpp
constexpr bool is_power_of_two(size_t value) noexcept
{
    return (value != 0U) && ((value & (value - 1U)) == 0U);
}
```

核心成员：

```cpp
T data_[Capacity]{};
volatile size_t head_{0U};
volatile size_t tail_{0U};
volatile uint32_t overflow_count_{0U};
```

同时建议禁止复制：

```cpp
RingBuffer(const RingBuffer&) = delete;
RingBuffer& operator=(const RingBuffer&) = delete;
```

这不是动态内存的 `delete`。它只是告诉编译器：这个 ring buffer 不能被复制。队列状态只能有一个所有者，否则 ISR 写一个、副本读另一个，会出现非常隐蔽的问题。

为什么保留一个空槽：

```text
head == tail 表示空
wrap(head + 1) == tail 表示满
```

最小 push：

```cpp
bool push_isr(const T& value) noexcept
{
    const size_t next = wrap(head_ + 1U);
    if (next == tail_) {
        overflow_count_++;
        return false;
    }
    data_[head_] = value;
    head_ = next;
    return true;
}
```

最小 pop：

```cpp
bool pop(T& out) noexcept
{
    if (tail_ == head_) {
        return false;
    }
    out = data_[tail_];
    tail_ = wrap(tail_ + 1U);
    return true;
}
```

你要理解：

```text
这个 ring buffer 不是多线程通用容器
它只适合单 ISR 写、单主循环读
不要多个中断一起写
不要多个任务一起读
```

串口第一版就按这个用：

```cpp
RingBuffer<uint8_t, 128> rx;

extern "C" void UART_IRQHandler(void)
{
    while (uart_rx_ready()) {
        (void)rx.push_isr(uart_read_byte());
    }
}

void Serial_Run()
{
    uint8_t byte;
    while (rx.pop(byte)) {
        parser.feed(byte);
    }
}
```

## 第 7 步：irq_guard.hpp

这个是 RAII 的最小用法。

这个组件的设计理由：

```text
关中断/开中断必须成对出现
C 里遇到 return/break 很容易漏开中断
RAII 用对象生命周期保证离开作用域自动恢复
只适合很短的临界区，不适合包住复杂业务逻辑
```

目的不是炫技，是避免这样的问题：

```c
NVIC_DisableIRQ(ADC_IRQn);
if (bad) {
    return; // 忘记 EnableIRQ
}
NVIC_EnableIRQ(ADC_IRQn);
```

C++ 写法：

```cpp
template <IRQn_Type Irq>
class NvicIrqGuard {
public:
    NvicIrqGuard() noexcept
    {
        NVIC_DisableIRQ(Irq);
    }

    ~NvicIrqGuard() noexcept
    {
        NVIC_EnableIRQ(Irq);
    }

    NvicIrqGuard(const NvicIrqGuard&) = delete;
    NvicIrqGuard& operator=(const NvicIrqGuard&) = delete;
};
```

使用：

```cpp
{
    AdcIrqGuard guard;
    s_mc.command = next_command;
}
```

注意：

```text
guard 生命周期越短越好
里面只放少量赋值
不要在 guard 内解析串口
不要在 guard 内填 watch
不要在 guard 内做 FOC 计算
```

## 第 8 步：static_asserts.hpp

这里放工程配置的编译期检查。

这个组件的设计理由：

```text
配置错误越早发现越好，最好在编译期失败
PWM/传感器/控制周期这类参数错了，上板后很难排查
static_assert 不产生运行时代码
它是工程级“保险丝”，不是运行时保护逻辑
```

适合检查：

```text
PWM_FREQ_HZ > 0
CTRL_FAST_LOOP_DIV > 0
PWM_DUTY_MIN < PWM_DUTY_50 < PWM_DUTY_MAX
MOT_SENSOR_DIR 只能是 1 或 -1
MotorControlCommand_t 不要突然变很大
RingBuffer 容量必须是 2 的幂
```

例子：

```cpp
static_assert(PWM_FREQ_HZ > 0U, "PWM_FREQ_HZ must be non-zero");
static_assert((MOT_SENSOR_DIR == 1) || (MOT_SENSOR_DIR == -1),
              "MOT_SENSOR_DIR must be 1 or -1");
```

你要理解：

```text
static_assert 不产生运行时代码
配置错了直接编译失败
比上电后调半天强很多
```

## 怎么验证你写对了

每写完一个头文件，跑这个命令：

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
#include "static_asserts.hpp"
#include "units.hpp"

using namespace cms32::support;

enum class Mode : unsigned char { A = 1U };
static_assert(to_underlying(Mode::A) == 1U, "enum helper failed");
static_assert(clamp_static<int, -2, 2>(5) == 2, "clamp failed");
static_assert(is_power_of_two(64U), "power-of-two failed");
static_assert(to_speed<262144>(Rpm{60}).value == 262144,
              "rpm conversion failed");

void support_compile_check()
{
    RingBuffer<unsigned char, 8> rb;
    (void)rb.push_isr(1U);
    unsigned char out = 0U;
    (void)rb.pop(out);

    LowPassI32<2> lp;
    (void)lp.update(100);

    SlewLimiter<int, 3> slew;
    (void)slew.update(10);
}
CPP
```

再跑主固件：

```sh
cmake --build build/gcc-debug --target cms32foc
cmake --build build/gcc-minsize --target cms32foc
```

## 写完后看什么

如果 Support 只是 header，还没有被任何 `.cpp` 使用，固件体积不应该变化。

等你把某个 `.cpp` 链接进来后，再检查：

```sh
arm-none-eabi-nm -C build/gcc-minsize/cms32foc \
  | rg "new|delete|__cxa|__gxx|vtable|typeinfo|throw"
```

不应该出现：

```text
operator new
operator delete
__cxa_throw
typeinfo
vtable
```

## 建议你怎么练

推荐顺序：

```text
1. 自己重写 clamp.hpp
2. 自己重写 units.hpp
3. 自己重写 slew_limiter.hpp
4. 自己重写 low_pass.hpp
5. 自己重写 ring_buffer.hpp
6. 最后写 irq_guard.hpp 和 static_asserts.hpp
```

每次只写一个文件。写完就编译。不要一次写一堆，不然报错会很难找。

最重要的是：每个组件都要能用一句话说明它运行时到底是什么。

```text
clamp: 两个比较
unit wrapper: 一个整数
slew limiter: 一个整数状态 + 两个比较
low pass: 一个整数状态 + 减法/移位/加法
ring buffer: 固定数组 + head/tail
irq guard: 构造关中断，析构开中断
```

能说清这一句，就说明没有被 C++ 表面语法绕进去。

## 完整参考代码

下面这些代码是参考答案，不是让你一开始就照抄。建议顺序是：

```text
1. 先看前面的讲解，自己写一版
2. 编译报错时只对照对应文件
3. 写完后再和这里逐行比较
4. 确认每个函数运行时到底生成什么动作
```

这些文件都用 `.hpp`，因为它们是 C++ header-only 组件。项目里继续保留 C ABI，后续 `.cpp` 可以 include 它们，但 `MotorControl.h`、`screw_axis.h` 这类对 C 暴露的头文件不要 include 这些 C++ 头。

### clamp.hpp

```cpp
#pragma once

#include <type_traits>

namespace cms32::support {

template <typename T>
constexpr T clamp(T value, T min_value, T max_value) noexcept
{
    return (value < min_value) ? min_value
                               : ((value > max_value) ? max_value : value);
}

template <typename T, T MinValue, T MaxValue>
constexpr T clamp_static(T value) noexcept
{
    static_assert(MinValue <= MaxValue, "invalid clamp range");
    return clamp<T>(value, MinValue, MaxValue);
}

template <typename T>
constexpr T abs_limit(T value, T limit) noexcept
{
    static_assert(std::is_signed<T>::value, "abs_limit requires signed type");
    return clamp<T>(value, static_cast<T>(-limit), limit);
}

template <typename T, T Limit>
constexpr T clamp_symmetric(T value) noexcept
{
    static_assert(std::is_signed<T>::value,
                  "clamp_symmetric requires signed type");
    static_assert(Limit >= static_cast<T>(0), "Limit must be non-negative");
    return clamp<T>(value, static_cast<T>(-Limit), Limit);
}

} // namespace cms32::support
```

重点看：

```text
clamp() 可以用于有符号和无符号
abs_limit()/clamp_symmetric() 只允许有符号类型
static_assert 是编译期检查，不产生运行时代码
```

### enum_utils.hpp

```cpp
#pragma once

#include <type_traits>

namespace cms32::support {

template <typename Enum>
constexpr auto to_underlying(Enum value) noexcept
    -> typename std::underlying_type<Enum>::type
{
    static_assert(std::is_enum<Enum>::value, "to_underlying requires enum");
    return static_cast<typename std::underlying_type<Enum>::type>(value);
}

} // namespace cms32::support
```

重点看：

```text
enum class 用于内部状态机
to_underlying() 是状态转成 watch 整数的出口
不要在代码里到处 static_cast<uint8_t>(state)
```

### units.hpp

```cpp
#pragma once

#include <stdint.h>

namespace cms32::support {

struct Rpm {
    int16_t value;
};

struct Milliseconds {
    uint16_t value;
};

struct CurrentCount {
    int16_t value;
};

struct VoltageCount {
    int16_t value;
};

struct EncoderRaw {
    uint16_t value;
};

struct EncoderPosition {
    int32_t value;
};

struct SpeedCounts {
    int32_t value;
};

struct Angle16 {
    uint16_t value;
};

template <int32_t CountsPerRev>
constexpr SpeedCounts to_speed(Rpm rpm) noexcept
{
    static_assert(CountsPerRev > 0, "CountsPerRev must be positive");
    return SpeedCounts{
        static_cast<int32_t>((static_cast<int32_t>(rpm.value) * CountsPerRev) /
                             60L)};
}

template <int32_t CountsPerRev>
constexpr Rpm to_rpm(SpeedCounts speed) noexcept
{
    static_assert(CountsPerRev > 0, "CountsPerRev must be positive");
    return Rpm{
        static_cast<int16_t>((speed.value * 60L) / CountsPerRev)};
}

constexpr Angle16 add_angle(Angle16 angle, int16_t delta) noexcept
{
    return Angle16{static_cast<uint16_t>(angle.value + delta)};
}

} // namespace cms32::support
```

重点看：

```text
这些 struct 只有一个字段，运行时就是一个整数
函数参数类型变成 Rpm/CurrentCount 后，裸 int16_t 不能乱传
模板参数 CountsPerRev 把换算基准固定在编译期
```

### slew_limiter.hpp

```cpp
#pragma once

#include <type_traits>

namespace cms32::support {

template <typename T>
constexpr T slew_step(T current, T target, T step) noexcept
{
    static_assert(std::is_signed<T>::value, "slew_step requires signed type");

    if (step <= static_cast<T>(0)) {
        return target;
    }

    const T delta = static_cast<T>(target - current);
    if (delta > step) {
        return static_cast<T>(current + step);
    }
    if (delta < static_cast<T>(-step)) {
        return static_cast<T>(current - step);
    }
    return target;
}

template <typename T, T Step>
class SlewLimiter {
public:
    static_assert(std::is_signed<T>::value, "SlewLimiter requires signed type");
    static_assert(Step > static_cast<T>(0), "Step must be positive");

    constexpr T update(T target) noexcept
    {
        value_ = slew_step<T>(value_, target, Step);
        return value_;
    }

    constexpr void reset(T value = static_cast<T>(0)) noexcept
    {
        value_ = value;
    }

    constexpr T value() const noexcept
    {
        return value_;
    }

private:
    T value_{0};
};

} // namespace cms32::support
```

重点看：

```text
SlewLimiter<int16_t, 4> 里只有一个 int16_t value_
Step 是编译期常量，不需要每次运行时从结构体里读
reset()/value() 是调试和状态机切换需要的最小接口
```

### low_pass.hpp

```cpp
#pragma once

#include <stdint.h>

namespace cms32::support {

template <uint8_t Shift>
class LowPassI32 {
public:
    static_assert(Shift < 31U, "Shift is too large");

    constexpr int32_t update(int32_t sample) noexcept
    {
        if constexpr (Shift == 0U) {
            value_ = sample;
        } else {
            value_ += (sample - value_) >> Shift;
        }
        return value_;
    }

    constexpr void reset(int32_t value = 0) noexcept
    {
        value_ = value;
    }

    constexpr int32_t value() const noexcept
    {
        return value_;
    }

private:
    int32_t value_{0};
};

} // namespace cms32::support
```

重点看：

```text
LowPassI32<0> 编译后没有滤波分支
LowPassI32<2> 编译后就是减法、右移、加法
这个类不分配内存，也不访问全局变量
```

### ring_buffer.hpp

```cpp
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace cms32::support {

constexpr bool is_power_of_two(size_t value) noexcept
{
    return (value != 0U) && ((value & (value - 1U)) == 0U);
}

template <typename T, size_t Capacity>
class RingBuffer {
public:
    static_assert(is_power_of_two(Capacity), "Capacity must be power of two");
    static_assert(Capacity >= 2U, "Capacity must leave one empty slot");

    RingBuffer() = default;
    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    bool push_isr(const T& value) noexcept
    {
        const size_t head = head_;
        const size_t next = wrap(head + 1U);
        if (next == tail_) {
            overflow_count_++;
            return false;
        }

        data_[head] = value;
        head_ = next;
        return true;
    }

    bool pop(T& out) noexcept
    {
        const size_t tail = tail_;
        if (tail == head_) {
            return false;
        }

        out = data_[tail];
        tail_ = wrap(tail + 1U);
        return true;
    }

    bool empty() const noexcept
    {
        return head_ == tail_;
    }

    bool full() const noexcept
    {
        return wrap(head_ + 1U) == tail_;
    }

    size_t size() const noexcept
    {
        return wrap(head_ - tail_);
    }

    static constexpr size_t capacity() noexcept
    {
        return Capacity - 1U;
    }

    uint32_t overflow_count() const noexcept
    {
        return overflow_count_;
    }

    void clear() noexcept
    {
        head_ = 0U;
        tail_ = 0U;
        overflow_count_ = 0U;
    }

private:
    static constexpr size_t wrap(size_t value) noexcept
    {
        return value & (Capacity - 1U);
    }

    T data_[Capacity]{};
    volatile size_t head_{0U};
    volatile size_t tail_{0U};
    volatile uint32_t overflow_count_{0U};
};

} // namespace cms32::support
```

重点看：

```text
这个版本只适合单 ISR 写、单主循环读
Capacity 必须是 2 的幂，但真实可用容量是 Capacity - 1
UART RX ISR 里只 push 字节，不做协议解析
```

### irq_guard.hpp

```cpp
#pragma once

#include "CMS32M6510.h"

namespace cms32::support {

template <IRQn_Type Irq>
class NvicIrqGuard {
public:
    NvicIrqGuard() noexcept
    {
        NVIC_DisableIRQ(Irq);
    }

    ~NvicIrqGuard() noexcept
    {
        NVIC_EnableIRQ(Irq);
    }

    NvicIrqGuard(const NvicIrqGuard&) = delete;
    NvicIrqGuard& operator=(const NvicIrqGuard&) = delete;
};

using AdcIrqGuard = NvicIrqGuard<ADC_IRQn>;

} // namespace cms32::support
```

重点看：

```text
构造函数关中断，析构函数开中断
只在很短的临界区使用
不要在 guard 生命周期内做串口解析、FOC 计算、长循环
```

### static_asserts.hpp

```cpp
#pragma once

#include "Config.h"
#include "MotorControl.h"
#include "ring_buffer.hpp"

namespace cms32::support {

static_assert(PWM_FREQ_HZ > 0U, "PWM_FREQ_HZ must be non-zero");
static_assert(PWM_PERIOD > 0U, "PWM_PERIOD must be non-zero");
static_assert(PWM_DUTY_MIN < PWM_DUTY_50, "PWM_DUTY_MIN must be below 50%");
static_assert(PWM_DUTY_50 < PWM_DUTY_MAX, "PWM_DUTY_50 must be below max");
static_assert(PWM_DUTY_MAX <= PWM_PERIOD, "PWM duty max exceeds period");

static_assert(CTRL_FAST_LOOP_DIV > 0U, "CTRL_FAST_LOOP_DIV must be non-zero");
static_assert(CTRL_SPD_EST_HZ > 0L, "CTRL_SPD_EST_HZ must be non-zero");
static_assert(CTRL_CUR_REF_RAMP_STEP > 0, "current ramp step must be positive");
static_assert(CTRL_SPD_IQ_SLEW_STEP > 0, "speed iq slew step must be positive");

static_assert(MOT_SENSOR_CPR > 0UL, "MOT_SENSOR_CPR must be non-zero");
static_assert(MOT_POLE_PAIRS > 0U, "MOT_POLE_PAIRS must be non-zero");
static_assert((MOT_SENSOR_DIR == 1) || (MOT_SENSOR_DIR == -1),
              "MOT_SENSOR_DIR must be 1 or -1");

static_assert(MOT_PWM_PHASE_MAP <= MOT_PHASE_MAP_WVU,
              "invalid PWM phase map");
static_assert(MOT_CURR_PHASE_MAP <= MOT_PHASE_MAP_WVU,
              "invalid current phase map");
static_assert((MOT_CURR_SIGN == 1) || (MOT_CURR_SIGN == -1),
              "MOT_CURR_SIGN must be 1 or -1");

static_assert(sizeof(MotorControlCommand_t) <= 128U,
              "MotorControlCommand_t is getting too large for command copy");
static_assert(is_power_of_two(128U), "default UART RX buffer size is invalid");

} // namespace cms32::support
```

重点看：

```text
static_asserts.hpp 会 include 项目配置，所以不要在所有地方乱 include
它适合放在某个 C++ 编译单元里做一次工程级检查
以后如果 C++ 文件还没接入主固件，可以先用单独编译命令检查它
```
