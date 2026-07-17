# Modern Cpp Adoption Guide

本文说明如何把现代 C++ 特性逐步写进当前固件，同时保持嵌入式项目需要的确定性、零动态内存、可读性和可控体积。参考方向来自用户给出的 Awesome Modern C++ embedded 教程：

```text
https://awesome-embedded-learning-studio.github.io/Tutorial_AwesomeModernCPP/vol8-domains/embedded/
```

本文不是要求立刻重写 FOC。当前策略是：先让 C ABI 和快环稳定，再把 C++ 用在能带来类型安全和编译期检查的位置。

## 总原则

允许：

```text
enum class
constexpr / consteval 辅助计算
template 编译期配置
if constexpr
static_assert
强类型单位 wrapper
固定容量容器
RAII 小 guard
CRTP / policy class
extern "C" C ABI 包装
```

禁止：

```text
new / delete
malloc / free
std::vector
std::string
std::function
iostream
exception
RTTI
shared_ptr / weak_ptr
快环路径 virtual dispatch
运行时工厂/注册表
```

判断标准：

```text
能在编译期决定的，不留到运行时。
能用类型表达的，不用裸 int 混传。
能静态分配的，不动态分配。
能保持 C ABI 的，不破坏现有 main/IRQ/Ozone 入口。
快环里只接受可内联、无堆、无异常、无虚表的代码。
```

## CMake 接入方式

当前顶层已经启用 C++：

```cmake
project(CMS32FOC_GCC C CXX ASM)
```

保持 C++17：

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

公共 C++ 限制建议加到单独的 interface target：

```cmake
add_library(cms32_cpp_options INTERFACE)
target_compile_options(cms32_cpp_options INTERFACE
    -fno-exceptions
    -fno-rtti
    -fno-threadsafe-statics
    -fno-use-cxa-atexit
)
```

第一阶段不要把整个控制层改成 C++。先新增小 target：

```cmake
add_library(cms32_support_cpp INTERFACE)
target_include_directories(cms32_support_cpp INTERFACE
    Firmware/Support
)
target_link_libraries(cms32_support_cpp INTERFACE
    cms32_cpp_options
)
```

当某个模块迁移为 C++，例如 `screw_axis.cpp`：

```cmake
add_library(cms32_app_screw_axis STATIC
    Firmware/App/screw_axis.cpp
)
target_link_libraries(cms32_app_screw_axis PUBLIC
    cms32_motor_control_c
    cms32_support_cpp
)
```

对外函数继续保持 C ABI：

```cpp
extern "C" void ScrewAxis_Init(void);
extern "C" void ScrewAxis_Run(void);
extern "C" void ScrewAxis_OnAdcSample(void);
```

这样 `main.c` 不需要马上迁移。

## 建议新增 Support 层

Board 层 UART bring-up 先保持 C，负责最小硬件初始化和字节收发。下面目录用于
后续 Comm 协议层，不直接替代 `Firmware/Board/Src/board_uart.c`：

推荐目录：

```text
Firmware/Support/
├── units.hpp
├── clamp.hpp
├── fixed_point.hpp
├── irq_guard.hpp
├── ring_buffer.hpp
├── static_string_view.hpp
└── expected.hpp
```

这些文件应保持 header-only、无全局对象构造、无动态内存。

## enum class

适合替代状态宏：

```cpp
enum class HomeState : uint8_t {
    Idle,
    FastRetract,
    FastBackoff,
    SlowRetract,
    FinalBackoff,
    Done,
    Fault,
};
```

不要让状态和模式、fault、普通 `uint8_t` 混用。

对外 watch 如果仍要给 Ozone 看数字，可以显式转换：

```cpp
template<typename Enum>
constexpr uint8_t to_u8(Enum value)
{
    return static_cast<uint8_t>(value);
}
```

应用到当前项目：

```text
ScrewAxis home state
MotorControl state/mode/fault
串口协议 command id
串口 parser state
Board diagnostic state
```

## constexpr

适合替代宏里的纯计算：

```cpp
constexpr int32_t speed_counts_per_rev =
    static_cast<int32_t>(MOT_SENSOR_CPR) *
    static_cast<int32_t>(MOT_SENSOR_POLE_PAIRS);

constexpr int32_t rpm_to_speed_counts(int16_t rpm)
{
    return (static_cast<int32_t>(rpm) * speed_counts_per_rev) / 60L;
}
```

注意：`constexpr` 不是要求所有调用都在编译期执行。参数是运行时变量时，它仍然生成普通指令；参数是常量时，编译器可以直接折叠。

适合放入：

```text
速度单位换算
PWM/SVPWM 限幅推导
采样分频推导
螺杆回零默认参数
协议帧长度/CRC 表大小
```

## 模板限幅和斜坡

当前 C 里有很多 `clamp_s16`、`clamp_u16`。C++ 可以写成：

```cpp
template<typename T, T Min, T Max>
constexpr T clamp(T value)
{
    static_assert(Min <= Max);
    return (value < Min) ? Min : ((value > Max) ? Max : value);
}
```

使用：

```cpp
auto iq = clamp<int16_t, 1, SCREW_IQ_LIMIT_MAX>(cmd.iq_limit);
auto speed = clamp<int16_t, -SCREW_SPEED_MAX_RPM, SCREW_SPEED_MAX_RPM>(cmd.speed_rpm);
```

斜坡限制：

```cpp
template<typename T, T Step>
constexpr T slew(T current, T target)
{
    static_assert(Step > 0);
    const T delta = target - current;
    if (delta > Step) {
        return current + Step;
    }
    if (delta < -Step) {
        return current - Step;
    }
    return target;
}
```

这类模板没有状态，优化后通常和手写 C 一样。

## 强类型单位

当前项目里很多量都是裸 `int16_t/int32_t`：

```text
rpm
encoder count/s
ADC current count
SVPWM voltage count
encoder raw position
```

建议先做轻量 wrapper：

```cpp
struct Rpm {
    int16_t value;
};

struct SpeedCounts {
    int32_t value;
};

struct CurrentCount {
    int16_t value;
};

struct VoltageCount {
    int16_t value;
};
```

换算函数显式表达输入输出：

```cpp
template<int32_t CountsPerRev>
constexpr SpeedCounts to_speed_counts(Rpm rpm)
{
    return SpeedCounts{(static_cast<int32_t>(rpm.value) * CountsPerRev) / 60L};
}
```

这样 `iq_limit` 不会误传给速度函数，`speed_ref_rpm` 不会误传给电流限幅。

使用范围：

```text
先用于 ScrewAxis 和串口命令解析
再用于速度环外层
最后再考虑 FOC 快环内部
```

## if constexpr

适合编译期选择不同策略：

```cpp
enum class SpeedFeedbackSource : uint8_t {
    RawDiff,
};

template<SpeedFeedbackSource Source>
int32_t update_speed_feedback()
{
    static_assert(Source == SpeedFeedbackSource::RawDiff);
    return estimate_from_raw_diff();
}
```

不用的分支不会进入最终代码。适合：

```text
速度差分滤波策略选择
串口协议 CRC 有/无
诊断固件功能开关
不同板级引脚策略
```

不要把运行时命令也硬塞进 `if constexpr`。Ozone 命令、串口命令、实际 mode 仍是运行时状态。

## static_assert

嵌入式配置错误要尽早在编译期失败：

```cpp
static_assert(PWM_FREQ_HZ == 20000U);
static_assert(CTRL_FAST_LOOP_DIV > 0U);
static_assert((PWM_DUTY_MIN < PWM_DUTY_50) && (PWM_DUTY_50 < PWM_DUTY_MAX));
static_assert(sizeof(MotorControlCommand_t) <= 64);
```

适合检查：

```text
协议帧最大长度
ring buffer 容量必须为 2 的幂
PWM duty 边界
FOC 分频不能为 0
命令结构大小
Watch 结构大小
```

## RAII guard

RAII 可以用，但只用于小范围资源恢复，不用于堆对象管理。

例如 ADC IRQ 临界区：

```cpp
class AdcIrqGuard {
public:
    AdcIrqGuard()
    {
        NVIC_DisableIRQ(ADC_IRQn);
    }

    ~AdcIrqGuard()
    {
        NVIC_EnableIRQ(ADC_IRQn);
    }

    AdcIrqGuard(const AdcIrqGuard&) = delete;
    AdcIrqGuard& operator=(const AdcIrqGuard&) = delete;
};
```

使用：

```cpp
{
    AdcIrqGuard guard;
    state.command = next_command;
}
```

注意：

```text
不依赖异常
不在析构里做复杂逻辑
不持有动态资源
析构函数必须可内联、可预测
```

## 固定容量 Ring Buffer

串口模块还没调试，所以 C++ 文档必须提前约束串口设计。推荐 SPSC ring buffer：ISR 单生产者写入，主循环单消费者读取。

```cpp
template<typename T, size_t Capacity>
class RingBuffer {
public:
    static_assert(Capacity >= 2);
    static_assert((Capacity & (Capacity - 1U)) == 0U);

    bool push_isr(T value)
    {
        const size_t next = (head_ + 1U) & (Capacity - 1U);
        if (next == tail_) {
            overflow_count_++;
            return false;
        }
        data_[head_] = value;
        head_ = next;
        return true;
    }

    bool pop(T& out)
    {
        if (tail_ == head_) {
            return false;
        }
        out = data_[tail_];
        tail_ = (tail_ + 1U) & (Capacity - 1U);
        return true;
    }

    uint32_t overflow_count() const
    {
        return overflow_count_;
    }

private:
    T data_[Capacity]{};
    volatile size_t head_{0U};
    volatile size_t tail_{0U};
    volatile uint32_t overflow_count_{0U};
};
```

串口 ISR：

```cpp
extern "C" void UART_IRQHandler(void)
{
    while (uart_rx_ready()) {
        (void)rx_buffer.push_isr(uart_read_byte());
    }
}
```

主循环解析：

```cpp
uint8_t byte;
while (rx_buffer.pop(byte)) {
    parser.feed(byte);
}
```

规则：

```text
ISR 只收字节，不解析完整协议
主循环解析帧
不在 ISR 里调用复杂回调
不用 std::vector/std::string
帧缓冲固定大小
溢出计数进 watch
```

## 串口协议 Parser

推荐写成固定状态机：

```cpp
enum class ParserState : uint8_t {
    WaitSync,
    Header,
    Payload,
    Crc,
};

enum class ParseResult : uint8_t {
    None,
    FrameReady,
    BadCrc,
    TooLong,
};

template<size_t MaxPayload>
class FrameParser {
public:
    ParseResult feed(uint8_t byte);
    bool take_frame(Frame& out);

private:
    ParserState state_{ParserState::WaitSync};
    uint8_t payload_[MaxPayload]{};
    uint8_t length_{0U};
    uint8_t index_{0U};
};
```

协议命令用 `enum class`：

```cpp
enum class CommandId : uint8_t {
    MotorEnable = 0x01,
    SetSpeedRpm = 0x02,
    StartHome = 0x10,
    StopHome = 0x11,
    ReadWatch = 0x80,
};
```

把串口命令转换成现有 C ABI：

```cpp
void apply_command(const DecodedCommand& command)
{
    switch (command.id) {
        case CommandId::SetSpeedRpm:
            g_motor_cmd.speed_ref_rpm = command.speed.value;
            break;

        case CommandId::StartHome:
            g_screw_home_cmd.start_seq++;
            break;

        default:
            break;
    }
}
```

不要让串口模块直接写控制层内部 `s_mc`。串口只能写公共命令结构或调用公共 C API。

## Expected / Error 返回

如果工具链和标准库支持 `std::expected`，可以考虑。但为保持 freestanding 可控，建议先写项目内极小版本：

```cpp
template<typename T, typename E>
class Expected {
public:
    static Expected ok(T value);
    static Expected err(E error);

    bool has_value() const;
    T value() const;
    E error() const;

private:
    bool has_value_;
    union {
        T value_;
        E error_;
    };
};
```

更保守的做法：解析器返回 `enum class ParseResult`，数据通过输出参数传出。串口模块第一版建议用保守做法。

## CRTP / Policy Class

CRTP 适合以后做可替换硬件策略，但不应先改快环。

可用于串口：

```cpp
template<typename Driver, size_t RxSize>
class UartPort {
public:
    void irq_handler()
    {
        while (Driver::rx_ready()) {
            (void)rx_.push_isr(Driver::read());
        }
    }

private:
    RingBuffer<uint8_t, RxSize> rx_;
};
```

`Driver` 是编译期策略，不需要虚函数：

```cpp
struct Cms32Uart0Driver {
    static bool rx_ready();
    static uint8_t read();
    static void write(uint8_t byte);
};
```

适合未来扩展：

```text
UART0/UART1
不同编码器驱动
不同电流采样策略
诊断固件 mock driver
```

## 类型安全寄存器访问

可以借鉴教程里的思想，但本项目不建议立刻重写原厂寄存器库。先在新写的串口驱动或小外设上试。

示例：

```cpp
template<uint32_t Address>
struct Reg32 {
    static volatile uint32_t& ref()
    {
        return *reinterpret_cast<volatile uint32_t*>(Address);
    }

    static void set_bits(uint32_t mask)
    {
        ref() |= mask;
    }

    static void clear_bits(uint32_t mask)
    {
        ref() &= ~mask;
    }
};
```

使用规则：

```text
先包新增串口寄存器，不动 ADC/PWM 快环
每个寄存器封装必须很薄
不要隐藏硬件时序
不要做运行时地址表
```

## 迁移顺序

### 阶段 1：只加 Support，不改行为

新增：

```text
Firmware/Support/clamp.hpp
Firmware/Support/units.hpp
Firmware/Support/ring_buffer.hpp
Firmware/Support/irq_guard.hpp
```

构建增加 C++，但主控制链行为不变。

验收：

```text
cms32foc Debug 编译通过
cms32foc MinSizeRel 编译通过
Flash/RAM 体积对比没有异常增长
```

### 阶段 2：迁移 ScrewAxis

`screw_axis.c` 改为 `screw_axis.cpp`，对外仍保留：

```cpp
extern "C" void ScrewAxis_Init(void);
extern "C" void ScrewAxis_Run(void);
extern "C" void ScrewAxis_OnAdcSample(void);
```

内部使用：

```text
enum class HomeState
Rpm / CurrentCount
constexpr 限幅
小型 RAII guard 如有必要
```

验收：

```text
符号表仍只暴露 g_screw_home_cmd/g_screw_home_watch
回零状态机行为不变
Flash/RAM 没有异常增长
```

### 阶段 3：新增串口模块

推荐目录：

```text
Firmware/Comm/
├── uart_port.cpp
├── uart_port.hpp
├── serial_protocol.cpp
└── serial_protocol.hpp
```

串口模块分层：

```text
Board UART byte API
  -> RingBuffer<uint8_t, N>
  -> FrameParser<MaxPayload>
  -> Command decoder
  -> apply to g_motor_cmd / g_screw_home_cmd
  -> response encoder
```

串口模块不直接调用快环内部函数，不直接访问 `MotorControlCState`。

### 阶段 4：抽纯数学小组件

可以考虑：

```text
FixedPiConfig
SlewLimiter
LowPassFilter
Angle16
```

先作为新文件并做 A/B 对比，不直接大改 `motor_control_current.c`。

### 阶段 5：最后才动 FOC 快环

只有当阶段 1-4 的体积、可读性和调试体验都稳定后，才考虑把：

```text
motor_control_current.c
motor_control_encoder.c
foc_math.c
```

逐步迁移。迁移时必须保持：

```text
无异常
无 RTTI
无堆
无虚函数
无运行时多态
快环函数可内联
Ozone watch 字段连续可观察
```

## 代码审查清单

每次引入 C++ 代码时检查：

```text
是否引入 new/delete/malloc/free
是否打开异常或 RTTI
是否使用 std::vector/std::string/std::function/iostream
是否在 ISR 中调用复杂对象或回调
是否有静态局部变量需要线程安全初始化
是否有虚函数进入快环
是否有不可控全局构造
是否 MinSizeRel 体积异常增长
是否 C ABI 仍然稳定
```

工具检查：

```sh
arm-none-eabi-nm -C build/gcc-minsize/cms32foc | rg "new|delete|__cxa|__gxx|vtable|typeinfo|throw"
arm-none-eabi-size build/gcc-minsize/cms32foc
```

期望：

```text
没有 __cxa_throw
没有 operator new/delete
没有 typeinfo/vtable 进入快环模块
Flash/RAM 增长有明确原因
```

## 本项目推荐的第一批 C++ 特性

优先级从高到低：

```text
1. enum class：状态机和协议 ID
2. constexpr：参数推导和单位换算
3. static_assert：配置合法性检查
4. 强类型单位：Rpm、CurrentCount、VoltageCount、SpeedCounts
5. 模板限幅/斜坡/滤波：零开销小组件
6. 固定容量 ring buffer：串口 RX/TX
7. if constexpr：编译期选择协议/反馈源/驱动策略
8. RAII guard：短临界区
9. CRTP/policy：串口和未来驱动抽象
```

不推荐第一批使用：

```text
std::variant 状态机
std::expected
复杂寄存器 DSL
模板元编程
把整个 FOC 控制层一次性 C++ 化
```

这些不是不能用，而是当前项目更需要稳定调试、清晰边界和可控体积。
