# C/C++ 混编最小封装落地指南

本文记录当前 MotorControl 的最终落地方向：C 保持稳定 ABI，C++ 负责类型安全、
编译期约束和状态所有权。控制路径由 `MotorController g_motor` 统一拥有，但不做
多层 getter/facade，也不恢复 watch 转发。

如果本文和早期 09-22 阶段文档冲突，以本文和当前源码为准。

## 1. 当前边界是什么

对外只看 `MotorControl.h`：

```c
extern volatile MotorControlCommand_t g_mc_cmd;

void MotorControl_Init(void);
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t *command);
void MotorControl_RunSlowLoop(void);
uint8_t MotorControl_FastLoopFromAdcIrq(void);
```

这就是 C ABI 外壳。`main.c`、中断入口、ScrewAxis、后续串口命令都通过这里和
MotorControl 交互。

MotorControl 内部不再有 `MotorControlCState` 总状态树，也不再拆成多个
`g_mc_*` 状态全局。当前真实状态由 `cms32::motor::g_motor` 单例拥有：

```text
g_motor.runtime   慢环状态机和 PWM 输出标志
g_motor.command   慢环锁存后的快环命令缓存
g_motor.encoder   编码器 raw、电角度、累计位置
g_motor.position  1 kHz 位置 P 环状态
g_motor.speed     速度估算、速度 PI、速度环输出 iq_ref
g_motor.current   三相电流、dq 电流、电流 PI、斜坡后的 id/iq 给定
g_motor.output    dq/alpha-beta 电压、SVPWM duty、限幅标志
g_motor.check     慢环安全检查结果
g_motor.diag      诊断计数和拒绝详情
g_motor.debug     Ozone 枚举调试摘要
```

这样做的目的很直接：Ozone 能展开真实数据，FOC 代码也不再出现
`state.closed_loop.current.pi_d` 这种长路径。

## 2. 为什么不再做更多封装

当前固件要优先满足三件事：

```text
20 kHz 快环路径直接、可读
Ozone 看到的就是参与控制的真实状态
C/C++ 混编边界简单，不引入第二套状态
```

所以不做：

```text
MotorControl class
状态 getter/setter 体系
大 watch 结构体转发
所有数值都强类型包装
```

这些东西会让代码“看起来更 C++”，但对当前项目不一定更清楚。我们只在容易写错的地方
使用 C++。

## 3. 哪些地方用 C++ 类型安全

状态和模式用枚举：

```cpp
enum class ControlState : uint8_t;
enum class ControlMode : uint8_t;
enum class ControlFault : uint8_t;
```

真实 `g_motor.runtime.state/mode/fault` 使用 C enum，方便 Ozone 展示枚举名；
C++ 内部比较和分支使用 `enum class`，避免把 mode、state、fault 混着比。

配置用 `static constexpr`：

```cpp
CurrentLoopConfig::kp
SpeedLoopConfig::sample_div
PwmConfig::duty_center
```

硬件和调参边界宏仍保留在 `BoardConfig.h` / `TuneConfig.h`，但 C++ 算法文件优先读
`config.hpp`。

单位只包装高风险换算：

```cpp
Rpm
SpeedCounts
Angle16
```

PI 多参数使用结构体：

```cpp
FixedPiConfig
FixedPiRef
```

这些包装都是轻量对象，不使用堆、异常、RTTI 或虚函数。

## 4. 快环怎么写

快环直接读写扁平状态：

```cpp
g_motor.current.phase.u = curr_u();
g_motor.current.phase.v = curr_v();
g_motor.current.phase.w = curr_w();

const uint16_t theta =
    static_cast<uint16_t>(g_motor.encoder.elec +
                          g_motor.command.current.voltage_theta_offset);

const FocAlphaBeta_t current_ab = foc_clarke_3phase(g_motor.current.phase);
g_motor.current.dq = foc_park(current_ab, theta);

FixedPiRef{g_motor.current.pi_d}.set_gains(current_pi_config);
FixedPiRef{g_motor.current.pi_q}.set_gains(current_pi_config);

g_motor.output.voltage_dq.d =
    FixedPiRef{g_motor.current.pi_d}.update(g_motor.current.id_ref_active.value,
                                         g_motor.current.dq.d);
g_motor.output.voltage_dq.q =
    FixedPiRef{g_motor.current.pi_q}.update(g_motor.current.iq_ref_active.value,
                                         g_motor.current.dq.q);
```

这就是当前推荐风格：不传一个很大的 `mc` 指针，不取四五层成员，不额外包 getter。

## 5. 命令为什么还要复制

命令入口是 volatile：

```text
g_mc_cmd
```

它可能被 Ozone、App、未来串口改。ADC IRQ 不直接读它，而是读慢环锁存后的：

```text
g_motor.command.current
g_motor.command.speed
g_motor.command.vf
```

这样做不是为了“封装”，而是为了实时性和一致性：

```text
慢环负责 snapshot + sanitize
快环只读已限幅、已拆分的小命令
ADC IRQ 不碰 volatile 公共入口
```

## 6. Ozone 现在看什么

命令：

```text
g_mc_cmd
```

状态和故障：

```text
g_motor.runtime
g_motor.debug
g_motor.check
```

FOC 主链路：

```text
g_motor.current
g_motor.encoder
g_motor.speed
g_motor.output
```

异常计数：

```text
g_motor.diag
```

不要再看 `g_motor_state`、`g_motor_watch` 或 `MotorControlCState`。这些是旧阶段概念。

## 7. 旧 C 文件怎么处理

旧 C 对照实现移动到：

```text
Firmware/MotorControl/LegacyC/
```

它们只作为历史参考，不接入 CMake target，不维护成第二套可编译控制实现。当前正式实现是：

```text
Firmware/MotorControl/Core/core.cpp
Firmware/MotorControl/Core/current.cpp
Firmware/MotorControl/Core/encoder.cpp
Firmware/MotorControl/Core/output.cpp
Firmware/MotorControl/Core/vf.cpp
```

## 8. 每次改完怎么验收

```sh
./testhpp.sh
cmake --build --preset gcc-debug --target cms32foc
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "g_mc_|g_motor_state|g_motor_cmd|g_motor_watch"
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "operator new|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
rg "g_motor_state|g_motor_cmd|MotorControlCState|mc->closed_loop" Firmware/MotorControl/Core Firmware/App Firmware/MotorControl/Abi Docs/CurrentProgram
```

验收标准：

```text
主固件能编译
ELF 只出现 `g_mc_cmd` 和 `cms32::motor::g_motor`，不出现旧 `g_motor_state/g_motor_watch`
没有 C++ 运行时污染
当前程序文档只教 `g_mc_cmd` 命令入口和 `g_motor.*` 观察路径
FOC 快环保持直读直写
```
