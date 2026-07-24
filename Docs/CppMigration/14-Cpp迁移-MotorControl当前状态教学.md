# MotorControl C++ 当前状态教学

本文记录当前 checkout 的 C/C++ 混编实际状态。它不是第四阶段历史教程全文，而是回答：

```text
MotorControl 现在到底走哪套代码？
Ozone 现在看哪些变量？
C++ 用到了哪里，哪里刻意不封装？
```

当前最终落地规则见：

```text
Docs/CppMigration/25-Cpp迁移-C_Cpp混编最小封装落地指南.md
```

如果早期 09-13 阶段文档里的旧名字和本文冲突，以本文和当前源码为准。

## 1. 当前结论

当前主固件已经不是“C 状态树 + watch 转发”的结构。

现在是：

```text
C ABI 外壳
  MotorControl.h

C++ 内部实现
  core.cpp
  current.cpp
  encoder.cpp
  output.cpp
  vf.cpp

扁平全局状态
  g_motor.runtime
  g_motor.command
  g_motor.encoder
  g_motor.speed
  g_motor.current
  g_motor.output
  g_motor.check
  g_motor.diag
  g_motor.debug
```

不再使用：

```text
g_motor_cmd
g_motor_watch
g_motor_state
MotorControlCState
MotorControl_UpdateWatch()
MotorControl_GetWatch()
```

## 2. C ABI 现在只保留什么

`MotorControl.h` 是外部唯一稳定入口：

```c
extern volatile MotorControlCommand_t g_mc_cmd;

void MotorControl_Init(void);
void MotorControl_ApplyCommand(const volatile MotorControlCommand_t *command);
void MotorControl_RunSlowLoop(void);
uint8_t MotorControl_FastLoopFromAdcIrq(void);
```

`main.c` 的主循环就是：

```c
ScrewAxis_Run();
MotorControl_ApplyCommand(&g_mc_cmd);
MotorControl_RunSlowLoop();
```

ADC 中断入口只调用：

```c
MotorControl_FastLoopFromAdcIrq();
```

这就是 C/C++ 混编边界。外部不用知道内部已经是 C++。

## 3. Ozone 现在看什么

命令入口：

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

诊断计数：

```text
g_motor.diag
```

`g_motor.debug` 只放枚举调试摘要，方便 Ozone 显示名字。电流、速度、编码器、PWM duty
仍直接看真实状态，不做二次复制。

## 4. C++ 当前用在哪里

状态类用 `enum class`：

```cpp
ControlState
ControlMode
ControlFault
```

配置用 `static constexpr`：

```cpp
CurrentLoopConfig
SpeedLoopConfig
PwmConfig
EncoderConfig
```

PI 多参数用结构体：

```cpp
FixedPiConfig
FixedPiRef
```

单位换算只在高风险位置轻量包装：

```cpp
Rpm
SpeedCounts
Angle16
```

这些都是零堆、无异常、无 RTTI、无虚函数的用法。

## 5. 为什么不继续封装

当前项目更需要：

```text
快环路径直接
Ozone 看真实状态
代码路径短
没有第二套状态
```

所以不做：

```text
MotorControl class
getter/setter 体系
状态 facade
大 watch 转发
所有变量强类型化
```

FOC 快环直接读写扁平状态，例如：

```cpp
g_motor.current.phase.u = curr_u();
g_motor.current.dq = foc_park(current_ab, theta);
g_motor.output.voltage_dq.q = FixedPiRef{g_motor.current.pi_q}.update(
    g_motor.current.iq_ref_active.value,
    g_motor.current.dq.q);
```

这比旧的多层状态树更直观，也比再包一层 getter 更适合当前固件。

## 6. 旧 C 文件在哪里

旧 C 对照实现已经移动到：

```text
Firmware/MotorControl/LegacyC/
```

它们只作为历史参考，不接入当前 CMake target，不维护成第二套可编译控制实现。

当前仍在 `Firmware/MotorControl/LegacyC/` 的主要是 C 兼容内部头：

```text
motor_control_internal.h
motor_control_vf.h
```

## 7. 当前验证方式

每次改完 MotorControl C++ 主线，至少跑：

```sh
./testhpp.sh
cmake --build --preset gcc-debug --target cms32foc
```

符号检查：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "g_mc_|g_motor_state|g_motor_cmd|g_motor_watch"
```

正常应只看到 `g_mc_cmd` 和 `cms32::motor::g_motor`，不应再看到旧
`g_motor_state/g_motor_watch`。

C++ runtime 污染检查：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "operator new|__cxa|__gxx|typeinfo|vtable|exception|throw|malloc|free"
```

正常应无输出。

## 8. 下一步

后续继续 C++ 化时，优先遵守这个顺序：

```text
1. 先保持 C ABI 不变。
2. 内部状态由 `g_motor` 单例拥有，Ozone 直接展开 `g_motor.*`。
3. 快环直接读写真实状态。
4. 只在 enum、配置、单位、PI 参数这些地方用 C++ 类型安全。
5. 不恢复 watch，不增加多层 getter/facade。
```
