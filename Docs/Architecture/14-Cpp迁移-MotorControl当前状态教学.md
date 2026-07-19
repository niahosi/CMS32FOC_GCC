# MotorControl C++ 当前状态教学

本文记录当前仓库的 C/C++ 混编实际状态，并说明下一步应该怎么手写程序。
它是教学笔记，不直接替你改 `Firmware/MotorControl/Cpp/*.cpp` 或 `*.hpp`。

阅读顺序建议：

```text
1. Docs/Architecture/11-Cpp迁移-分阶段实践指南.md
2. Docs/Architecture/13-Cpp迁移-MotorControl核心调度层动手指南.md
3. 本文
```

前两篇讲通用路线和完整示例；本文只讲“当前 checkout 现在卡在哪，下一步怎么写”。

如果 `MotorControl` C++ core 已经写完并接入 CMake，继续读：

```text
Docs/Architecture/15-Cpp迁移-MotorControl数学小组件动手指南.md
Docs/Architecture/16-Cpp迁移-FOC坐标变换动手指南.md
Docs/Architecture/17-Cpp迁移-编码器与速度估算动手指南.md
Docs/Architecture/18-Cpp迁移-串口通信动手指南.md
Docs/Architecture/19-Cpp迁移-Board薄封装动手指南.md
```

## 当前仓库状态

当前工程已经不是纯 C 工程：

```text
project(CMS32FOC_GCC C CXX ASM)
```

已经实际启用的 C++ 部分：

```text
Firmware/Support/*.hpp
  clamp / units / enum_utils / irq_guard 等 header-only 工具已经存在。

Firmware/App/screw_axis.cpp
  ScrewAxis 已经是 C++ 实现，对外仍保留 ScrewAxis_* C ABI。

Firmware/MotorControl/Cpp/
  motor_control_types.hpp
  motor_control_config.hpp
  motor_command_sanitizer.hpp
  motor_control_core.cpp
```

还没有切换的部分：

```text
CMake 当前仍然把 Firmware/MotorControl/C/motor_control_c.c 编进 cms32_motor_control_c。
Firmware/MotorControl/Cpp/motor_control_core.cpp 现在只是草稿，没有参与主固件链接。
```

这意味着当前主固件运行时仍走 C shell：

```text
MotorControl_Init()
MotorControl_ApplyCommand()
MotorControl_RunSlowLoop()
MotorControl_FastLoopFromAdcIrq()
MotorControl_GetWatch()
MotorControl_UpdateWatch()
```

实际定义仍来自：

```text
Firmware/MotorControl/C/motor_control_c.c
```

## 现在不能直接切 CMake 的原因

`motor_control_core.cpp` 当前只写到：

```text
MotorControl_Init()
MotorControl_ApplyCommand()
```

还缺这四个 C ABI：

```text
MotorControl_RunSlowLoop()
MotorControl_FastLoopFromAdcIrq()
MotorControl_GetWatch()
MotorControl_UpdateWatch()
```

如果现在就把 CMake 从 `motor_control_c.c` 换到 `motor_control_core.cpp`，链接阶段会找不到这些符号。

所以正确顺序是：

```text
1. 先让 C++ core 草稿补齐全部 C ABI。
2. 单独编译 motor_control_core.cpp，确认 C++ 语法和 include 没问题。
3. 再切 CMake，让 C++ core 替代 C shell。
4. 构建完整 cms32foc。
5. 上板只验证 shell 行为，不改快环。
```

## 当前已经可以保留的 C++ 草稿

### motor_control_types.hpp

这个文件方向是对的：把 C 宏包成 `enum class`。

你要记住它的定位：

```text
C 宏仍然是过渡期数字协议源头。
C++ 里用 enum class 防止 mode/state/fault 混成普通 uint8_t。
```

写程序时不要急着删：

```text
MC_STATE_*
MC_MODE_*
MC_FAULT_*
```

因为 `motor_control_current.c`、`motor_control_encoder.c`、`motor_control_watch.c`
还在通过 C 内部状态协作。

### motor_control_config.hpp

这个文件方向也是对的：把 `BoardConfig.h` / `TuneConfig.h` 的业务参数整理成
C++ 编译期配置。

当前阶段它只是镜像：

```text
CurrentLoopConfig::ref_limit  -> CTRL_CUR_REF_LIMIT
SpeedLoopConfig::kp           -> CTRL_SPD_KP
EncoderConfig::counts_per_rev -> MOT_SENSOR_CPR * MOT_SENSOR_POLE_PAIRS
```

不要把它误解成运行时调参表。它不会给 Ozone 提供可写变量，也不应该创建对象。

### motor_command_sanitizer.hpp

这个文件现在拆得比原教学示例更好：

```text
snapshot()
  只从 volatile MotorControlCommand_t 逐字段复制。

sanitize()
  对普通 MotorControlCommand_t 做限幅和 rpm 转 speed count/s。

current_command() / speed_command() / vf_command()
  把完整命令拆成快环直接读取的小命令缓存。
```

这个拆法可以保留。它比在一个函数里同时做复制、限幅、拆分更适合教学，也更容易单独测试。

## 下一步 1：补完 motor_control_core.cpp 的四个 C ABI

你现在不要先动 CMake。先在现有 `motor_control_core.cpp` 的
`MotorControl_ApplyCommand()` 后面继续手写四个函数。

### RunSlowLoop 应该怎么想

C 版本慢环做三件事：

```text
1. 读取当前电流和安全检查。
2. 如果没 enable，只计数然后返回。
3. 根据 mode 和 ready 条件进入 closed-loop 或 fault。
```

C++ 写法不要照搬大 if，而是使用已经写好的 helper：

```text
current_mode()
refresh_slow_checks(mode)
is_supported_run_mode(mode)
enter_ready_state(mode)
enter_fault_state(fault)
fault_for_not_ready(mode)
```

你要写出的结构应该长这样：

```cpp
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
```

这段代码的教学重点不是“少写几行”，而是把判断分层：

```text
refresh_slow_checks 负责刷新事实。
is_supported_run_mode 负责 mode 合法性。
enter_ready_state / enter_fault_state 负责状态切换副作用。
```

### FastLoopFromAdcIrq 应该怎么想

快环入口不能变复杂。它仍然只做：

```text
1. 调 bsp_adc_irq()。
2. 没形成有效采样就返回 0。
3. 已 enable 且 closed-loop 时，根据 mode 分发到旧 C 快环。
4. 返回 1 表示这次有有效控制采样。
```

结构应该是：

```cpp
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
```

这一步禁止顺手改：

```text
MotorControl_CurrentRunFastLoop()
MotorControl_InternalUpdateEncoderAngle()
MotorControl_InternalUpdateEncoderSpeed()
MotorControl_InternalApplyVoltageVector()
MotorControlVf_RunFastLoop()
```

第四阶段只迁 shell，不迁 FOC 快环。

### Watch 两个函数应该怎么想

watch 逻辑继续复用 C 文件里的填充函数：

```text
MotorControl_WatchFill()
MotorControl_WatchCopyToVolatile()
```

普通快照：

```cpp
extern "C" void MotorControl_GetWatch(MotorControlWatch_t* out)
{
    if (out == nullptr)
    {
        return;
    }

    MotorControl_WatchFill(&s_mc, out);
}
```

Ozone volatile 入口：

```cpp
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

不要直接对 `volatile MotorControlWatch_t*` 做大段复杂计算。先填普通局部变量，再逐字段 copy 到 volatile。

## 下一步 2：单独编译 C++ core

补完四个函数后，先不要改 CMake。用单文件编译确认语法：

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
  -I Firmware/Drivers/CMS32M6510 \
  -I Firmware/Drivers/CMS32M6510/Memory/Inc \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/CMSIS/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Include \
  -I Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Driver/inc \
  -c Firmware/MotorControl/Cpp/motor_control_core.cpp \
  -o /tmp/cms32_motor_control_core.o
```

这个命令只证明 `.cpp` 能编译，不证明能链接成固件。链接要等 CMake 切换后验证。

## 下一步 3：CMake 只替换 shell，不替换快环

等 C++ core 补完后，把 MotorControl 源列表从：

```cmake
set(CMS32_MOTOR_CONTROL_C_SOURCES
    ${CMS32_MOTOR_C_DIR}/motor_control_c.c
    ${CMS32_MOTOR_C_DIR}/motor_control_current.c
    ${CMS32_MOTOR_C_DIR}/motor_control_encoder.c
    ${CMS32_MOTOR_C_DIR}/motor_control_output.c
    ${CMS32_MOTOR_C_DIR}/motor_control_vf.c
    ${CMS32_MOTOR_C_DIR}/motor_control_watch.c
)
```

改成：

```cmake
set(CMS32_MOTOR_CONTROL_C_SOURCES
    ${CMS32_MOTOR_CPP_DIR}/motor_control_core.cpp
    ${CMS32_MOTOR_C_DIR}/motor_control_current.c
    ${CMS32_MOTOR_C_DIR}/motor_control_encoder.c
    ${CMS32_MOTOR_C_DIR}/motor_control_output.c
    ${CMS32_MOTOR_C_DIR}/motor_control_vf.c
    ${CMS32_MOTOR_C_DIR}/motor_control_watch.c
)
```

同时给 target 增加 C++ 目录和 support 选项：

```cmake
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

注意不要同时编译：

```text
motor_control_c.c
motor_control_core.cpp
```

它们都会定义同一批符号，会重复定义。

## 下一步 4：构建和符号检查

切 CMake 后先构建：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

然后检查对外符号还在：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "MotorControl_|g_motor_"
```

你要看到：

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

再检查没有引入 C++ 运行时特征：

```sh
arm-none-eabi-nm -C build/gcc-debug/cms32foc | rg "__cxa|__gxx|typeinfo|vtable|throw|exception|operator new|operator delete"
```

正常应该无输出。

## 下一步 5：上板只测 shell 行为

这一阶段上板不要急着调 PI，不要改电流环。

只测这些状态：

```text
enable = 0:
  state 应为 idle
  fault_reason 应为 none
  pwm_safe 应为 1

enable = 1, control_mode = speed:
  current_ok 正常时可以进入 closed-loop
  encoder 未初始化时允许第一次进入
  encoder 明确坏角后应进入 fault encoder

enable = 1, control_mode = current:
  current_ok 正常时可以进入 closed-loop
  使用 iq_ref/id_ref，不跑速度 PI 输出作为目标

enable = 1, control_mode = vf:
  不要求 encoder ready
  current_ok 正常时可以进入 closed-loop
  vf_voltage 镜像仍能更新

control_mode = 4/5:
  state 应进入 fault
  fault_reason 应为 unsupported mode
  PWM 应安全关闭
```

如果这些行为通过，说明 C++ shell 替换成功。后面再考虑迁移 PI、速度估算、编码器等内部组件。

## 写程序时的几个具体注意点

### 1. g_motor_command / g_motor_status 是兼容宏

`MotorControl.h` 里当前保留：

```c
#define g_motor_command g_motor_cmd
#define g_motor_status g_motor_watch
```

所以旧写法和新命名暂时都能用。C++ core 里定义真实符号时，以头文件声明为准：

```cpp
volatile MotorControlCommand_t g_motor_cmd = { ... };
volatile MotorControlWatch_t g_motor_watch;
```

不要再新增第三套名字。

### 2. AdcIrqGuard 只包共享状态交换

可以包：

```text
s_mc.enabled
s_mc.mode
s_mc.current_command
s_mc.speed_command
s_mc.vf_command
```

不要包：

```text
sanitize()
curr_set_vf_voltage()
MotorControl_InternalEnterSafeState()
MotorControl_WatchFill()
串口解析
FOC 快环计算
```

临界区越小，ADC 抖动越少。

### 3. 先保留 C 内部状态结构

现在不要把 `MotorControlCState` 改成 C++ class。

原因是这些 C 文件还在共享它：

```text
motor_control_current.c
motor_control_encoder.c
motor_control_output.c
motor_control_vf.c
motor_control_watch.c
```

第四阶段只让 shell 变清楚。等快环模块逐个迁移时，再考虑把对应子状态拆成 C++ 类型。

### 4. Support 不能反向依赖 MotorControl

不要把这些放进 `Firmware/Support`：

```text
ControlMode
ControlFault
CommandSanitizer
SpeedLoopConfig
```

它们属于 MotorControl 业务层，位置应该继续在：

```text
Firmware/MotorControl/Cpp/
```

`Support` 只放无业务含义的工具：

```text
clamp
units
irq_guard
ring_buffer
low_pass
slew_limiter
```

## 本阶段完成标志

当下面全部成立，第四阶段才算完成：

```text
1. CMake 不再编译 motor_control_c.c。
2. CMake 编译 motor_control_core.cpp。
3. motor_control_current/encoder/output/vf/watch 仍保持 C。
4. cms32foc 构建通过。
5. nm 能看到全部 MotorControl_* C ABI。
6. nm 查不到 new/delete/throw/typeinfo/vtable。
7. Ozone 仍能看 g_motor_cmd/g_motor_watch。
8. enable/mode/state/fault/PWM safe 行为和 C 版本一致。
```

完成后再进入下一阶段：

```text
先迁 PI/斜坡/滤波这类纯数学小组件；
再迁速度估算；
最后才评估电流快环和 Board 采样路径。
```
