# Control Layer Policy

当前规则很简单：`cms32foc` 主固件只保留 C 主线，目标是稳定 Current、Speed 和 VF。

## Active Path

```text
main.c
  -> MotorControl_ApplyCommand()
  -> MotorControl_RunSlowLoop()
ADC_IRQHandler
  -> MotorControl_FastLoopFromAdcIrq()
       -> Current / Speed / VF
```

主控制层只通过 `foc_bsp.h` 使用板级能力，只通过 `foc_math.h` 使用 FOC 算法。寄存器细节留在 `Firmware/Board/Src`。

## Frozen Paths

以下内容不参与 `cms32foc` 默认构建：

- `Firmware/MotorControl/Src/*.cpp` 旧 C++ 控制层。
- Align 和 EncoderVoltage，冻结在 `Firmware/FrozenDiagnostics/MotorControl/`。
- MA600 在线 BCT/CORR/NVM 调参，冻结在 `Firmware/FrozenDiagnostics/Board/`。
- BoardWatch 和 StartupSmoke 诊断固件，保留为手动 target。

## Re-enable Rule

恢复任何冻结路径时，必须单独开一轮：

1. 先说明要恢复的目标和原因。
2. 同步接口和 CMake target。
3. 确认不会改变 Current/Speed/VF 主线行为。
4. 构建并检查 `cms32foc` 符号，避免辅助代码重新混进主固件。

当前运行状态和函数说明以 `Docs/Architecture/ActiveControlChain.md` 为准。
