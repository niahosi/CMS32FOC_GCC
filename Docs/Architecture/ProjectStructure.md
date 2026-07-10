# Project Structure

本文记录当前工程目录分工。当前 `cms32foc` 主固件只使用 C/ASM 主线；C++ 控制层源码保留为冻结参考，不参与默认构建。

## Root Layout

```text
CMS32FOC_GCC/
├── Firmware/
├── Tools/
├── Docs/
├── Reference/
├── cmake/
├── CMakeLists.txt
├── CMakePresets.json
└── README.md
```

## Firmware Layout

```text
Firmware/
├── App/
├── Board/
├── Drivers/
├── FrozenDiagnostics/
├── MotorControl/
├── Tests/
└── ThirdParty/
```

| 路径 | 当前用途 |
| --- | --- |
| `Firmware/App` | `main.c`、全局 Ozone 命令/watch、ADC IRQ 入口 |
| `Firmware/Board` | BSP、PWM、ADC/PGA 电流采样、MA600、板级安全态 |
| `Firmware/Board/Config` | 当前硬件和调试参数：`BoardConfig.h`、`TuneConfig.h` |
| `Firmware/Drivers/CMS32M6510` | 启动文件、链接脚本、平台 glue、内存布局 |
| `Firmware/FrozenDiagnostics` | 已从主固件移出的 Align、EncoderVoltage、MA600 在线调参等辅助代码 |
| `Firmware/MotorControl/C` | 当前 C 主控制层：外壳、Current/Speed、编码器、输出、watch、VF |
| `Firmware/MotorControl/Algorithm` | Clarke/Park/PI/SVPWM 等纯 FOC 数学 |
| `Firmware/MotorControl/Inc` | 公共控制 API；其中 `.hpp/.cpp` C++ 层冻结 |
| `Firmware/Tests` | 可手动构建的诊断固件，默认 all build 不构建 |
| `Firmware/ThirdParty` | CMS32 原厂 CMSIS/Driver 包，视作只读输入 |

## Active Build

当前主固件链接边界：

```text
cms32foc
  -> cms32_motor_control_c
       -> motor_control_c.c
       -> motor_control_current.c
       -> motor_control_encoder.c
       -> motor_control_output.c
       -> motor_control_vf.c
       -> motor_control_watch.c
       -> cms32_bsp
       -> cms32_foc_algorithm
```

`cms32_board_watch_test` 和 `cms32_startup_smoke_test` 使用 `EXCLUDE_FROM_ALL`，需要指定 target 才构建。

## Rules

- 主固件状态以 `Docs/Architecture/ActiveControlChain.md` 为准。
- 不把冻结诊断代码加入默认 CMake target。
- 不把 `Firmware/MotorControl/Src/*.cpp` 重新接入主固件，除非单独做 C++ 恢复计划。
- 不在控制层直接访问寄存器；控制层通过 `foc_bsp.h` 和 `foc_math.h` 使用板级能力。
- 不编辑 `Firmware/ThirdParty`，除非没有项目侧替代方案。
- 生成文件只放在 `build/`。
