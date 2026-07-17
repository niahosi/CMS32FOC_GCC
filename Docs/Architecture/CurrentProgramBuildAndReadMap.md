# Current Program Build And Read Map

本文说明当前 `cms32foc` 主固件怎么阅读、CMake 怎么组织编译、参与主固件的 `.c/.h/.S` 之间是什么关系。本文以当前工程状态为准：主固件仍以 C 控制主线为主，C++ 语言已为后续新模块启用；`ScrewAxis` 只保留回零和零位记录，`Jog/Position` 已从主固件删除。

## 先看总图

当前主固件可以按这条线读：

```text
Reset_Handler
  -> SystemInit()
  -> main()
      -> bsp_init()
      -> MotorControl_Init()
      -> ScrewAxis_Init()
      -> bsp_start_adc_sync()
      -> while(1)
          -> ScrewAxis_Run()
          -> MotorControl_ApplyCommand()
          -> MotorControl_RunSlowLoop()
          -> MotorControl_UpdateWatch()

ADC_IRQHandler()
  -> MotorControl_FastLoopFromAdcIrq()
      -> bsp_adc_irq()
      -> Current / Speed / VF fast loop
  -> ScrewAxis_OnAdcSample()
```

一句话：`main.c` 调度慢环和应用层，`ADC_IRQHandler()` 触发控制快环，Board 层负责真实 PWM/ADC/MA600，MotorControl 层负责 FOC 和状态机。

## CMake 管理方式

顶层 `CMakeLists.txt` 当前声明：

```cmake
project(CMS32FOC_GCC C CXX ASM)
```

也就是说当前工程已经启用 C++ 编译器，但主固件还没有链接新的 C++ 源码。旧冻结 C++ 控制层已删除，后续 C++ 代码应从新的模块和 target 开始。

### 公共编译选项

`cms32_project_options` 是 `INTERFACE` 库，不生成 `.a`，只传播编译参数和头文件路径。

它提供：

```text
-mcpu=cortex-m0plus
-mthumb
-ffunction-sections
-fdata-sections
-fdiagnostics-color=always
-Wall
-Wextra
```

其中 `-ffunction-sections` 和 `-fdata-sections` 配合链接参数 `--gc-sections`，让未引用函数和数据可以被链接器删除。

### 固件链接配置

`cms32_configure_firmware(target)` 给每个可烧录固件挂接：

```text
Firmware/Drivers/CMS32M6510/cms32m6510_flash.ld
-Wl,--gc-sections
-Wl,-Map=<target>.map
--specs=nano.specs
--specs=nosys.specs
```

构建后生成：

```text
<target>
<target>.hex
<target>.bin
<target>.map
```

并打印 Flash/RAM 报告。真正烧进 Flash 的是 `.text + .data`，不是带 DWARF 调试信息的 ELF 文件大小。

## CMake Target 图

主固件 target：

```text
cms32foc
  links -> cms32_motor_control_c
              links -> cms32_bsp
                          links -> cms32_platform
                          links -> cms32_vendor_drivers
              links -> cms32_foc_algorithm
```

展开后：

```text
cms32foc
├── Firmware/App/main.c
├── Firmware/App/screw_axis.c
├── Firmware/Drivers/CMS32M6510/startup_CMS32M6510_gcc.S
├── cms32_motor_control_c
│   ├── Firmware/MotorControl/C/motor_control_c.c
│   ├── Firmware/MotorControl/C/motor_control_current.c
│   ├── Firmware/MotorControl/C/motor_control_encoder.c
│   ├── Firmware/MotorControl/C/motor_control_output.c
│   ├── Firmware/MotorControl/C/motor_control_vf.c
│   └── Firmware/MotorControl/C/motor_control_watch.c
├── cms32_bsp
│   ├── Firmware/Board/Src/foc_bsp.c
│   ├── Firmware/Board/Src/foc_curr.c
│   ├── Firmware/Board/Src/foc_ma600.c
│   └── Firmware/Board/Src/foc_pwm.c
├── cms32_foc_algorithm
│   └── Firmware/MotorControl/Algorithm/foc_math.c
├── cms32_platform
│   ├── Firmware/Drivers/CMS32M6510/cms32m6510_platform.c
│   └── Firmware/ThirdParty/Cmsemicon/CMS32M65xx/Device/Source/system_CMS32M6510.c
└── cms32_vendor_drivers
    ├── adc.c
    ├── adcldo.c
    ├── cgc.c
    ├── delay.c
    ├── epwm.c
    ├── gpio.c
    ├── pga.c
    └── ssp.c
```

手动诊断固件：

```text
cms32_board_watch_test      EXCLUDE_FROM_ALL
```

它不会被默认 `all` 构建打进 `cms32foc`，需要显式指定 target。

## 主固件源码依赖关系

### App 层

```text
Firmware/App/main.c
├── MotorControl.h
├── cms32m6510_platform.h
├── foc_bsp.h
└── screw_axis.h
```

`main.c` 只做启动和调度，不直接实现控制算法。

```text
Firmware/App/screw_axis.c
├── screw_axis.h
├── Config.h
└── MotorControl.h
```

`screw_axis.c` 当前只负责：

```text
回零命令 g_screw_home_cmd
回零观察 g_screw_home_watch
回零状态机
记录 zero_encoder_pos
根据 encoder_pos 更新 pos_counts
向 g_motor_cmd 写速度模式命令
```

它不再包含 `Jog/Position` 调试逻辑。

### MotorControl 公共 API

```text
Firmware/MotorControl/Inc/MotorControl.h
└── <stdint.h>
```

这是控制层 C ABI。主循环、应用层和后续 C++ 包装层都应通过它进入控制层。

它暴露：

```text
g_motor_cmd
g_motor_watch
MotorControl_Init()
MotorControl_ApplyCommand()
MotorControl_RunSlowLoop()
MotorControl_FastLoopFromAdcIrq()
MotorControl_GetWatch()
MotorControl_UpdateWatch()
```

### MotorControl 内部层

```text
motor_control_internal.h
├── Config.h
├── MotorControl.h
└── foc_math.h
```

`motor_control_internal.h` 是 C 控制层内部共享状态定义，核心是：

```text
MotorControlCState
```

除 `motor_control_c.c` 外，Current、Encoder、Output、VF、Watch 都通过 `MotorControlCState*` 操作同一份控制状态。

直接 include 图：

```text
motor_control_c.c
├── MotorControl.h
├── motor_control_internal.h
├── motor_control_vf.h
├── foc_bsp.h
├── foc_curr.h
├── foc_pwm.h
└── CMS32M6510.h

motor_control_current.c
├── motor_control_internal.h
└── foc_curr.h

motor_control_encoder.c
├── motor_control_internal.h
└── foc_bsp.h

motor_control_output.c
├── motor_control_internal.h
└── foc_pwm.h

motor_control_vf.c
├── motor_control_vf.h
└── foc_curr.h

motor_control_watch.c
├── motor_control_internal.h
├── motor_control_vf.h
├── foc_curr.h
└── foc_pwm.h
```

### FOC 算法层

```text
foc_math.c
└── foc_math.h

foc_math.h
└── <stdint.h>
```

算法层只做纯计算：

```text
sin/cos 查表
clamp
Clarke
Park
PI
dq 电压矢量限幅
InvPark
SVPWM
```

它不访问寄存器，不依赖 Board 层。

### Board 层

```text
foc_bsp.c
├── foc_bsp.h
├── foc_curr.h
├── foc_ma600.h
├── foc_pwm.h
├── CMS32M6510.h
├── delay.h
└── gpio.h

foc_curr.c
├── foc_curr.h
├── foc_pwm.h
├── adc.h
├── adcldo.h
├── cgc.h
├── gpio.h
└── pga.h

foc_ma600.c
├── foc_ma600.h
├── Config.h
├── cgc.h
├── common.h
├── delay.h
├── gpio.h
└── ssp.h

foc_pwm.c
├── foc_pwm.h
├── foc_curr.h
├── cgc.h
├── common.h
├── epwm.h
└── gpio.h
```

Board 层负责所有寄存器相关工作，控制层不直接访问寄存器。

### Config 入口

```text
Config.h
├── BoardConfig.h
└── TuneConfig.h
```

`BoardConfig.h` 放硬件基线：

```text
PWM 频率/周期
ADC/PGA 换算
相序和电流方向
电机极对数
MA600 SPI 配置
电角零位
```

`TuneConfig.h` 放当前调试参数：

```text
电流采样窗口
坏角过滤
电流环参数
速度环参数
VF 开环参数
```

## FOC 调用链

### 主循环慢链路

```text
main()
  -> ScrewAxis_Run()
      -> home_update()
          -> 写 g_motor_cmd.control_mode = speed
          -> 写 g_motor_cmd.speed_ref_rpm
          -> 写 g_motor_cmd.iq_limit

  -> MotorControl_ApplyCommand(&g_motor_cmd)
      -> 复制 volatile 命令到 s_mc.command
      -> 限幅 id/iq/speed/vf/PI 参数
      -> 模式切换时 reset PI、速度估算、VF

  -> MotorControl_RunSlowLoop()
      -> 读取慢速电流快照
      -> 检查 current_ok
      -> 检查 encoder_ok
      -> 设置 state = IDLE / CLOSED_LOOP / FAULT

  -> MotorControl_UpdateWatch(&g_motor_watch)
      -> 从 s_mc 和 Board 层填充 Ozone watch
```

慢链路不做 Clarke/Park/PI/SVPWM。慢链路只负责命令、状态和观察。

### ADC 快环链路

```text
ADC_IRQHandler()
  -> MotorControl_FastLoopFromAdcIrq()
      -> bsp_adc_irq()
          -> curr_irq()
              -> 读 ADC
              -> 当前采样窗口解析
              -> 两相采样/三相重构
              -> 更新 curr_u/v/w
```

如果本次 ADC 形成有效采样：

```text
MotorControl_FastLoopFromAdcIrq()
  -> mode == Current
      -> MotorControl_CurrentRunFastLoop(speed_mode = 0)
  -> mode == Speed
      -> MotorControl_CurrentRunFastLoop(speed_mode = 1)
  -> mode == VF
      -> MotorControlVf_RunFastLoop()
```

### Current/Speed 快环

```text
MotorControl_CurrentRunFastLoop()
  -> curr_u/v/w()
  -> MotorControl_InternalCurrentOk()
  -> MotorControl_InternalUpdateEncoderAngle()
      -> bsp_update_angle_fast()
      -> ma600_update_fast()
      -> electrical_from_raw()
      -> bad angle reject / retry / hold

  -> 每 MC_SPEED_SAMPLE_DIV 次:
      -> MotorControl_InternalUpdateEncoderSpeed()
      -> speed_mode != 0:
          -> update_speed_loop()
              -> rpm/count 换算
              -> speed_ref 斜坡
              -> speed PI
              -> speed_iq_ref

  -> foc_clarke_3phase()
  -> foc_park()
  -> foc_pi_set_gains()
  -> foc_pi_update() for d/q
  -> MotorControl_InternalApplyVoltageVector()
      -> foc_limit_dq()
      -> foc_inv_park()
      -> foc_svpwm()
      -> pwm_set_duty()
      -> pwm_enable()
```

Current 模式直接使用 `g_motor_cmd.iq_ref`。Speed 模式先用速度环得到 `speed_iq_ref`，再作为 q 轴电流给定。

### VF 快环

```text
MotorControlVf_RunFastLoop()
  -> curr_u/v/w()
  -> MotorControl_InternalCurrentOk()
  -> 检查 open_loop_timeout
  -> update_open_loop_theta()
  -> MotorControl_InternalApplyVoltageVector(vd=0, vq=vf_voltage, theta=open_loop_theta)
  -> 定期读 MA600 角度，速度观察由 raw 差分得到
```

VF 的角度来自内部开环积分，不来自编码器。编码器在 VF 下主要用于看转子是否跟上。

## Board 层细节阅读顺序

建议不要先从原厂库读起。按当前主链路读：

```text
1. foc_bsp.c
2. foc_pwm.c
3. foc_curr.c
4. foc_ma600.c
5. 原厂 adc/epwm/gpio/pga/ssp 驱动
```

`foc_curr.c` 是 Board 层最重的文件。先抓住这条线：

```text
pwm_set_duty()
  -> curr_sync_timing()
      -> 根据 duty 选择低边采样窗口
      -> 设置 ADC 触发 tick

ADC IRQ
  -> curr_irq()
      -> 读两相
      -> 判断双点采样差值
      -> 重构第三相
      -> 相序/符号映射到逻辑 U/V/W
```

## 当前不要从这些地方开始

```text
Firmware/FrozenDiagnostics/**
Reference/**
build/**
```

冻结诊断和参考目录可能有参考价值，但不参与当前 `cms32foc` 主固件。

## 验证命令

主固件 Debug 构建：

```sh
cmake --build build/gcc-debug --target cms32foc
```

体积更接近发布状态的构建：

```sh
cmake -S . -B build/gcc-minsize -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake \
  -DCMAKE_BUILD_TYPE=MinSizeRel
cmake --build build/gcc-minsize --target cms32foc
```

查看符号是否进入主固件：

```sh
arm-none-eabi-nm -g build/gcc-debug/cms32foc | rg "g_screw|ScrewAxis|Jog|Position"
```

当前期望只看到：

```text
ScrewAxis_Init
ScrewAxis_Run
ScrewAxis_OnAdcSample
g_screw_home_cmd
g_screw_home_watch
```
