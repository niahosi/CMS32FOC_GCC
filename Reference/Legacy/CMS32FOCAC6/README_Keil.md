# CMS32FOCAC6 - 电机控制项目

基于 CMS32M6513AGE40NB 的 FOC（磁场定向控制）电机控制项目。

## 项目概述

本项目实现空心杯无刷电机的 FOC 控制，使用 CMS32M6513AGE40NB 芯片（QFN40封装，集成 3P3N 功率驱动）。

### 硬件配置
- **主控芯片**: CMS32M6513AGE40NB（Arm Cortex-M0+，64 MHz）
- **电机**: 空心杯无刷电机 + 滚珠丝杠负载
- **位置传感器**: MA600 磁性角度传感器 + 侧边空心磁环（4对极）
- **电流采样**: 三电阻低边电流采样（80 mΩ）
- **PWM 频率**: 20 kHz EPWM
- **ADC 触发**: EPWM CMP0 同步触发

## 当前项目状态

### 已完成
- ✅ Keil ARMCC5 全量构建（0 Error，0 Warning）
- ✅ 系统时钟确认 64 MHz
- ✅ MA600 SPI 角度读取验证
- ✅ 三相 ADC/PGA 基础采样
- ✅ 电流零漂校准（U/V/W: 2034/2041/2038）
- ✅ EPWM 三相 20 kHz PWM 输出
- ✅ EPWM CMP0 触发 ADC 同步采样
- ✅ ADC_IRQHandler -> Board_HandleAdcIrq -> Motor_FastLoop 快环入口

### 进行中
- 🔄 FOC 快环骨架搭建
- 🔄 角度处理和速度估算
- 🔄 Clarke/Park 变换
- 🔄 电流环 PI 控制

### 待实现
- ⏳ SVPWM 或零序注入
- ⏳ 速度环控制
- ⏳ 丝杠位置环控制
- ⏳ 弱磁控制

## 项目结构

```
CMS32FOCAC6/
├── User/
│   ├── App/                # 应用层：main、调试入口、测试指令源
│   ├── Board/              # 板级硬件适配层：MA600、ADC/PGA、EPWM
│   ├── Config/             # 可调参数入口：电机、控制环、保护、PWM、测试默认值
│   │   └── Config.h
│   └── Motor/              # 电机控制算法层：状态机、FOC、电流环、速度环
├── Vendor/                 # 厂商文件（尽量不修改）
│   ├── Cmsemicon/
│   │   └── CMS32M65xx/     # 民芯半导体驱动库
│   └── SeggerRtt/          # SEGGER RTT 调试输出
├── Docs/                   # 中文文档
│   ├── README.md           # 文档入口
│   ├── control-framework-roadmap.md
│   ├── learning-and-debug-record.md
│   ├── project-organization.md
│   └── ...
├── Objects/                # 编译输出
├── Listings/               # 链接输出
└── DebugFile/              # 调试文件
```

## 开发环境

- **IDE**: Keil MDK
- **编译器**: ARMCC5（ARM Compiler 5.06）
- **调试器**: J-Link
- **目标芯片**: CMS32M6513AGE40NB

## 快速开始

### 1. 打开工程
使用 Keil MDK 打开 `CMS32FOCAC6.uvprojx` 工程文件。

### 2. 编译
点击 Keil 中的 "Build" 按钮，确保 0 Error，0 Warning。

### 3. 下载调试
连接 J-Link 调试器，点击 "Download" 或 "Debug" 按钮。

### 4. 调试变量
在 `main.c` 中当前主要可写调试变量：
```c
volatile uint8_t debug_motor_enable;
volatile uint8_t debug_ctrl_mode;
volatile int16_t debug_id_ref;
volatile int16_t debug_iq_ref;
volatile int32_t debug_speed_ref;
volatile int16_t debug_iq_limit;
volatile int32_t debug_elec_zero_trim;
volatile uint8_t debug_diag_cmd;
```

## 关键文件说明

### User/App/Src/main.c
主程序入口，包含：
- `main()` 函数：初始化 Board 和 Motor，主循环调用 Motor_TASK
- `ADC_IRQHandler()`：ADC 中断服务程序，调用 Motor_FastLoop

### User/Config/Config.h
当前工程可调参数入口，包含电机/编码器参数、控制环参数、保护阈值、PWM 参数和测试默认值。

### User/Motor/Src/Motor.c
电机控制核心，包含：
- 电机状态机（IDLE -> SENSOR_CHECK -> ALIGN -> CLOSED_LOOP -> FAULT）
- Motor_FastLoop()：快环入口，后续 FOC 算法实现位置
- 安全检查和保护逻辑

### Board/Src/Board.c
板级初始化，包含：
- Board_Init()：时钟、GPIO、MA600 SPI、ADC/PGA、EPWM、电流零漂校准
- Board_HandleAdcIrq()：ADC 中断处理

## 硬件约束

### 保护规则
- PWM 极性、死区、inactive 电平、刹车链路和电流零漂确认前，不编写会实际励磁的逻辑
- 未实测的硬件量必须标为待确认
- EPWM/ADC 中断路径禁止阻塞式 UART 打印、动态内存和耗时格式化
- 不要禁用 P06/P07 的 SWD

### 引脚分配
- **P06/P07**: SWD 调试接口
- **P02/P03/P04/P05**: MA600 SPI 接口
- **P00/P24/P26**: 电流采样脚（不要随意改成数字 BKIN）
- **P20**: NTC 温度采样
- **P21/P22/P23**: 霍尔传感器输入

## 参考工程

- **acc 参考工程**: `D:\wjw\MotorRec\01_ELA_ACC`
  - 用于参考 MA600 SPI 时序、FOC 调度、环路组织、启动流程
  - 当说"按 acc 处理"时，沿用参考工程组织方式，适配 CMS32M65xx 外设接口

## 文档

详细文档请查看 `Docs/` 目录：
- [文档入口](Docs/README.md)
- [控制框架路线图](Docs/control-framework-roadmap.md)
- [学习与排错记录](Docs/learning-and-debug-record.md)
- [项目组织规划](Docs/project-organization.md)

## 版本历史

- **v0.1.0** (2026-05-28): 初始版本，完成基础外设验证，搭建 FOC 快环骨架

## 许可证

本项目为私有项目，仅用于个人学习和开发。

## 联系方式

如有问题，请查看 `Docs/learning-and-debug-record.md` 中的学习记录。
