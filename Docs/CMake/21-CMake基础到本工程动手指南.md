# CMake 基础到本工程动手指南

本文从零解释当前工程的 CMake。目标不是把所有 CMake 语法讲完，而是让你能看懂、修改、验证这个工程的构建结构。

当前工程的主要文件：

```text
CMakeLists.txt
CMakePresets.json
cmake/arm-none-eabi-gcc.cmake
cmake/firmware_output.cmake
cmake/print_firmware_size.cmake
```

你平时最常用的命令：

```sh
cmake --preset gcc-debug
cmake --build --preset gcc-debug --target cms32foc
```

要记住一句话：

```text
cmake --preset 只是生成构建系统；
cmake --build 才是真正编译。
```

## 1. CMake 到底在做什么

你可以把 CMake 分成两层：

```text
CMake:
  读取 CMakeLists.txt，生成 Ninja/Makefile 工程。

Ninja:
  真正调用 arm-none-eabi-gcc / arm-none-eabi-g++ 编译链接。
```

所以 CMake 不是编译器。它更像“工程生成器”。

当前 preset 使用 Ninja：

```json
"generator": "Ninja"
```

完整流程：

```text
1. 你写 CMakeLists.txt
2. cmake --preset gcc-debug
3. CMake 生成 build/gcc-debug/build.ninja
4. cmake --build --preset gcc-debug --target cms32foc
5. Ninja 调 arm-none-eabi-gcc/g++
6. 链接生成 cms32foc
7. objcopy 生成 hex/bin
8. size 脚本打印 Flash/RAM 报告
```

## 2. Configure 和 Build 的区别

### 2.1 Configure

命令：

```sh
cmake --preset gcc-debug
```

它做的是：

```text
读取 CMakePresets.json
读取 CMakeLists.txt
读取 toolchain 文件
检查编译器
生成 build/gcc-debug/
生成 compile_commands.json
生成 Ninja 构建文件
```

它不等于编译通过。

如果你只跑了：

```sh
cmake --preset gcc-debug
```

只能说：

```text
CMake 配置通过。
```

不能说：

```text
固件编译通过。
```

### 2.2 Build

命令：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

它才会真正编译：

```text
.c   -> arm-none-eabi-gcc
.cpp -> arm-none-eabi-g++
.S   -> arm-none-eabi-gcc
.o   -> 静态库 / 最终 ELF
ELF  -> hex/bin
```

所以验证代码改动时，至少要跑：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

## 3. CMakePresets.json

当前文件：

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "gcc-debug",
      "displayName": "GCC Debug",
      "generator": "Ninja",
      "binaryDir": "${sourceDir}/build/gcc-debug",
      "toolchainFile": "${sourceDir}/cmake/arm-none-eabi-gcc.cmake",
      "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug"
      }
    }
  ],
  "buildPresets": [
    {
      "name": "gcc-debug",
      "configurePreset": "gcc-debug"
    }
  ]
}
```

它定义了一个配置：

```text
名字:
  gcc-debug

生成器:
  Ninja

构建目录:
  build/gcc-debug

交叉编译工具链:
  cmake/arm-none-eabi-gcc.cmake

构建类型:
  Debug
```

你输入：

```sh
cmake --preset gcc-debug
```

就等价于手写一长串：

```sh
cmake -S . -B build/gcc-debug \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake \
  -DCMAKE_BUILD_TYPE=Debug
```

preset 的好处是：

```text
不用每次记一长串参数
VS Code CMake Tools 能识别
团队里每个人用同一套构建目录和工具链
```

## 4. Toolchain 文件

当前文件：

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
```

这个文件告诉 CMake：

```text
这不是给你的 Linux 主机编译程序。
这是给裸机 ARM MCU 编译固件。
```

每行含义：

```text
CMAKE_SYSTEM_NAME Generic:
  目标系统不是 Linux/Windows/macOS，而是裸机。

CMAKE_SYSTEM_PROCESSOR arm:
  目标处理器是 ARM。

CMAKE_C_COMPILER:
  C 文件用 arm-none-eabi-gcc。

CMAKE_CXX_COMPILER:
  C++ 文件用 arm-none-eabi-g++。

CMAKE_ASM_COMPILER:
  汇编启动文件也交给 arm-none-eabi-gcc。

CMAKE_OBJCOPY:
  用来把 ELF 转成 hex/bin。

CMAKE_SIZE:
  用来统计 text/data/bss。

CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY:
  CMake 检查编译器时不要尝试链接可执行文件。
```

最后一项很重要。裸机工程没有普通操作系统入口，CMake 如果尝试链接一个 PC 风格测试程序，可能会失败。

## 5. CMakeLists.txt 顶部

当前开头：

```cmake
cmake_minimum_required(VERSION 3.20)

project(CMS32FOC_GCC C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

含义：

```text
cmake_minimum_required:
  要求 CMake 至少 3.20。

project(... C CXX ASM):
  本工程启用 C、C++、汇编三种语言。

CMAKE_EXPORT_COMPILE_COMMANDS:
  生成 compile_commands.json，clangd 靠它知道每个文件怎么编译。
```

这也是为什么 `.vscode/settings.json` 里 clangd 指向：

```text
build/gcc-debug
```

因为 `compile_commands.json` 在这个目录里。

## 6. C 和 C++ 标准

当前配置：

```cmake
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

含义：

```text
C 文件按 C11 编译。
C++ 文件按 C++17 编译。
不使用 GNU 扩展标准名，比如 gnu11 / gnu++17。
```

这和当前 C/C++ 混编策略一致：

```text
C ABI 稳定
C++ 内部逐步使用 C++17
```

## 7. 路径变量

当前 CMake 里没有到处硬写路径，而是先集中定义：

```cmake
set(CMS32_FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Firmware)
set(CMS32_VENDOR_DIR ${CMS32_FIRMWARE_DIR}/ThirdParty/Cmsemicon/CMS32M65xx)
set(CMS32_PLATFORM_DIR ${CMS32_FIRMWARE_DIR}/Drivers/CMS32M6510)

set(CMS32_APP_DIR ${CMS32_FIRMWARE_DIR}/App)
set(CMS32_BOARD_DIR ${CMS32_FIRMWARE_DIR}/Board)
set(CMS32_BOARD_CONFIG_DIR ${CMS32_BOARD_DIR}/Config)
set(CMS32_BOARD_INC_DIR ${CMS32_BOARD_DIR}/Inc)
set(CMS32_BOARD_SRC_DIR ${CMS32_BOARD_DIR}/Src)
set(CMS32_MOTOR_DIR ${CMS32_FIRMWARE_DIR}/MotorControl)
set(CMS32_MOTOR_ALGORITHM_DIR ${CMS32_MOTOR_DIR}/Algorithm)
set(CMS32_MOTOR_C_DIR ${CMS32_MOTOR_DIR}/C)
set(CMS32_MOTOR_CPP_DIR ${CMS32_MOTOR_DIR}/Cpp)
set(CMS32_MOTOR_INC_DIR ${CMS32_MOTOR_DIR}/Inc)
set(CMS32_SUPPORT_DIR ${CMS32_FIRMWARE_DIR}/Support)
```

这样做的好处：

```text
后面 target 里不用写长路径
目录重构时只改一处
CMakeLists.txt 更容易读
```

你后面新增目录时，也应该先加路径变量，例如：

```cmake
set(CMS32_COMM_DIR ${CMS32_FIRMWARE_DIR}/Comm)
set(CMS32_COMM_CPP_DIR ${CMS32_COMM_DIR}/Cpp)
set(CMS32_COMM_INC_DIR ${CMS32_COMM_DIR}/Inc)
```

## 8. 编译选项 target

当前有两个重要的 interface target。

### 8.1 cms32_project_options

```cmake
add_library(cms32_project_options INTERFACE)
target_compile_options(cms32_project_options INTERFACE
    ${CMS32_COMMON_COMPILE_OPTIONS}
)
target_include_directories(cms32_project_options INTERFACE
    ${CMS32_VENDOR_DIR}/CMSIS/Include
    ${CMS32_VENDOR_DIR}/Device/Include
    ${CMS32_VENDOR_DIR}/Driver/inc
)
```

`INTERFACE` 的意思是：

```text
它自己不编译源码。
它只携带编译选项、include 路径、链接依赖。
谁 link 它，谁获得这些设置。
```

当前它携带：

```text
-mcpu=cortex-m0plus
-mthumb
-ffunction-sections
-fdata-sections
-Wall
-Wextra
CMSIS/vendor include 路径
```

为什么不用全局 `include_directories()`？

```text
全局 include 会污染所有 target。
target 方式能看清谁依赖谁。
```

### 8.2 cms32_cpp_options

```cmake
add_library(cms32_cpp_options INTERFACE)
target_compile_options(cms32_cpp_options INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)
target_link_libraries(cms32_cpp_options INTERFACE
    cms32_project_options
)
```

这里用到了 generator expression：

```cmake
$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
```

意思是：

```text
只有 C++ 文件才加 -fno-exceptions。
C 文件不加这个选项。
```

这些 C++ 选项是嵌入式 C++ 的核心约束：

```text
不使用异常
不使用 RTTI
不使用线程安全静态局部初始化
不注册全局析构
```

## 9. Target 是什么

CMake 里最重要的概念是 target。

当前工程主要 target：

```text
cms32_project_options     INTERFACE 编译公共选项
cms32_cpp_options         INTERFACE C++ 限制选项
cms32_support_cpp         INTERFACE header-only support
cms32_vendor_drivers      STATIC vendor driver 静态库
cms32_platform            STATIC 平台/启动相关静态库
cms32_bsp                 STATIC Board 层静态库
cms32_foc_algorithm       STATIC FOC 数学静态库
cms32_motor_control     STATIC MotorControl 静态库
cms32foc                  EXECUTABLE 最终固件
```

target 大概分三种：

```text
INTERFACE:
  没有源码，只传播配置。

STATIC:
  有源码，编译成 .a 静态库。

EXECUTABLE:
  最终链接成可执行文件，这里是固件 ELF。
```

## 10. PUBLIC / PRIVATE / INTERFACE

这是你后面最需要掌握的。

### 10.1 PRIVATE

```cmake
target_include_directories(foo PRIVATE dir_a)
```

意思：

```text
foo 自己编译时可以 include dir_a。
链接 foo 的下游 target 不会继承 dir_a。
```

适合：

```text
模块内部头文件
.cpp 私有头
C internal 目录
MotorControl/Cpp 目录
```

### 10.2 PUBLIC

```cmake
target_include_directories(foo PUBLIC dir_api)
```

意思：

```text
foo 自己编译时可以 include dir_api。
链接 foo 的下游 target 也可以 include dir_api。
```

适合：

```text
模块对外 API 头文件目录
```

例如：

```text
MotorControl/Inc
Board/Inc
```

### 10.3 INTERFACE

```cmake
target_include_directories(foo INTERFACE dir_header_only)
```

意思：

```text
foo 自己没有源码，不需要编译。
链接 foo 的下游 target 获得 dir_header_only。
```

适合：

```text
header-only 库
纯编译选项 target
```

例如当前：

```text
cms32_support_cpp
cms32_project_options
cms32_cpp_options
```

### 10.4 一句话判断

```text
只给自己用: PRIVATE
自己和下游都要用: PUBLIC
自己没源码，只给下游用: INTERFACE
```

## 11. 当前 target 结构

### 11.1 Vendor drivers

```cmake
add_library(cms32_vendor_drivers STATIC
    ${CMS32_VENDOR_DRIVER_SOURCES}
)
target_link_libraries(cms32_vendor_drivers PUBLIC
    cms32_project_options
)
```

vendor driver 是静态库：

```text
adc.c
gpio.c
epwm.c
ssp.c
uart.c
...
```

它 link `cms32_project_options`，所以 vendor 源码也获得：

```text
CPU 选项
CMSIS/vendor include
-Wall / -Wextra
```

### 11.2 Platform

```cmake
add_library(cms32_platform STATIC
    ${CMS32_PLATFORM_DIR}/cms32m6510_platform.c
    ${CMS32_VENDOR_DIR}/Device/Source/system_CMS32M6510.c
)
target_include_directories(cms32_platform PUBLIC
    ${CMS32_PLATFORM_DIR}
    ${CMS32_PLATFORM_DIR}/Memory/Inc
)
target_link_libraries(cms32_platform PUBLIC
    cms32_project_options
)
```

它提供：

```text
平台初始化
system_CMS32M6510.c
平台头文件
linker script 相关头目录
```

### 11.3 Board BSP

```cmake
add_library(cms32_bsp STATIC
    ${CMS32_BOARD_SOURCES}
)
target_include_directories(cms32_bsp PUBLIC
    ${CMS32_BOARD_INC_DIR}
    ${CMS32_BOARD_CONFIG_DIR}
)
target_link_libraries(cms32_bsp PUBLIC
    cms32_platform
    cms32_vendor_drivers
)
```

Board 层包含：

```text
foc_bsp.c
foc_curr.c
foc_ma600.c
foc_pwm.c
board_uart.c
```

它的 public include 合理，因为 App 和 MotorControl 都会 include：

```text
foc_bsp.h
foc_curr.h
foc_pwm.h
board_uart.h
BoardConfig.h / TuneConfig.h
```

后续如果 Board 内部有私有头，可以再加：

```cmake
target_include_directories(cms32_bsp
    PUBLIC
        ${CMS32_BOARD_INC_DIR}
        ${CMS32_BOARD_CONFIG_DIR}
    PRIVATE
        ${CMS32_BOARD_SRC_DIR}
)
```

### 11.4 FOC Algorithm

```cmake
add_library(cms32_foc_algorithm STATIC
    ${CMS32_MOTOR_ALGORITHM_DIR}/foc_math.c
)
target_include_directories(cms32_foc_algorithm PUBLIC
    ${CMS32_MOTOR_ALGORITHM_DIR}
    ${CMS32_BOARD_CONFIG_DIR}
)
target_link_libraries(cms32_foc_algorithm PUBLIC
    cms32_project_options
)
```

这里 `foc_math.h` 目前给 MotorControl 使用，所以 `CMS32_MOTOR_ALGORITHM_DIR` 是 PUBLIC。

如果以后 `foc_math` 完全变成 MotorControl 私有实现，可以考虑收窄。但现在不急。

### 11.5 MotorControl

当前：

```cmake
target_include_directories(cms32_motor_control PUBLIC
    ${CMS32_MOTOR_C_DIR}
    ${CMS32_MOTOR_CPP_DIR}
    ${CMS32_MOTOR_INC_DIR}
)
```

这能工作，但分层偏宽。

它等于让 App 也能 include：

```text
motor_control_state.h
motor_control_internal.h
config.hpp
encoder_math.hpp
current.cpp 内部相关头
```

长期不理想。

更清楚的方向：

```cmake
target_include_directories(cms32_motor_control
    PUBLIC
        ${CMS32_MOTOR_INC_DIR}
    PRIVATE
        ${CMS32_MOTOR_C_DIR}
        ${CMS32_MOTOR_CPP_DIR}
)
```

含义：

```text
对外只公开 MotorControl.h。
内部才能 include motor_control_internal.h。
App 如果确实需要读取 Ozone 同款 `g_mc_*` 状态，可以先只 include 更窄的
motor_control_state.h。
内部才能 include config.hpp / speed_math.hpp 等。
```

这是 C++ 分层的关键动作。

## 12. include 可见性怎么整理

建议分两步做，不要一次大改。

### 12.1 第一步：只收 MotorControl include

把当前：

```cmake
target_include_directories(cms32_motor_control PUBLIC
    ${CMS32_MOTOR_C_DIR}
    ${CMS32_MOTOR_CPP_DIR}
    ${CMS32_MOTOR_INC_DIR}
)
```

改成：

```cmake
target_include_directories(cms32_motor_control
    PUBLIC
        ${CMS32_MOTOR_INC_DIR}
    PRIVATE
        ${CMS32_MOTOR_C_DIR}
        ${CMS32_MOTOR_CPP_DIR}
)
```

然后跑：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

如果通过，说明 App 没有直接依赖 MotorControl 内部头。

当前源码检查结果：

```text
Firmware/App/main.c:
  include MotorControl.h

Firmware/App/screw_axis.cpp:
  include MotorControl.h
  include clamp.hpp
```

所以第一步理论上应该能通过。

### 12.2 第二步：整理 link 可见性

当前：

```cmake
target_link_libraries(cms32_motor_control PUBLIC
    cms32_bsp
    cms32_foc_algorithm
    cms32_support_cpp
)
```

它能工作，但也偏宽。它会把 `cms32_bsp` 和 `cms32_support_cpp` 传给下游。

更收敛的方向：

```cmake
target_link_libraries(cms32_motor_control
    PUBLIC
        cms32_foc_algorithm
    PRIVATE
        cms32_bsp
        cms32_support_cpp
)
```

但是要注意：`cms32foc` 的 `main.c` 直接 include：

```c
#include "board_uart.h"
#include "foc_bsp.h"
```

所以最终 executable 必须自己链接 `cms32_bsp`：

```cmake
target_link_libraries(cms32foc PRIVATE
    cms32_motor_control
    cms32_bsp
    cms32_support_cpp
)
```

第二步比第一步影响更大，所以建议晚一点做。

### 12.3 第三步：Support 可见性

当前：

```cmake
add_library(cms32_support_cpp INTERFACE)
target_include_directories(cms32_support_cpp INTERFACE
    ${CMS32_SUPPORT_DIR}
    ${CMS32_BOARD_CONFIG_DIR}
    ${CMS32_MOTOR_INC_DIR}
)
```

这表示谁 link `cms32_support_cpp`，谁就能 include：

```text
Firmware/Support
Firmware/Board/Config
Firmware/MotorControl/Abi
```

这在当前混编阶段可接受，因为：

```text
units.hpp / speed_math.hpp / command_sanitizer.hpp 会用 BoardConfig/MotorControl 类型。
screw_axis.cpp 也直接 include clamp.hpp。
```

长期更理想：

```text
Support 只依赖 Support 自己。
MotorControl 专用 math/config 不放 Support。
App 如果只需要 clamp.hpp，可以有更小的 support target。
```

但现在不要急着拆，否则 CMake 学习成本会突然变高。

## 13. 链接和 include 的区别

这是最容易混的地方。

### 13.1 include 是编译期找头文件

例如：

```c
#include "MotorControl.h"
```

编译器要能在 include path 里找到它。

对应 CMake：

```cmake
target_include_directories(...)
```

### 13.2 link 是链接期找函数实现

例如 `main.c` 调用：

```c
MotorControl_Init();
```

头文件只告诉编译器“这个函数存在”。真正函数实现在哪里，要链接器从库里找。

对应 CMake：

```cmake
target_link_libraries(...)
```

所以：

```text
include 解决“声明在哪”
link 解决“实现在哪”
```

你只 include 不 link，会出现：

```text
undefined reference
```

你只 link 不 include，会出现：

```text
implicit declaration
unknown type name
file not found
```

## 14. 为什么有 INTERFACE target

例如：

```cmake
add_library(cms32_project_options INTERFACE)
```

它没有源码，不会生成 `.a`。

它的作用是集中携带：

```text
编译选项
include 路径
链接依赖
```

你可以把它理解成“配置包”。

比如：

```cmake
target_link_libraries(cms32_bsp PUBLIC
    cms32_platform
    cms32_vendor_drivers
)
```

`cms32_bsp` 会间接拿到：

```text
cms32_platform 的 include
cms32_vendor_drivers 的依赖
cms32_project_options 的 CPU 选项
```

这就是 target 传播。

## 15. 固件输出函数

当前自定义函数定义在：

```text
cmake/firmware_output.cmake
```

顶层 `CMakeLists.txt` 只负责 include 它：

```cmake
set(CMS32_SIZE_REPORT_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/print_firmware_size.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/firmware_output.cmake)
```

这样拆开的原因是：`cms32_configure_firmware()` 是“固件输出配置”，负责链接选项、hex/bin 输出和调用尺寸报告；`print_firmware_size.cmake` 只负责“打印尺寸报告”。不要把这两个职责混在同一个 print 文件里。

函数内容大致是：

```cmake
function(cms32_configure_firmware target_name)
    target_link_options(${target_name} PRIVATE
        ${CMS32_CPU_OPTIONS}
        -T${CMS32_LINKER_SCRIPT}
        -Wl,--gc-sections
        -Wl,-Map=$<TARGET_FILE_DIR:${target_name}>/${target_name}.map
        -Wl,--defsym=__stack_size__=${CMS32_STACK_BYTES}
        -Wl,--defsym=__heap_size__=${CMS32_HEAP_BYTES}
        -nostartfiles
        --specs=nano.specs
        --specs=nosys.specs
    )
    ...
endfunction()
```

它只作用在最终固件 target：

```cmake
cms32_configure_firmware(cms32foc)
```

它做三件事：

```text
1. 给链接器加 linker script 和裸机链接选项。
2. 链接后生成 .hex 和 .bin。
3. 调用 cmake/print_firmware_size.cmake 打印 Flash/RAM 使用报告。
```

关键链接选项：

```text
-T cms32m6510_flash.ld:
  指定链接脚本。

-Wl,--gc-sections:
  删除没有被引用的函数/数据段。

-nostartfiles:
  不使用标准启动文件，使用我们自己的 startup_CMS32M6510_gcc.S。

--specs=nano.specs:
  使用 newlib-nano，减小体积。

--specs=nosys.specs:
  裸机环境没有操作系统 syscall。
```

## 16. 最终固件 target

当前：

```cmake
add_executable(cms32foc
    ${CMS32_APP_DIR}/main.c
    ${CMS32_APP_DIR}/screw_axis.cpp
    ${CMS32_PLATFORM_DIR}/startup_CMS32M6510_gcc.S
)
target_link_libraries(cms32foc PRIVATE
    cms32_motor_control
    cms32_support_cpp
)
cms32_configure_firmware(cms32foc)
```

含义：

```text
cms32foc 是最终固件。
它直接编译 main.c / screw_axis.cpp / startup 汇编。
它链接 MotorControl 和 Support。
它调用 cms32_configure_firmware 加裸机链接和输出步骤。
cms32_configure_firmware 的定义在 cmake/firmware_output.cmake。
```

注意：

```text
Board BSP 现在是通过 cms32_motor_control PUBLIC 间接传过来的。
如果后续收窄 MotorControl link 可见性，cms32foc 需要直接 link cms32_bsp。
```

## 17. 新增 C 文件怎么做

如果你新增一个 Board C 文件：

```text
Firmware/Board/Src/foo.c
Firmware/Board/Inc/foo.h
```

应该加入：

```cmake
set(CMS32_BOARD_SOURCES
    ${CMS32_BOARD_SRC_DIR}/foc_bsp.c
    ${CMS32_BOARD_SRC_DIR}/foc_curr.c
    ${CMS32_BOARD_SRC_DIR}/foc_ma600.c
    ${CMS32_BOARD_SRC_DIR}/foc_pwm.c
    ${CMS32_BOARD_SRC_DIR}/board_uart.c
    ${CMS32_BOARD_SRC_DIR}/foo.c
)
```

如果 `foo.h` 放在 `Firmware/Board/Inc`，不需要额外 include，因为：

```cmake
target_include_directories(cms32_bsp PUBLIC
    ${CMS32_BOARD_INC_DIR}
    ${CMS32_BOARD_CONFIG_DIR}
)
```

已经包含了 Board/Inc。

## 18. 新增 C++ 文件怎么做

如果你新增：

```text
Firmware/MotorControl/Core/observer.cpp
```

应该把它加入：

```cmake
set(CMS32_MOTOR_CONTROL_SOURCES
    ${CMS32_MOTOR_CPP_DIR}/core.cpp
    ${CMS32_MOTOR_CPP_DIR}/current.cpp
    ${CMS32_MOTOR_CPP_DIR}/encoder.cpp
    ${CMS32_MOTOR_CPP_DIR}/observer.cpp
    ${CMS32_MOTOR_CPP_DIR}/output.cpp
    ${CMS32_MOTOR_CPP_DIR}/vf.cpp
)
```

同时删除被替代的 C 文件：

```cmake
${CMS32_MOTOR_C_DIR}/motor_control_watch.c
```

不要两个都留，否则会重复定义：

```text
MotorControl_WatchFill
MotorControl_WatchCopyToVolatile
```

新增后跑：

```sh
cmake --build --preset gcc-debug --target cms32foc
```

如果你新增的是 header-only C++ 工具，比如：

```text
Firmware/MotorControl/Core/foo_math.hpp
```

通常不需要加入 CMake 源列表。只要某个 `.cpp` include 它，它就会参与编译。

但要加入 `testhpp.sh` 做头文件编译检查。

## 19. 修改 include 可见性的动手步骤

这是我们接下来最适合练的 CMake 修改。

### 19.1 当前状态

```cmake
target_include_directories(cms32_motor_control PUBLIC
    ${CMS32_MOTOR_C_DIR}
    ${CMS32_MOTOR_CPP_DIR}
    ${CMS32_MOTOR_INC_DIR}
)
```

### 19.2 改成分层写法

```cmake
target_include_directories(cms32_motor_control
    PUBLIC
        ${CMS32_MOTOR_INC_DIR}
    PRIVATE
        ${CMS32_MOTOR_C_DIR}
        ${CMS32_MOTOR_CPP_DIR}
)
```

### 19.3 为什么这样改

因为：

```text
MotorControl/Inc/MotorControl.h:
  是对外 API。

MotorControl/C/motor_control_state.h:
  是扁平状态和 Ozone 观察入口。当前 include 可见性还没收窄时，App 可以少量使用。

MotorControl/C/motor_control_internal.h:
  是内部函数声明，不应该给 App include。

MotorControl/Cpp/*.hpp:
  是内部 C++ 迁移工具，不应该给 App include。
```

### 19.4 改完怎么验证

```sh
cmake --build --preset gcc-debug --target cms32foc
```

如果报：

```text
fatal error: motor_control_internal.h: No such file or directory
```

说明某个外部模块偷偷 include 了内部头。要先修依赖，而不是把目录重新 PUBLIC。

如果报：

```text
fatal error: config.hpp: No such file or directory
```

同理，说明有外部模块直接用了 MotorControl C++ 内部头。

当前工程检查下来，App 层只直接 include：

```text
MotorControl.h
clamp.hpp
```

所以第一步收口应该能通过。

## 20. 常见错误

### 20.1 把 include path 当成 link

错误理解：

```text
我 include 了头文件，所以函数实现也能找到。
```

正确理解：

```text
include 只解决声明。
link 才解决实现。
```

### 20.2 把所有目录都 PUBLIC

这样最容易一开始能编译，但长期分层失控：

```cmake
target_include_directories(foo PUBLIC
    Src
    Inc
    Internal
    Cpp
)
```

更好的习惯：

```cmake
target_include_directories(foo
    PUBLIC
        Inc
    PRIVATE
        Src
        Internal
        Cpp
)
```

### 20.3 只 configure 不 build

错误：

```sh
cmake --preset gcc-debug
```

然后说“编译通过”。

正确：

```sh
cmake --preset gcc-debug
cmake --build --preset gcc-debug --target cms32foc
```

### 20.4 忘记 C++ 文件需要 C++ options

如果某个 C++ target 没有间接 link `cms32_cpp_options`，可能会丢掉：

```text
-fno-exceptions
-fno-rtti
-fno-threadsafe-statics
-fno-use-cxa-atexit
```

当前 `cms32_motor_control` 通过：

```cmake
cms32_support_cpp -> cms32_cpp_options -> cms32_project_options
```

获得这些选项。

后续新增新的 C++ target 时也要记得链接对应 options。

## 21. 推荐学习顺序

先学这几个 CMake 概念就够当前工程用：

```text
1. configure vs build
2. preset
3. toolchain file
4. target
5. STATIC / INTERFACE / EXECUTABLE
6. target_include_directories
7. target_link_libraries
8. PUBLIC / PRIVATE / INTERFACE
9. source list
10. post build command
```

暂时不用急着学：

```text
install
package
CTest
FetchContent
ExternalProject
复杂 generator expression
跨平台安装规则
```

我们这个裸机固件项目，最重要的是：

```text
构建路径清楚
include 边界清楚
链接依赖清楚
编译选项可追踪
Flash/RAM 输出可验证
```

## 22. 当前一句话总结

当前 CMake 已经是比较现代的 target-based 结构：

```text
不是全局 include
不是全局 flags
用 preset 固定构建方式
用 toolchain 固定交叉编译器
用 interface target 传播编译选项
用 static library 分层组织固件
用 post-build 生成 hex/bin 和内存报告
```

下一步最适合做的小改动是：

```text
把 cms32_motor_control 的 include 目录从“全部 PUBLIC”
改成“MotorControl/Inc PUBLIC，MotorControl/C 和 Cpp PRIVATE”。
```

这一步不会改变固件行为，只会让 CMake 帮我们守住 C/C++ 分层边界。
