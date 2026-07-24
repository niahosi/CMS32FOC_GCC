cmake_minimum_required(VERSION 3.20)

project(CMS32FOC_GCC C CXX ASM)

#clangd需要使用的compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#C按照C11，不使用GNU拓展标准名
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------------------------------------------------------------
# Paths 路径变量，不在CMake中硬写路径，先是集中定义
# -----------------------------------------------------------------------------

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
set(CMS32_MOTOR_ABI_DIR ${CMS32_MOTOR_DIR}/Abi)
set(CMS32_MOTOR_CONFIG_DIR ${CMS32_MOTOR_DIR}/Config)
set(CMS32_MOTOR_CORE_DIR ${CMS32_MOTOR_DIR}/Core)
set(CMS32_MOTOR_MATH_DIR ${CMS32_MOTOR_DIR}/Math)
set(CMS32_MOTOR_TYPES_DIR ${CMS32_MOTOR_DIR}/Types)
set(CMS32_SUPPORT_DIR ${CMS32_FIRMWARE_DIR}/Support)

set(CMS32_LINKER_SCRIPT ${CMS32_PLATFORM_DIR}/cms32m6510_flash.ld)
set(CMS32_SIZE_REPORT_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/print_firmware_size.cmake)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/firmware_output.cmake)

# -----------------------------------------------------------------------------
# Firmware Memory Budget
# -----------------------------------------------------------------------------

set(CMS32_FLASH_BYTES 65536)
set(CMS32_RAM_BYTES 8192)
set(CMS32_HEAP_BYTES 1024)
set(CMS32_STACK_BYTES 1024)

# -----------------------------------------------------------------------------
# Project Options
# -----------------------------------------------------------------------------

#定义最底层的硬件和编译规则
#-mcpu=cortex-m0plus：明确指定目标芯片架构为 Cortex-M0+
#-mthumb：必须的，因为 Cortex-M 系列只支持 Thumb 指令集。
set(CMS32_CPU_OPTIONS
    -mcpu=cortex-m0plus
    -mthumb
)

#[[
-----------------------------------------------------------------------------------
-ffunction-sections 和 -fdata-sections：
这两个是将每一个函数和变量放到独立的代码段/数据段中，在最终链接时
配合链接器 --gc-sections 参数，就能精确剔除没有被调用到的废弃代码。
-----------------------------------------------------------------------------------
-fdiagnostics-color=always
强制编译器输出带有颜色的诊断信息。
-----------------------------------------------------------------------------------
-Wall -Wextra：
开启严格的编译警告，提前把低级错误扼杀在摇篮里。
-----------------------------------------------------------------------------------
]]
set(CMS32_COMMON_COMPILE_OPTIONS
    ${CMS32_CPU_OPTIONS}
    -ffunction-sections
    -fdata-sections
    -fdiagnostics-color=always
    -Wall
    -Wextra
)

# 20 kHz 电流环不能使用 CMake Debug 默认的 -O0。保留 -g 调试信息，
# 但让所有参与 ADC/PWM/FOC 实时链路的 target 在 Debug 配置下也使用 -O2。
function(cms32_enable_realtime_optimization target_name)
    target_compile_options(${target_name} PRIVATE
        $<$<CONFIG:Debug>:-O2>
    )
endfunction()

#CMS32开发要用到的虚拟配置包
#INTERFACE target 不生成真实库文件，只向依赖它的 target 传播配置。
#这里传播公共编译选项以及 CMS32 官方驱动头文件路径。
add_library(cms32_project_options INTERFACE)
target_compile_options(cms32_project_options INTERFACE
    ${CMS32_COMMON_COMPILE_OPTIONS}
)
target_include_directories(cms32_project_options INTERFACE
    ${CMS32_VENDOR_DIR}/CMSIS/Include
    ${CMS32_VENDOR_DIR}/Device/Include
    ${CMS32_VENDOR_DIR}/Driver/inc
)

#CMS32开发要用到的CPP虚拟配置包
#[[
$<$<COMPILE_LANGUAGE:CXX>:...> 是 CMake 的生成器表达式。
只有在编译 .cpp 文件时，才添加后面的参数，防止这些 C++ 专属参数污染 C 语言的编译。
-----------------------------------------------------------------------------------
-fno-exceptions：
关闭 C++ 异常，避免裸机工程引入异常运行时。
-----------------------------------------------------------------------------------
-fno-rtti：
关闭 RTTI，避免 typeinfo / dynamic_cast 相关运行时和额外体积。
-----------------------------------------------------------------------------------
-fno-threadsafe-statics / -fno-use-cxa-atexit：
避免局部静态对象线程安全保护和析构登记机制，这些在当前裸机控制路径里不需要。
-----------------------------------------------------------------------------------
target_link_libraries(cms32_cpp_options INTERFACE cms32_project_options)
让 C++ 配置包继承基础配置包。
-----------------------------------------------------------------------------------
]]
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

# -----------------------------------------------------------------------------
# Header-Only Support
# -----------------------------------------------------------------------------

add_library(cms32_support_cpp INTERFACE)
target_include_directories(cms32_support_cpp INTERFACE
    ${CMS32_SUPPORT_DIR}
    ${CMS32_BOARD_CONFIG_DIR}
    ${CMS32_MOTOR_ABI_DIR}
)
target_link_libraries(cms32_support_cpp INTERFACE
    cms32_cpp_options
)

# -----------------------------------------------------------------------------
# Vendor And Platform
# -----------------------------------------------------------------------------

set(CMS32_VENDOR_DRIVER_SOURCES
    ${CMS32_VENDOR_DIR}/Driver/src/adc.c
    ${CMS32_VENDOR_DIR}/Driver/src/adcldo.c
    ${CMS32_VENDOR_DIR}/Driver/src/cgc.c
    ${CMS32_VENDOR_DIR}/Driver/src/delay.c
    ${CMS32_VENDOR_DIR}/Driver/src/epwm.c
    ${CMS32_VENDOR_DIR}/Driver/src/gpio.c
    ${CMS32_VENDOR_DIR}/Driver/src/pga.c
    ${CMS32_VENDOR_DIR}/Driver/src/ssp.c
    ${CMS32_VENDOR_DIR}/Driver/src/timer.c
    ${CMS32_VENDOR_DIR}/Driver/src/uart.c
)

#vendor包含芯片原厂底层外设驱动
#STATIC 表示把这些 .c 文件真实地编译成静态库文件
add_library(cms32_vendor_drivers STATIC
    ${CMS32_VENDOR_DRIVER_SOURCES}
)
target_link_libraries(cms32_vendor_drivers PUBLIC
    cms32_project_options
)
cms32_enable_realtime_optimization(cms32_vendor_drivers)

#platform是我修改原厂后一些底层配置
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

# -----------------------------------------------------------------------------
# Board Support
# -----------------------------------------------------------------------------

set(CMS32_BOARD_SOURCES
    ${CMS32_BOARD_SRC_DIR}/board_profile.c
    ${CMS32_BOARD_SRC_DIR}/board_uart.c
    ${CMS32_BOARD_SRC_DIR}/foc_bsp.c
    ${CMS32_BOARD_SRC_DIR}/foc_curr.c
    ${CMS32_BOARD_SRC_DIR}/foc_ma600.c
    ${CMS32_BOARD_SRC_DIR}/foc_pwm.c
)

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
cms32_enable_realtime_optimization(cms32_bsp)

# -----------------------------------------------------------------------------
# Motor Control
# -----------------------------------------------------------------------------

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
cms32_enable_realtime_optimization(cms32_foc_algorithm)

set(CMS32_MOTOR_CONTROL_SOURCES
    ${CMS32_MOTOR_CORE_DIR}/core.cpp
    ${CMS32_MOTOR_CORE_DIR}/current.cpp
    ${CMS32_MOTOR_CORE_DIR}/encoder.cpp
    ${CMS32_MOTOR_CORE_DIR}/output.cpp
    ${CMS32_MOTOR_CORE_DIR}/vf.cpp
)

add_library(cms32_motor_control STATIC
    ${CMS32_MOTOR_CONTROL_SOURCES}
)
target_include_directories(cms32_motor_control PUBLIC
    ${CMS32_MOTOR_ABI_DIR}
    ${CMS32_MOTOR_CONFIG_DIR}
    ${CMS32_MOTOR_CORE_DIR}
    ${CMS32_MOTOR_MATH_DIR}
    ${CMS32_MOTOR_TYPES_DIR}
)
target_link_libraries(cms32_motor_control PUBLIC
    cms32_bsp
    cms32_foc_algorithm
    cms32_support_cpp
)
cms32_enable_realtime_optimization(cms32_motor_control)

# -----------------------------------------------------------------------------
# Firmware Image
# -----------------------------------------------------------------------------

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
