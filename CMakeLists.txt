cmake_minimum_required(VERSION 3.20)

# 当前 bring-up 工程只启用 C/ASM：
# - C   ：原厂驱动、板级支持、main 入口包装、bring-up 控制层
# - ASM ：Cortex-M 启动文件
#
# C++ 最终控制层源码暂时冻结为普通源码文件，不创建 CMake 目标，
# 不参与 configure/build，也不要求 C++ 工具链。
project(CMS32FOC_GCC C ASM)

# clangd 直接读取该文件获得交叉编译参数和头文件路径。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 嵌入式工程不要依赖编译器默认语言标准。工具链升级后默认值可能变化，
# 显式指定可以让工程行为更稳定。
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# 工程内常用根路径。集中定义后，下面的目标不需要反复写长路径，
# 也更容易看出 ODrive 风格的固件目录结构。
set(CMS32_FIRMWARE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Firmware)
set(CMS32_VENDOR_DIR ${CMS32_FIRMWARE_DIR}/ThirdParty/Cmsemicon/CMS32M65xx)
set(CMS32_PLATFORM_DIR ${CMS32_FIRMWARE_DIR}/Drivers/CMS32M6510)
set(CMS32_LINKER_SCRIPT ${CMS32_PLATFORM_DIR}/cms32m6510_flash.ld)
set(CMS32_SIZE_REPORT_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/print_firmware_size.cmake)
set(CMS32_FLASH_BYTES 65536)
set(CMS32_RAM_BYTES 8192)
set(CMS32_HEAP_BYTES 1024)
set(CMS32_STACK_BYTES 1024)

# CPU 选项既要用于编译，也要用于链接，所以单独放成一个列表复用。
set(CMS32_CPU_OPTIONS
    -mcpu=cortex-m0plus
    -mthumb
)

# C/C++ 公共编译选项：
# - 函数/数据独立分段：让链接器可以丢弃未使用的函数和数据
# - 调试源码路径保持 Linux 原始路径，由 Ozone AddPathSubstitute 映射到 WSL UNC
# - 当前先打开常用警告，包含项目代码和原厂代码
set(CMS32_COMMON_COMPILE_OPTIONS
    ${CMS32_CPU_OPTIONS}
    -ffunction-sections
    -fdata-sections
    -fdiagnostics-color=always
    -Wall
    -Wextra
)

# 在 add_executable() 之后调用，用来配置一个固件目标。
#
# 主固件和每个可烧录诊断固件都会复用这个函数。
# 它负责挂接链接脚本、生成 map/hex/bin，并在链接后打印 size。
function(cms32_configure_firmware target_name)
    target_link_options(${target_name} PRIVATE
        ${CMS32_CPU_OPTIONS}
        # 项目自有链接脚本负责 Flash/RAM 布局、向量表、option byte、
        # .data/.bss、heap、stack 和 C++ init array。
        -T${CMS32_LINKER_SCRIPT}
        -Wl,--gc-sections
        -Wl,-Map=$<TARGET_FILE_DIR:${target_name}>/${target_name}.map
        -Wl,--defsym=__stack_size__=${CMS32_STACK_BYTES}
        -Wl,--defsym=__heap_size__=${CMS32_HEAP_BYTES}
        -nostartfiles
        --specs=nano.specs
        --specs=nosys.specs
    )

    # 链接脚本变化时必须重新链接固件。CMake 对 linker script 的依赖
    # 不一定能自动推导出来，所以这里显式声明。
    set_target_properties(${target_name} PROPERTIES
        LINK_DEPENDS "${CMS32_LINKER_SCRIPT};${CMS32_SIZE_REPORT_SCRIPT}"
    )

    # 把 ELF、HEX、BIN、MAP 都放在同一个 build/gcc-debug 目录下。
    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target_name}> $<TARGET_FILE_DIR:${target_name}>/${target_name}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target_name}> $<TARGET_FILE_DIR:${target_name}>/${target_name}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target_name}>
        COMMAND ${CMAKE_COMMAND}
            -D SIZE_TOOL=${CMAKE_SIZE}
            -D ELF_FILE=$<TARGET_FILE:${target_name}>
            -D TARGET_NAME=${target_name}
            -D FLASH_BYTES=${CMS32_FLASH_BYTES}
            -D RAM_BYTES=${CMS32_RAM_BYTES}
            -D HEAP_BYTES=${CMS32_HEAP_BYTES}
            -D STACK_BYTES=${CMS32_STACK_BYTES}
            -P ${CMS32_SIZE_REPORT_SCRIPT}
    )
endfunction()

# 接口库不会生成 .a 文件。它常用来打包“使用要求”，
# 比如编译选项、头文件搜索路径，再传播给链接它的目标。
add_library(cms32_project_options INTERFACE)
target_compile_options(cms32_project_options INTERFACE
    ${CMS32_COMMON_COMPILE_OPTIONS}
)
target_include_directories(cms32_project_options INTERFACE
    ${CMS32_VENDOR_DIR}/CMSIS/Include
    ${CMS32_VENDOR_DIR}/Device/Include
    ${CMS32_VENDOR_DIR}/Driver/inc
)

# 原厂驱动层。这里的文件来自 CMS32M65xx 厂商包，
# 应当视作第三方输入，而不是项目自己的架构代码。
add_library(cms32_vendor_drivers STATIC
    ${CMS32_VENDOR_DIR}/Driver/src/adc.c
    ${CMS32_VENDOR_DIR}/Driver/src/adcldo.c
    ${CMS32_VENDOR_DIR}/Driver/src/cgc.c
    ${CMS32_VENDOR_DIR}/Driver/src/delay.c
    ${CMS32_VENDOR_DIR}/Driver/src/epwm.c
    ${CMS32_VENDOR_DIR}/Driver/src/gpio.c
    ${CMS32_VENDOR_DIR}/Driver/src/pga.c
    ${CMS32_VENDOR_DIR}/Driver/src/ssp.c
)
# PUBLIC 表示链接 cms32_vendor_drivers 的目标，也会继承
# cms32_project_options 里的头文件搜索路径和编译选项。
target_link_libraries(cms32_vendor_drivers PUBLIC cms32_project_options)

# 平台层：CMS32M6510 专属的平台胶水代码和原厂 SystemInit()。
# 汇编启动文件不放进这个库，而是直接链接到每个可执行目标；
# 这样每个固件镜像都有自己的向量表。
add_library(cms32_platform STATIC
    ${CMS32_PLATFORM_DIR}/cms32m6510_platform.c
    ${CMS32_VENDOR_DIR}/Device/Source/system_CMS32M6510.c
)
target_include_directories(cms32_platform PUBLIC
    ${CMS32_PLATFORM_DIR}
    ${CMS32_PLATFORM_DIR}/Memory/Inc
)
target_link_libraries(cms32_platform PUBLIC cms32_project_options)

# 板级支持包。这里负责真实硬件连线和安全态：
# GPIO、PWM、ADC/PGA、MA600 SPI，以及 P16 驱动使能策略。
add_library(cms32_bsp STATIC
    Firmware/Board/Src/foc_bsp.c
    Firmware/Board/Src/foc_curr.c
    Firmware/Board/Src/foc_ma600.c
    Firmware/Board/Src/foc_ma600_diag.c
    Firmware/Board/Src/foc_pwm.c
)
target_include_directories(cms32_bsp PUBLIC
    Firmware/Board/Inc
    Firmware/Board/Config
)
target_link_libraries(cms32_bsp
    PUBLIC
        # Board 层依赖芯片平台层和原厂外设驱动。
        cms32_platform
        cms32_vendor_drivers
)

# 开放式 FOC 算法层。这里只放坐标变换、PI、限幅、SVPWM 等纯算法；
# 不访问寄存器，也不隐藏中间量，便于后续对照 Legacy/LXQS/ACC 调参。
add_library(cms32_foc_algorithm STATIC
    Firmware/MotorControl/Algorithm/foc_math.c
)
target_include_directories(cms32_foc_algorithm PUBLIC
    Firmware/MotorControl/Algorithm
    Firmware/Board/Config
)
target_link_libraries(cms32_foc_algorithm PUBLIC
    cms32_project_options
)

# 纯 C bring-up 控制层。主固件当前使用它来优先稳定 VF 和电流采样；
# C++ 控制层源码冻结，不创建库目标。
add_library(cms32_motor_control_c STATIC
    Firmware/MotorControl/C/motor_control_c.c
    Firmware/MotorControl/C/motor_control_diag.c
)
target_include_directories(cms32_motor_control_c PUBLIC
    Firmware/MotorControl/C
    Firmware/MotorControl/Inc
)
target_link_libraries(cms32_motor_control_c PUBLIC
    cms32_bsp
    cms32_foc_algorithm
)

# 当前主固件是纯 C bring-up 固件。VF、电流采样和 Ozone 调试先在
# cms32_motor_control_c 中稳定，C++ 控制层后续再按验证结果恢复。
add_executable(cms32foc
    Firmware/App/main.c
    ${CMS32_PLATFORM_DIR}/startup_CMS32M6510_gcc.S
)
target_link_libraries(cms32foc PRIVATE
    cms32_motor_control_c
)
cms32_configure_firmware(cms32foc)

# 板级观察诊断固件。它初始化板级硬件，保持 PWM 功率输出安全关闭，
# 并暴露 g_board_watch 给 Ozone Watch。
add_executable(cms32_board_watch_test
    Firmware/Tests/BoardWatch/main_board_watch.c
    ${CMS32_PLATFORM_DIR}/startup_CMS32M6510_gcc.S
)
target_link_libraries(cms32_board_watch_test PRIVATE
    cms32_bsp
)
cms32_configure_firmware(cms32_board_watch_test)

# 启动冒烟诊断固件。最小 C 测试，用于验证向量表、Reset 入口、
# .data/.bss 初始化，以及能否进入 main 循环。
add_executable(cms32_startup_smoke_test
    Firmware/Tests/StartupSmoke/main_startup_smoke.c
    ${CMS32_PLATFORM_DIR}/startup_CMS32M6510_gcc.S
)
target_link_libraries(cms32_startup_smoke_test PRIVATE
    cms32_platform
)
cms32_configure_firmware(cms32_startup_smoke_test)
