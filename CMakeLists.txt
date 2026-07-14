cmake_minimum_required(VERSION 3.20)

project(CMS32FOC_GCC C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------------------------------------------------------------
# Paths
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
set(CMS32_MOTOR_C_DIR ${CMS32_MOTOR_DIR}/C)
set(CMS32_MOTOR_INC_DIR ${CMS32_MOTOR_DIR}/Inc)
set(CMS32_SUPPORT_DIR ${CMS32_FIRMWARE_DIR}/Support)

set(CMS32_LINKER_SCRIPT ${CMS32_PLATFORM_DIR}/cms32m6510_flash.ld)
set(CMS32_SIZE_REPORT_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/cmake/print_firmware_size.cmake)

# -----------------------------------------------------------------------------
# Memory budget
# -----------------------------------------------------------------------------

set(CMS32_FLASH_BYTES 65536)
set(CMS32_RAM_BYTES 8192)
set(CMS32_HEAP_BYTES 1024)
set(CMS32_STACK_BYTES 1024)

# -----------------------------------------------------------------------------
# Common options
# -----------------------------------------------------------------------------

set(CMS32_CPU_OPTIONS
    -mcpu=cortex-m0plus
    -mthumb
)

set(CMS32_COMMON_COMPILE_OPTIONS
    ${CMS32_CPU_OPTIONS}
    -ffunction-sections
    -fdata-sections
    -fdiagnostics-color=always
    -Wall
    -Wextra
)

add_library(cms32_project_options INTERFACE)
target_compile_options(cms32_project_options INTERFACE
    ${CMS32_COMMON_COMPILE_OPTIONS}
)
target_include_directories(cms32_project_options INTERFACE
    ${CMS32_VENDOR_DIR}/CMSIS/Include
    ${CMS32_VENDOR_DIR}/Device/Include
    ${CMS32_VENDOR_DIR}/Driver/inc
)

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
# Firmware output helper
# -----------------------------------------------------------------------------

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

    set_target_properties(${target_name} PROPERTIES
        LINK_DEPENDS "${CMS32_LINKER_SCRIPT};${CMS32_SIZE_REPORT_SCRIPT}"
    )

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${target_name}>
            $<TARGET_FILE_DIR:${target_name}>/${target_name}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${target_name}>
            $<TARGET_FILE_DIR:${target_name}>/${target_name}.bin
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

# -----------------------------------------------------------------------------
# Header-only C++ support
# -----------------------------------------------------------------------------

add_library(cms32_support_cpp INTERFACE)
target_include_directories(cms32_support_cpp INTERFACE
    ${CMS32_SUPPORT_DIR}
    ${CMS32_BOARD_CONFIG_DIR}
    ${CMS32_MOTOR_INC_DIR}
)
target_link_libraries(cms32_support_cpp INTERFACE
    cms32_cpp_options
)

# -----------------------------------------------------------------------------
# Vendor and platform
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
)

add_library(cms32_vendor_drivers STATIC
    ${CMS32_VENDOR_DRIVER_SOURCES}
)
target_link_libraries(cms32_vendor_drivers PUBLIC
    cms32_project_options
)

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
# Board support
# -----------------------------------------------------------------------------

set(CMS32_BOARD_SOURCES
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

# -----------------------------------------------------------------------------
# Motor control
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

set(CMS32_MOTOR_CONTROL_C_SOURCES
    ${CMS32_MOTOR_C_DIR}/motor_control_c.c
    ${CMS32_MOTOR_C_DIR}/motor_control_current.c
    ${CMS32_MOTOR_C_DIR}/motor_control_encoder.c
    ${CMS32_MOTOR_C_DIR}/motor_control_output.c
    ${CMS32_MOTOR_C_DIR}/motor_control_vf.c
    ${CMS32_MOTOR_C_DIR}/motor_control_watch.c
)

add_library(cms32_motor_control_c STATIC
    ${CMS32_MOTOR_CONTROL_C_SOURCES}
)
target_include_directories(cms32_motor_control_c PUBLIC
    ${CMS32_MOTOR_C_DIR}
    ${CMS32_MOTOR_INC_DIR}
)
target_link_libraries(cms32_motor_control_c PUBLIC
    cms32_bsp
    cms32_foc_algorithm
)

# -----------------------------------------------------------------------------
# Firmware image
# -----------------------------------------------------------------------------

add_executable(cms32foc
    ${CMS32_APP_DIR}/main.c
    ${CMS32_APP_DIR}/screw_axis.cpp
    ${CMS32_PLATFORM_DIR}/startup_CMS32M6510_gcc.S
)
target_link_libraries(cms32foc PRIVATE
    cms32_motor_control_c
    cms32_support_cpp
)
cms32_configure_firmware(cms32foc)
