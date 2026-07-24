#拦截器,检查传入的变量在前面是否被定义过
function(cms32_require_variable variable_name)
    if(NOT DEFINED ${variable_name})
        message(FATAL_ERROR "${variable_name} is required before calling cms32_configure_firmware()")
    endif()
endfunction()

#拦截器，防止拼写错误
#[[
检查传进来的名字（比如 cms32foc）是不是一个真正用 add_executable 创建出来的目标。
如果你不小心把 cms32_configure_firmware(cms32foc)
拼成了 cms32_configure_firmware(cms32_foc)，
这里会立刻拦截，防止 CMake 带着一个空目标往下走。
]]
function(cms32_configure_firmware target_name)
    if(NOT TARGET ${target_name})
        message(FATAL_ERROR "cms32_configure_firmware() target does not exist: ${target_name}")
    endif()

    cms32_require_variable(CMS32_CPU_OPTIONS)
    cms32_require_variable(CMS32_LINKER_SCRIPT)
    cms32_require_variable(CMS32_SIZE_REPORT_SCRIPT)
    cms32_require_variable(CMS32_FLASH_BYTES)
    cms32_require_variable(CMS32_RAM_BYTES)
    cms32_require_variable(CMS32_HEAP_BYTES)
    cms32_require_variable(CMS32_STACK_BYTES)

#执行总装与后处理，核心逻辑
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
