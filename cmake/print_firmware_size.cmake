function(format_kib out_var bytes)
    math(EXPR scaled "(${bytes} * 100 + 512) / 1024")
    math(EXPR whole "${scaled} / 100")
    math(EXPR frac "${scaled} % 100")
    if(frac LESS 10)
        set(frac_text "0${frac}")
    else()
        set(frac_text "${frac}")
    endif()
    set(${out_var} "${whole}.${frac_text}" PARENT_SCOPE)
endfunction()

function(format_percent out_var used total)
    if(total EQUAL 0)
        set(${out_var} "0.0" PARENT_SCOPE)
        return()
    endif()

    math(EXPR scaled "(${used} * 1000 + ${total} / 2) / ${total}")
    math(EXPR whole "${scaled} / 10")
    math(EXPR frac "${scaled} % 10")
    set(${out_var} "${whole}.${frac}" PARENT_SCOPE)
endfunction()

if(NOT DEFINED SIZE_TOOL)
    message(FATAL_ERROR "SIZE_TOOL is not set")
endif()
if(NOT DEFINED ELF_FILE)
    message(FATAL_ERROR "ELF_FILE is not set")
endif()

execute_process(
    COMMAND ${SIZE_TOOL} ${ELF_FILE}
    RESULT_VARIABLE size_result
    OUTPUT_VARIABLE size_output
    ERROR_VARIABLE size_error
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT size_result EQUAL 0)
    message(FATAL_ERROR "failed to run ${SIZE_TOOL}: ${size_error}")
endif()

string(REPLACE "\n" ";" size_lines "${size_output}")
list(LENGTH size_lines size_line_count)
if(size_line_count LESS 2)
    message(FATAL_ERROR "unexpected size output: ${size_output}")
endif()

list(GET size_lines 1 size_values)
string(REGEX MATCH
    "^[ \t]*([0-9]+)[ \t]+([0-9]+)[ \t]+([0-9]+)[ \t]+([0-9]+)[ \t]+([^ \t]+)[ \t]+(.+)$"
    size_match
    "${size_values}"
)

if(NOT size_match)
    message(FATAL_ERROR "cannot parse size output line: ${size_values}")
endif()

set(text_bytes "${CMAKE_MATCH_1}")
set(data_bytes "${CMAKE_MATCH_2}")
set(bss_bytes "${CMAKE_MATCH_3}")

math(EXPR flash_used "${text_bytes} + ${data_bytes}")
math(EXPR ram_static "${data_bytes} + ${bss_bytes}")
math(EXPR ram_reserved "${HEAP_BYTES} + ${STACK_BYTES}")
math(EXPR ram_total "${ram_static} + ${ram_reserved}")

format_kib(text_kib "${text_bytes}")
format_kib(data_kib "${data_bytes}")
format_kib(bss_kib "${bss_bytes}")
format_kib(flash_kib "${flash_used}")
format_kib(flash_total_kib "${FLASH_BYTES}")
format_kib(ram_static_kib "${ram_static}")
format_kib(ram_reserved_kib "${ram_reserved}")
format_kib(ram_total_kib "${ram_total}")
format_kib(ram_capacity_kib "${RAM_BYTES}")
format_percent(flash_percent "${flash_used}" "${FLASH_BYTES}")
format_percent(ram_percent "${ram_total}" "${RAM_BYTES}")

message(STATUS "")
message(STATUS "${TARGET_NAME} memory report")
message(STATUS "  +-------------+------------+-----------+------------------------------+")
message(STATUS "  | Field       | Bytes      | KiB       | Meaning                      |")
message(STATUS "  +-------------+------------+-----------+------------------------------+")
message(STATUS "  | text        | ${text_bytes}      | ${text_kib}     | code + rodata in Flash      |")
message(STATUS "  | data        | ${data_bytes}        | ${data_kib}      | init data in Flash + RAM    |")
message(STATUS "  | bss         | ${bss_bytes}       | ${bss_kib}      | zero-init RAM               |")
message(STATUS "  | Flash image | ${flash_used}      | ${flash_kib}     | text + data                 |")
message(STATUS "  | RAM static  | ${ram_static}       | ${ram_static_kib}      | data + bss                  |")
message(STATUS "  | RAM reserve | ${ram_reserved}       | ${ram_reserved_kib}      | heap + stack                |")
message(STATUS "  | RAM total   | ${ram_total}       | ${ram_total_kib}      | data + bss + heap + stack   |")
message(STATUS "  +-------------+------------+-----------+------------------------------+")
message(STATUS "")
message(STATUS "  +--------+------------+------------+--------+")
message(STATUS "  | Region | Used       | Capacity   | Use    |")
message(STATUS "  +--------+------------+------------+--------+")
message(STATUS "  | Flash  | ${flash_used} bytes | ${FLASH_BYTES} bytes | ${flash_percent}%  |")
message(STATUS "  | RAM    | ${ram_total} bytes  | ${RAM_BYTES} bytes  | ${ram_percent}%  |")
message(STATUS "  +--------+------------+------------+--------+")
message(STATUS "  RAM reserve detail: heap ${HEAP_BYTES} bytes + stack ${STACK_BYTES} bytes")
message(STATUS "")
