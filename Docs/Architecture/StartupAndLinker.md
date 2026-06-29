# CMS32M6510 Startup and Linker Notes

This note records the startup knowledge validated on the CMS32FOC_GCC board. It is written as a practical debugging map, not as a complete ARM assembly tutorial.

## Startup Flow

The CPU does not start from `main()`. On reset, Cortex-M reads the first two words from Flash:

```text
0x00000000: initial stack pointer
0x00000004: Reset_Handler address
```

In this project those words come from `Firmware/Drivers/CMS32M6510/startup_CMS32M6510_gcc.S`:

```asm
__Vectors:
    .word __StackTop
    .word Reset_Handler
```

The verified reset flow is:

```text
Reset_Handler
 -> copy .data from Flash to RAM
 -> clear .bss in RAM
 -> SystemInit()
 -> __libc_init_array()
 -> main()
```

`SystemInit()` comes from the vendor device support package. `__libc_init_array()` is required for C++ global/static constructors.

## Memory Sections

The linker script decides where code and data live:

```text
Code Flash: 0x00000000 - 0x0000FFFF, 64 KiB
SRAM      : 0x20000000 - 0x20001FFF, 8 KiB
Data Flash: 0x00500200 - 0x005005FF, 1 KiB
```

The important runtime sections are:

- `.vectors`: interrupt vector table, must start at `0x00000000`.
- `.option_byte`: vendor option bytes, fixed at `0x000000C0`.
- `.text`: executable code in Flash.
- `.rodata`: constants in Flash.
- `.data`: variables with initial values; initial image is in Flash, runtime copy is in RAM.
- `.bss`: zero-initialized variables in RAM.
- `.init_array`: C++ static/global constructor table.
- `.heap`: dynamic allocation area, used by `malloc`/`new`.
- `.stack`: function call, interrupt, local variable, and register-save area.

The current project reserves:

```text
Stack: 0x400 bytes
Heap : 0x400 bytes
```

The stack grows downward from `__StackTop`, near the end of SRAM. On this MCU, SRAM ends at `0x20002000`.

## Why `.data` and `.bss` Matter

For a variable such as:

```c
int x = 123;
```

the value `123` is stored in Flash, but `x` must live in RAM while the program runs. Startup copies `.data` from Flash to RAM before `main()`.

For variables such as:

```c
int y;
static int z;
```

C/C++ require them to start as zero. Startup clears `.bss` before `main()`.

If `.data` or `.bss` setup is wrong, global variables look random, state machines start in strange states, and faults often appear far away from the real bug.

## C++ Constructor Fix

The first C++ smoke test showed:

```text
loop_count increments
ctor_value = 0
```

That meant the program reached `main()`, but C++ static constructors did not run.

The root cause was that `.init_array` existed, but the linker script did not define the boundary symbols used by newlib:

```text
__preinit_array_start / __preinit_array_end
__init_array_start    / __init_array_end
__fini_array_start    / __fini_array_end
```

After adding those symbols in `cms32m6510_flash.ld`, `__libc_init_array()` could find and call the constructor table. The C++ smoke test then showed:

```text
ctor_value = 0x517B1D3D
```

This confirms:

```text
Reset_Handler
 -> SystemInit()
 -> __libc_init_array()
 -> C++ static constructor
 -> main()
```

## Why `_init` and `_fini` Exist

Because this is a bare-metal firmware, the project links with `-nostartfiles`. That means the usual C runtime startup objects are not linked.

newlib's `__libc_init_array()` still references `_init`, so the project provides empty bare-metal definitions in startup assembly:

```asm
_init:
    bx lr

_fini:
    bx lr
```

Putting these symbols directly in the startup object avoids archive link-order issues.

## Debugging Method

Startup problems are easier when split into small tests:

- `cms32_startup_smoke_test`: verifies vector table, reset entry, `.data`, `.bss`, and `main()`.
- `cms32_cpp_smoke_test`: verifies `__libc_init_array()` and C++ constructor execution.
- `cms32_board_watch_test`: verifies Board init, MA600, ADC/PWM sync, and PWM safety state.

Useful checks:

```sh
arm-none-eabi-nm -n build/gcc-debug/cms32_cpp_smoke_test \
  | rg "__Vectors|user_opt_data|main|__init_array|__libc_init_array|_init|_fini"
```

Expected examples:

```text
__Vectors       = 0x00000000
user_opt_data   = 0x000000C0
__init_array_*  exists when C++ constructors are used
```

In Ozone, the useful smoke variables are:

```text
g_startup_smoke.loop_count
g_cpp_smoke.loop_count
g_cpp_smoke.ctor_value
```

If `loop_count` does not move, suspect reset/vector/startup/flash programming first. If `loop_count` moves but `ctor_value` is zero, suspect `__libc_init_array()` or `.init_array` linker symbols.
