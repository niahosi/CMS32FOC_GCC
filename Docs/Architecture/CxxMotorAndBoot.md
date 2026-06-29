# CMS32FOC_GCC C/C++ and Boot Layout

This project keeps the vendor package, platform startup, linker script, and BSP in C. The motor-control layer may use lightweight C++ for structure and compile-time checks.

For the low-level reset flow, stack/data/bss setup, and C++ constructor debugging notes, see `Docs/Architecture/StartupAndLinker.md`.

## C++ rules

- Use C++17 for `Firmware/MotorControl`.
- Do not use exceptions, RTTI, iostream, heap allocation, or complex STL containers.
- Interrupt handlers keep a C ABI and call short wrapper functions.
- `Reset_Handler` calls `__libc_init_array()` after `SystemInit()` and before `main()`.

## Flash layout

- Code Flash: `0x00000000 - 0x0000FFFF`, 64 KiB.
- User option bytes: `0x000000C0 - 0x000000C3`.
- Data Flash: `0x00500200 - 0x005005FF`, 1 KiB.
- Data protection option byte: `0x00500004`; normal firmware must not write it.

Initial Data Flash use is two 512-byte parameter/status sectors:

- Block A: `0x00500200`
- Block B: `0x00500400`

Each block carries magic, version, sequence, payload, and CRC. Bootloader support will later use these blocks for boot request, app validity, app size, app CRC, and update sequence.

## Future bootloader direction

The first bootloader split should use:

- `cms32_bootloader` at `0x00000000`, size 12 KiB.
- `cms32foc_app` at `0x00003000`.

CMS32M6510 declares `__VTOR_PRESENT = 1`, so the bootloader can relocate `SCB->VTOR` to the app vector table before jumping.
