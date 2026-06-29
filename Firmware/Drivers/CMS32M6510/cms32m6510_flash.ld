/*
 * CMS32M6510GE24SS flash linker script for the CMS32FOC_GCC project.
 *
 * Flash: 64 KiB at 0x00000000
 * SRAM :  8 KiB at 0x20000000
 * Option bytes are kept at 0x000000C0 to match the vendor system file.
 */

ENTRY(Reset_Handler)

FLASH_ORIGIN = 0x00000000;
FLASH_LENGTH = 64K;
RAM_ORIGIN = 0x20000000;
RAM_LENGTH = 8K;

STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x400;
HEAP_SIZE = DEFINED(__heap_size__) ? __heap_size__ : 0x400;

MEMORY
{
    FLASH (rx)  : ORIGIN = FLASH_ORIGIN, LENGTH = FLASH_LENGTH
    RAM   (rwx) : ORIGIN = RAM_ORIGIN, LENGTH = RAM_LENGTH
}

GROUP(libgcc.a libc.a libm.a libnosys.a)

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.vectors))
        __Vectors_End = .;
        __Vectors_Size = __Vectors_End - __Vectors;

        . = 0x000000C0;
        KEEP(*(.option_byte))

        . = ALIGN(4);
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN(__preinit_array_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN(__init_array_end = .);

        . = ALIGN(4);
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN(__fini_array_end = .);

        . = ALIGN(4);
    } > FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    . = ALIGN(4);
    __etext = .;

    .data : AT(__etext)
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(.data*)
        . = ALIGN(4);
        __data_end__ = .;
    } > RAM

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > RAM

    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __HeapBase = .;
        . += HEAP_SIZE;
        __HeapLimit = .;
    } > RAM

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        __StackLimit = .;
        . += STACK_SIZE;
        . = ALIGN(8);
        __StackTop = .;
    } > RAM

    PROVIDE(__stack = __StackTop);
    PROVIDE(end = __HeapBase);
    PROVIDE(__end__ = __HeapBase);

    /DISCARD/ :
    {
        *(.note*)
        *(.comment)
    }

    ASSERT(__Vectors == ORIGIN(FLASH), "vector table must start at flash origin")
    ASSERT(__Vectors_Size == 0xC0, "unexpected CMS32M6510 vector table size")
    ASSERT(__StackTop <= ORIGIN(RAM) + LENGTH(RAM), "region RAM overflowed")
    ASSERT(__etext <= ORIGIN(FLASH) + LENGTH(FLASH), "region FLASH overflowed")
}
