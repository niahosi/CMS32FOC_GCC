/*
 * Project-owned GCC startup for CMS32M6510 Cortex-M0+.
 *
 * Reset flow:
 *   copy .data -> clear .bss -> SystemInit() -> main()
 */

    .syntax unified
    .cpu cortex-m0plus
    .thumb

    .extern SystemInit
    .extern __libc_init_array
    .extern main
    .extern __StackTop
    .extern __etext
    .extern __data_start__
    .extern __data_end__
    .extern __bss_start__
    .extern __bss_end__

    .section .vectors, "a", %progbits
    .align 2
    .global __Vectors
__Vectors:
    .word __StackTop
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word 0
    .word SVC_Handler
    .word 0
    .word 0
    .word PendSV_Handler
    .word SysTick_Handler

    .word LVI_IRQHandler
    .word INTP0_IRQHandler
    .word INTP1_IRQHandler
    .word INTP2_IRQHandler
    .word INTP3_IRQHandler
    .word TM01H_IRQHandler
    .word CCP_IRQHandler
    .word EPWM_IRQHandler
    .word ADC_IRQHandler
    .word ACMP_IRQHandler
    .word UART0_IRQHandler
    .word I2C0_IRQHandler
    .word SSP0_IRQHandler
    .word TIMER0_IRQHandler
    .word TIMER1_IRQHandler
    .word LSITIMER_IRQHandler
    .word Default_Handler
    .word TM00_IRQHandler
    .word TM01_IRQHandler
    .word TM02_IRQHandler
    .word TM03_IRQHandler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word Default_Handler
    .word FMC_IRQHandler
    .size __Vectors, . - __Vectors

    .section .text.Reset_Handler, "ax", %progbits
    .align 2
    .thumb_func
    .global Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    ldr r0, =__etext
    ldr r1, =__data_start__
    ldr r2, =__data_end__

.L_copy_data:
    cmp r1, r2
    bcc .L_copy_data_word
    b .L_clear_bss_prepare

.L_copy_data_word:
    ldr r3, [r0]
    str r3, [r1]
    adds r0, #4
    adds r1, #4
    b .L_copy_data

.L_clear_bss_prepare:
    ldr r0, =__bss_start__
    ldr r1, =__bss_end__
    movs r2, #0

.L_clear_bss:
    cmp r0, r1
    bcc .L_clear_bss_word
    b .L_call_main

.L_clear_bss_word:
    str r2, [r0]
    adds r0, #4
    b .L_clear_bss

.L_call_main:
    bl SystemInit
    bl __libc_init_array
    bl main

.L_hang:
    b .L_hang
    .size Reset_Handler, . - Reset_Handler

    .section .text.Default_Handler, "ax", %progbits
    .align 2
    .thumb_func
    .weak Default_Handler
    .type Default_Handler, %function
Default_Handler:
    b Default_Handler
    .size Default_Handler, . - Default_Handler

    .section .text._init, "ax", %progbits
    .align 2
    .thumb_func
    .global _init
    .type _init, %function
_init:
    bx lr
    .size _init, . - _init

    .section .text._fini, "ax", %progbits
    .align 2
    .thumb_func
    .global _fini
    .type _fini, %function
_fini:
    bx lr
    .size _fini, . - _fini

    .macro weak_handler handler_name
    .weak \handler_name
    .set \handler_name, Default_Handler
    .endm

    weak_handler NMI_Handler
    weak_handler HardFault_Handler
    weak_handler SVC_Handler
    weak_handler PendSV_Handler
    weak_handler SysTick_Handler

    weak_handler LVI_IRQHandler
    weak_handler INTP0_IRQHandler
    weak_handler INTP1_IRQHandler
    weak_handler INTP2_IRQHandler
    weak_handler INTP3_IRQHandler
    weak_handler TM01H_IRQHandler
    weak_handler CCP_IRQHandler
    weak_handler EPWM_IRQHandler
    weak_handler ADC_IRQHandler
    weak_handler ACMP_IRQHandler
    weak_handler UART0_IRQHandler
    weak_handler I2C0_IRQHandler
    weak_handler SSP0_IRQHandler
    weak_handler TIMER0_IRQHandler
    weak_handler TIMER1_IRQHandler
    weak_handler LSITIMER_IRQHandler
    weak_handler TM00_IRQHandler
    weak_handler TM01_IRQHandler
    weak_handler TM02_IRQHandler
    weak_handler TM03_IRQHandler
    weak_handler FMC_IRQHandler

    .end
