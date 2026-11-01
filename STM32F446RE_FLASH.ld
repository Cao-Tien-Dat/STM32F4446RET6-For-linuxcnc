/*
 * Linker script for STM32F446RE — Remora Flexi-HAL bare-metal firmware
 *
 * Memory layout (matches Mbed mbed_app_target):
 *   Flash: 512 KB @ 0x08000000
 *   App    starts at 0x08010000 (64 KB reserved for bootloader)
 *   RAM:   128 KB @ 0x20000000
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08010000, LENGTH = 448K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
    /* Vector table and code */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* ARM exception unwind tables (if needed) */
    .ARM.exidx :
    {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    .init_array :
    {
        . = ALIGN(4);
        __init_array_start = .;
        KEEP(*(.init_array*))
        __init_array_end = .;
    } > FLASH

    .fini_array :
    {
        . = ALIGN(4);
        __fini_array_start = .;
        KEEP(*(.fini_array*))
        __fini_array_end = .;
    } > FLASH

    /* Data section — initialized data copied from Flash to RAM at startup */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    __sidata = LOADADDR(.data);

    /* BSS section — zero-initialized data */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = .;
    } > RAM

    /* Stack and heap */
    . = ALIGN(8);
    _end = .;
    __heap_limit = ORIGIN(RAM) + LENGTH(RAM);
    . = ALIGN(8);
    _estack = ORIGIN(RAM) + LENGTH(RAM);
    _min_stack_size = 0x400;

    /* Symbols for startup — vector table is at start of Flash (0x08010000) */
    __vector_table = ADDR(.isr_vector);
    _sidata = __sidata;
    _sdata = _sdata;
    _edata = _edata;
    _sbss = _sbss;
    _ebss = _ebss;
    _eronly = .;
}
