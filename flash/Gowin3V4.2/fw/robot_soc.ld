/* 功能：片内RAM启动链接布局；与官方pico_define.vh的128KB/64KB配置一致。
 * 参数：无；链接溢出或静态数据侵入4KB保留栈时失败；不使用Flash或DDR。
 */
OUTPUT_ARCH(riscv)
ENTRY(_start)
MEMORY {
    ITCM (rx) : ORIGIN = 0x02000000, LENGTH = 128K
    DTCM (rw) : ORIGIN = 0x01000000, LENGTH = 64K
}
SECTIONS {
    .text : {
        KEEP(*(.vectors))
        *(.text.startup*) *(.text*) *(.rodata*) *(.srodata*)
        . = ALIGN(4);
    } > ITCM
    .data : {
        . = ALIGN(4); __data_start = .;
        *(.data*) *(.sdata*)
        . = ALIGN(4); __data_end = .;
    } > DTCM AT> ITCM
    __data_load = LOADADDR(.data);
    .bss (NOLOAD) : {
        . = ALIGN(4); __bss_start = .;
        *(.bss*) *(.sbss*) *(COMMON)
        . = ALIGN(4); __bss_end = .;
    } > DTCM
    __global_pointer$ = __data_start + 0x800;
    __stack_top = ORIGIN(DTCM) + LENGTH(DTCM);
    ASSERT(__bss_end <= __stack_top - 4096, "DTCM stack reserve exhausted")
    /DISCARD/ : { *(.comment) *(.eh_frame*) *(.riscv.attributes) }
}

