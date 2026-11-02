/* 功能：按官方ITCM=0x02000000、DTCM=0x01000000映射启动裸机C。
 * 参数：无；返回：不返回。关闭中断的最小构建不使用厂商自定义IRQ指令。
 */
    .section .vectors,"ax",@progbits
    .option norvc
    .global _start
_start:
    j reset_entry
    nop
    nop
    nop
trap_entry:
    j trap_entry
    .section .text.startup,"ax",@progbits
reset_entry:
    .option push
    .option norelax
    la gp,__global_pointer$
    .option pop
    la sp,__stack_top
    la a0,__data_load
    la a1,__data_start
    la a2,__data_end
1:  bgeu a1,a2,2f
    lw a3,0(a0)
    sw a3,0(a1)
    addi a0,a0,4
    addi a1,a1,4
    j 1b
2:  la a0,__bss_start
    la a1,__bss_end
3:  bgeu a0,a1,4f
    sw zero,0(a0)
    addi a0,a0,4
    j 3b
4:  call main
5:  j 5b
