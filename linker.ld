OUTPUT_FORMAT("elf32-littlearm")
ENTRY(handler_reset)
OUTPUT_ARCH(arm)
GROUP(-lgcc -lm)

MEMORY {
	flash (rx)      : org = 0x08000000, len = 1024k
    ccm_sram (rwx)  : org = 0x10000000, len = 64k
	sram (rwx)      : org = 0x20000000, len = 128k
}

MAIN_STACK_SIZE = 4k;

SECTIONS {
	.isr_vector : {
		*(.isr_vector)
	} > flash

	.text : {
		*(.text .text*)
		*(.rodata .rodata*)
	} > flash

	.data : {
		__data_load = LOADADDR(.data);

		__data_start = .;
		*(.data .data*)
		__data_end = .;
	} >sram AT>flash

	.bss : {
		__bss_start = .;
		*(.bss .bss* COMMON)
		__bss_end = .;
	} >sram

	.stack : {
        __stack_start = .;
		__main_stack_start = .;
		. += MAIN_STACK_SIZE;
		__main_stack_end = .;
        *(.stack .stack*)
        __stack_end = .;
	} >ccm_sram
}
