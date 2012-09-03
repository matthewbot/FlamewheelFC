#ifndef KERNEL_LINKER_H
#define KERNEL_LINKER_H

#include <stdint.h>

extern "C" {
	extern uint32_t __data_load;
	extern uint32_t __data_start;
	extern uint32_t __data_end;

	extern uint32_t __bss_start;
	extern uint32_t __bss_end;

	extern uint32_t __stack_start;
	extern uint32_t __stack_end;

	extern uint32_t __main_stack_start;
	extern uint32_t __main_stack_end;
}

#endif
