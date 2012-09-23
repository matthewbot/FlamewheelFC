#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/sched.h"
#include "kernel/kernel.h"
#include "kernel/debug.h"

int main() {
	debug_init();

	while (true) {
		debug_puts("Hello World!\r\n");
		for (int i=0; i<3; i++)
			debug_setLED(i, true);
		sched_sleep(1000);
		debug_puthex(0xABCD1234);
		debug_puts("\r\n");
		for (int i=0; i<3; i++)
			debug_setLED(i, false);
		sched_sleep(1000);
	}
}
