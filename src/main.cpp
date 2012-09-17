#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/sched.h"
#include "kernel/kernel.h"
#include "kernel/debug.h"

int main() {
	debug_init();
	while (true) {
		debug_puts("Hello World!\r\n");
		debug_setLED(0, true);
		sched_sleep(1000);
		debug_puthex(0xABCD1234);
		debug_puts("\r\n");
		debug_setLED(0, false);
		sched_sleep(1000);
	}
}
