#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/sched.h"
#include "kernel/kernel.h"
#include "kernel/debug.h"
#include "drivers/rgbled.h"
#include "drivers/esc.h"

int main() {
	debug_init();
	rgbled_init();
	esc_init();
	sched_sleep(5000);

	rgbled_set(0xFFAF00, 1000);
	esc_arm();

	while (true) {
		esc_set(1, 300);
		debug_puts("Hello World!\r\n");
		for (int i=0; i<3; i++)
			debug_setLED(i, true);
		sched_sleep(1000);

		esc_set(1, 500);
		debug_puthex(0xABCD1234);
		debug_puts("\r\n");
		for (int i=0; i<3; i++)
			debug_setLED(i, false);
		sched_sleep(1000);
	}
}
