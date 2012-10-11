#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "kernel/debug.h"
#include "drivers/rgbled.h"
#include "drivers/mpu.h"

extern "C" void irq_ext4();

int main() {
	debug_init();
	rgbled_init();
	mpu_init();
	rgbled_set(0xFF9000, 2000);

	while (true) {
		kernel_sleep(100);

		MPUSample sample = mpu_sample();
		debug_puts("ACCEL_Z ");
		debug_puthex(sample.accel[2]);
		debug_puts(" NUM ");
		debug_puthex(sample.samplenum);
		debug_puts("\r\n");
	}
}
