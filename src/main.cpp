#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mpu.h"

extern "C" void irq_ext4();

int main() {
	uart_init();
	rgbled_init();
	mpu_init();
	rgbled_set(0xFF9000, 2000);

	while (true) {
		kernel_sleep(1);
		MPUSample sample = mpu_sample();
		for (int i=0; i<3; i++)
			uart << sample.accel[i] << ",";
		for (int i=0; i<3; i++)
			uart << sample.gyro[i] << ",";
		uart << endl;
	}
}
