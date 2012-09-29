#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "kernel/debug.h"
#include "drivers/rgbled.h"
#include "drivers/mpu.h"

int main() {
	debug_init();
	rgbled_init();
	kernel_sleep(1);
	mpu_init();
	kernel_sleep(1);
	rgbled_set(0x00FF00, 1000);

	mpu_write_reg(107, (1 << 7)); // reset
	kernel_sleep(100);
	mpu_write_reg(107, 3); // Clock source is z gyro
	mpu_write_reg(106, (1 << 4)); // disable I2C interface
	mpu_write_reg(26, 1); // set DLPF to 1
	mpu_write_reg(27, 2 << 3); // set Gyro FS_SEL to 2
	mpu_write_reg(28, 2 << 3); // set Accel FS_SEL to 2

	while (true) {
		kernel_sleep(100);
		debug_puts("GYRO_Z ");
		debug_puthex((mpu_read_reg(71) << 8) | mpu_read_reg(72));
		debug_puts("\r\n");
		/*		kernel_sleep(100);
		debug_puts("WHO_AM_I ");
		debug_puthex(mpu_read_reg(117));
		debug_puts("\r\n");*/
	}

	/*int i=0;
	while (true) {
		debug_puts("Reading ");
		debug_puthex(i);
		debug_puts(" - ");
		debug_puthex(mpu_read_reg(i));
		debug_puts("\r\n");
		kernel_sleep(10);
		if (++i > 117)
			i = 0;
			}*/
}
