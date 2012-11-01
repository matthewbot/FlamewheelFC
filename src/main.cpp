#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0x0000FF, 100);

    sched_sleep(1000);
    mag_init();
    mpu_init();

    while (true) {
        MPUSample mpu = mpu_sample();
        for (int i=0; i<3; i++)
            uart << mpu.accel[i] << ",\t";
        for (int i=0; i<3; i++)
            uart << mpu.gyro[i] << ",\t";

        MagSample mag = mag_sample();
        uart << mag.x << ",\t" << mag.y << ",\t" << mag.z << endl;
        sched_sleep(10);
    }
}
