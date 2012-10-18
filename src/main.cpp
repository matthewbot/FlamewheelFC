#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mpu.h"

int main() {
    uart_init();
    rgbled_init();
    mpu_init();
    rgbled_set(0x00FF20, 100);

    while (true) {
        MPUSample sample = mpu_sample();
        for (int i=0; i<3; i++)
            uart << sample.accel[i] << ",\t";
        for (int i=0; i<3; i++)
            uart << sample.gyro[i] << ",\t";
        uart << endl;
    }
}
