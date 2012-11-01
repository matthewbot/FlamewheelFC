#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0x0000FF, 100);
    mag_init();

    sched_sleep(1000);
    while (true) {
        MagSample sample = mag_sample();
        uart << sample.x << " " << sample.y << " " << sample.z << endl;
        sched_sleep(25);
    }
}
