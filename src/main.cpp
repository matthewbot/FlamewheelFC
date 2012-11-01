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

    uart << "Configuring mag" << endl;
    mag_writereg(0, 0x18);
    mag_writereg(2, 0x00);

    sched_sleep(500);

    int i=0;
    while (true) {
        int16_t x = (mag_readreg(3) << 8) | mag_readreg(4);
        int16_t z = (mag_readreg(5) << 8) | mag_readreg(6);
        int16_t y = (mag_readreg(7) << 8) | mag_readreg(8);
        uart << "Reading: " << x << " " << y << " " << z << endl;
        sched_sleep(100);
    }
}
