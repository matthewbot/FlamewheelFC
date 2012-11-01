#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "nav/calibration.h"
#include "math/orientation.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0x0000FF, 100);

    sched_sleep(1000);
    mag_init();
    mpu_init();

    while (true) {
        MPUSample mpu = mpu_sample();
        MagSample mag = mag_sample();

        MatrixF<3, 3> rot = triad_algorithm(calibration_accel(mpu.accel), calibration_mag(mag.field));
        VectorF<3> rpy = rotation_to_rpy(rot);

        for (int i=0; i<3; i++)
            uart << static_cast<int>(rad_to_deg(rpy[i])) << "\t";
        uart << endl;
        sched_sleep(100);
    }
}
