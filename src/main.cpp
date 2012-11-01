#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "math/orientation.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0x0000FF, 100);

    sched_sleep(1000);
    mag_init();
    mpu_init();

/*    while (true) {
        MagSample mag = mag_sample();

        uart << mag.x << ",\t" << mag.y << ",\t" << mag.z << endl;
        sched_sleep(10);
        }*/

    while (true) {
        MPUSample mpu = mpu_sample();
        MagSample mag = mag_sample();

        VectorF<3> accelvec = {(float)mpu.accel[0], (float)-mpu.accel[1], (float)-mpu.accel[2]};
        VectorF<3> magvec = {(float)-mag.x, (float)-mag.y, (float)-mag.z};

        magvec = magvec - VectorF<3>{90, 149, 213.50};

//        VectorF<3> accelvec = {0, 0, 1};
//        VectorF<3> magvec = {0.21, 0, 0.98};

        MatrixF<3, 3> rot = triad_algorithm(accelvec, magvec);
        VectorF<3> rpy = rotation_to_rpy(rot);

        uart << (int)accelvec[0] << "\t" << (int)accelvec[1] << "\t" << (int)accelvec[2] << "\t";
        uart << (int)magvec[0] << "\t" << (int)magvec[1] << "\t" << (int)magvec[2] << "\t";
        uart << (int)(rpy[0]*(180/pi)) << "\t" << (int)(rpy[1]*(180/pi)) << "\t" << (int)(rpy[2]*(180/pi)) << endl;
        sched_sleep(100);
    }
}
