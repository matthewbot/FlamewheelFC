#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "nav/calibration.h"
#include "nav/ins.h"
#include "math/orientation.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0x0000FF, 100);

    sched_sleep(1000);
    mag_init();
    mpu_init();
    ins_init();

/*    while (true) {
        MPUSample mpu = mpu_sample();
        MagSample mag = mag_sample();

        MatrixF<3, 3> rot = triad_algorithm(calibration_accel(mpu.accel), calibration_mag(mag.field));
        VectorF<3> rpy = rotation_to_rpy(rot);

        for (int i=0; i<3; i++)
            uart << static_cast<int>(rad_to_deg(rpy[i])) << "\t";
        uart << endl;
        sched_sleep(100);
        } */

    int sums[3] = { 0, 0, 0};
    for (int i=0; i<1024; i++) {
        MPUSample mpu = mpu_sample();
        for (int i=0; i<3; i++)
            sums[i] += mpu.gyro[i];
    }

    for (int i=0; i<3; i++)
        sums[i] /= 1024;
    int16_t sample[3] = { (int16_t)sums[0], (int16_t)sums[1], (int16_t)sums[2] };
    VectorF<3> bias = calibration_gyro(sample);
    ins_reset({1, 0, 0, 0}, bias);
    ins_start();

    while (true) {
        sched_sleep(25);
        Quaternion quat = ins_get_quaternion();
        VectorF<3> rpy = quat_to_rpy(quat);
        for (int i=0; i<3; i++)
            uart << (int)(rad_to_deg(rpy[i])*10) << "\t";
        uart << endl;
    }
}
