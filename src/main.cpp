#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "nav/calibration.h"
#include "nav/ins.h"
#include "nav/attitude.h"
#include "math/orientation.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0x40FF40, 100);

    sched_sleep(1000);
    mag_init();
    mpu_init();

    ins_init();
    attitude_init();

    ins_start();
    attitude_start_from_triad();

    while (true) {
        AttitudeState state = attitude_get_state();
        VectorF<3> rpy = quat_to_rpy(state.quat);
        uart << rad_to_deg(rpy[0]) << "\t" << rad_to_deg(rpy[1]) << "\t" << rad_to_deg(rpy[2]) << endl;

        sched_sleep(100);
    } //*/
/*
    int16_t max[3] = {0, 0, 0};
    int16_t min[3] = {0, 0, 0};
    while (true) {
        MagSample mag = mag_sample_averaged(50);
        for (int i=0; i<3; i++) {
            if (mag.field[i] > max[i])
                max[i] = mag.field[i];
            if (mag.field[i] < min[i])
                min[i] = mag.field[i];
            uart << (max[i] - min[i])/2 + min[i] << "\t";
        }
        uart << endl;

}
//*/

/*    while (true) {
        MagSample mag = mag_sample_averaged(10);
        VectorF<3> vec = calibration_mag(mag.field);
        VectorF<3> vec = { mag.field[0], mag.field[1], mag.field[2] };
        uart << norm(vec) << "\t";
        for (int i=0; i<3; i++) {
            uart << vec[i] << "\t";
        }
        uart << endl;
        } */
//*/
/*    while (true) {
        MPUSample mpu = mpu_sample();
        MagSample mag = mag_sample();

        VectorF<3> cal;
        VectorF<3> cala;
        MatrixF<3, 3> rot = triad_algorithm(cala=calibration_accel(mpu.accel), cal=calibration_mag(mag.field));
        VectorF<3> rpy = rot_to_rpy(rot);

        for (int i=0; i<3; i++)
            uart << static_cast<int>(rad_to_deg(rpy[i])) << "\t";

        for (int i=0; i<3; i++)
                 uart << static_cast<int>(cala[i]*1000) << "\t";

        for (int i=0; i<3; i++)
            uart << static_cast<int>(cal[i]) << "\t";

        uart << endl;
        }
*/
/*
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
*/
}
