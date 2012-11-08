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

#include "math/attitude_ekf.h"

#include "drivers/uart.h"
#include "kernel/sched.h"

template <typename T>
void dumpmat(const T &mat, float scale) {
    for (int r=0; r<mat.rows(); r++) {
        for (int c=0; c<mat.cols(); c++) {
            uart << mat(r, c)*scale << "\t";
        }
        uart << endl;
        sched_sleep(1);
    }
    uart << endl;
}

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
        //VectorF<3> rpy = quat_to_rpy(ins_get_quaternion());
        uart << rad_to_deg(rpy[0]) << "\t" << rad_to_deg(rpy[1]) << "\t" << rad_to_deg(rpy[2]) << "\t";

        VectorF<3> bias = ins_get_bias();
        uart << bias[0]*100 << "\t" << bias[1]*1000 << "\t" << bias[2]*1000;

        uart << endl;
        sched_sleep(100);
        }//*/
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
        MagSample magsample = mag_sample_averaged(10);
        MPUSample mpusample = mpu_sample_averaged(10);
        VectorF<3> mag = calibration_mag(magsample.field);
        VectorF<3> accel = calibration_accel(mpusample.accel);
        VectorF<3> gyro = calibration_gyro(mpusample.gyro);
        for (int i=0; i<3; i++) {
            uart << mag[i] << "\t";
        }
        for (int i=0; i<3; i++) {
            uart << accel[i] << "\t";
        }
        for (int i=0; i<3; i++) {
            uart << gyro[i]*1e3 << "\t";
        }
        uart << endl;
    }
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

/*    while (true) {
        MPUSample mpusample = mpu_sample();
        MagSample magsample = mag_sample(false);

        VectorF<3> accel = calibration_accel(mpusample.accel);
        VectorF<3> gyro = calibration_gyro(mpusample.gyro);
        VectorF<3> mag = calibration_mag(magsample.field);


        for (int i=0; i<3; i++)
            uart << static_cast<int>(accel[i]*1000) << "\t";

        for (int i=0; i<3; i++)
            uart << static_cast<int>(gyro[i]*1000) << "\t";

        for (int i=0; i<3; i++)
            uart << static_cast<int>(mag[i]) << "\t";

        uart << endl;
    }
//*/

/*    ins_init();
    MPUSample sample = mpu_sample_averaged(1000);
    VectorF<3> bias = calibration_gyro(sample.gyro);
    ins_reset({1, 0, 0, 0}, bias);
    ins_start();

    while (true) {
        sched_sleep(25);
        Quaternion quat = ins_get_quaternion();
        VectorF<3> rpy = quat_to_rpy(quat);
        for (int i=0; i<3; i++)
            uart << (int)(rad_to_deg(rpy[i])) << "\t";
        uart << endl;
        }//*/

/*    EKFState state;
    state.x = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    for (int r=0; r<9; r++) {
        for (int c=0; c<9; c++)
            state.P(r, c) = (r==c) ? 10 : 1;
    }

    VectorF<3> y_g = { 1, 1, 1 };
    VectorF<3> y_a = { 2, 2, 2 };
    VectorF<3> y_m = { 3, 3, 3 };
    Quaternion q_ins = { 1, 0, 0, 0 };

    EKFState newstate = attitude_ekf(state, y_g, y_a, y_m, q_ins, true, true, .01);

    while (true) { }//*/

}
