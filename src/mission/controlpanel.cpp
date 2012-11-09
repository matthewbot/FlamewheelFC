#include "mission/controlpanel.h"
#include "nav/calibration.h"
#include "nav/ins.h"
#include "nav/attitude.h"
#include "math/orientation.h"
#include "drivers/uart.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "kernel/sched.h"
#include <string.h>

static char buf[128];
static void dump_rpy(const VectorF<3> &rpy);
template <typename MatT>
void dump_mat(const MatT &t, int scale);
template <typename VecT>
void dump_vec(const VecT &t, int scale);

void controlpanel_run() {
    while (true) {
        uart << "FC> ";
        uart >> buf;
        if (strcmp(buf, "sensors raw") == 0) {
            controlpanel_sensors_raw();
        } else if (strcmp(buf, "sensors cal") == 0) {
            controlpanel_sensors_cal();
        } else if (strcmp(buf, "triad") == 0) {
            controlpanel_triad();
        } else if (strcmp(buf, "ins start") == 0) {
            ins_start();
            uart << "INS started" << endl;
        } else if (strcmp(buf, "ins triad") == 0) {
            ins_start_triad();
            uart << "INS started from triad" << endl;
        } else if (strcmp(buf, "ins stop") == 0) {
            ins_stop();
            uart << "INS stopped" << endl;
        } else if (strcmp(buf, "ins reset") == 0) {
            ins_reset();
            uart << "INS reset" << endl;
        } else if (strcmp(buf, "ins") == 0) {
            controlpanel_ins();
        } else if (strcmp(buf, "attitude start") == 0) {
            attitude_start();
            uart << "Attitude started" << endl;
        } else if (strcmp(buf, "attitude stop") == 0) {
            attitude_stop();
            uart << "Attitude stopped" << endl;
        } else if (strcmp(buf, "attitude reset") == 0) {
            attitude_reset();
            uart << "Attitude reset" << endl;
        } else if (strcmp(buf, "attitude triad") == 0) {
            attitude_start_triad();
            uart << "Attitude started from triad" << endl;
        } else if (strcmp(buf, "attitude") == 0) {
            controlpanel_attitude();
        } else if (buf[0] != '\0') {
            uart << "unknown command '" << buf << '\'' << endl;
        }
    }
}

void controlpanel_sensors_raw() {
    while (!uart_avail()) {
        MPUSample mpusample = mpu_sample();
        MagSample magsample = mag_sample(false);

        for (int i=0; i<3; i++)
            uart << mpusample.gyro[i] << '\t';
        for (int i=0; i<3; i++)
            uart << mpusample.gyro[i] << '\t';
        for (int i=0; i<3; i++)
            uart << magsample.field[i] << '\t';
        uart << endl;
    }
    uart_getch();
    uart << endl;
}

void controlpanel_sensors_cal() {
    while (!uart_avail()) {
        MPUSample mpusample = mpu_sample();
        MagSample magsample = mag_sample(false);

        VectorF<3> gyro = calibration_gyro(mpusample.gyro);
        VectorF<3> accel = calibration_accel(mpusample.accel);
        VectorF<3> mag = calibration_mag(magsample.field);

        dump_vec(accel, 1000);
        dump_vec(accel, 1000);
        dump_vec(mag, 1);
        uart << endl;
    }
    uart_getch();
    uart << endl;
}

void controlpanel_triad() {
    MPUSample mpusample = mpu_sample_averaged(50);
    MagSample magsample = mag_sample_averaged(50);

    VectorF<3> accel = calibration_accel(mpusample.accel);
    VectorF<3> mag = calibration_mag(magsample.field);

    MatrixF<3, 3> triad = triad_algorithm(accel, mag);
    dump_rpy(rot_to_rpy(triad)); uart << endl;
}

void controlpanel_ins() {
    ins_start();

    while (!uart_avail()) {
        Quaternion quat = ins_get_quaternion();
        VectorF<3> rate = ins_get_rate();
        VectorF<3> bias = ins_get_bias();

        dump_rpy(quat_to_rpy(quat));
        dump_vec(rate, 1000);
        dump_vec(bias, 1000);
        uart << endl;
        sched_sleep(1);
    }
    uart_getch();
    uart << endl;
}

void controlpanel_attitude() {
    attitude_start();

    while (!uart_avail()) {
        AttitudeDebugState state = attitude_get_debug_state();

        dump_rpy(quat_to_rpy(state.quat));
        dump_vec(state.bias_gyro, 1000);
        dump_vec(state.bias_accel, 1000);
        uart << endl;
        sched_sleep(1);
    }

    uart_getch();
    uart << endl;
}

static void dump_rpy(const VectorF<3> &rpy) {
    uart << "R " << rad_to_deg(rpy[0]) << "\tP " << rad_to_deg(rpy[1]) << "\tY " << rad_to_deg(rpy[2]) << '\t';
}

template <typename MatT>
void dump_mat(const MatT &t, int scale) {
    for (int r=0; r<t.rows(); r++) {
        for (int c=0; c<t.cols(); c++)
            uart << t(r, c)*scale << '\t';
        uart << endl;
    }
}

template <typename VecT>
void dump_vec(const VecT &t, int scale) {
    for (int r=0; r<t.rows(); r++) {
        uart << t[r]*scale << '\t';
    }
}

