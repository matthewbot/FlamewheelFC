#include "mission/controlpanel.h"
#include "nav/calibration.h"
#include "nav/ins.h"
#include "nav/inscomp.h"
#include "math/orientation.h"
#include "drivers/uart.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "drivers/spektrum.h"
#include "drivers/esc.h"
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
        } else if (strcmp(buf, "inscomp start") == 0) {
            inscomp_start();
            uart << "Inscomp started" << endl;
        } else if (strcmp(buf, "inscomp stop") == 0) {
            inscomp_stop();
            uart << "Inscomp stopped" << endl;
        } else if (strcmp(buf, "inscomp reset") == 0) {
            inscomp_reset();
            uart << "Inscomp reset" << endl;
        } else if (strcmp(buf, "inscomp triad") == 0) {
            inscomp_start_triad();
            uart << "Inscomp started from triad" << endl;
        } else if (strcmp(buf, "inscomp") == 0) {
            controlpanel_inscomp();
        } else if (strcmp(buf, "spektrum bind") == 0) {
            spektrum_bind();
            uart << "Entered bind mode" << endl;
        } else if (strcmp(buf, "spektrum") == 0) {
            controlpanel_spektrum();
        } else if (strcmp(buf, "esc test") == 0) {
            controlpanel_esctest();
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
            uart << mpusample.accel[i] << '\t';
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
        VectorF<3> accel = ins_get_accel();

        dump_rpy(quat_to_rpy(quat));
        dump_vec(rate, 1000);
        dump_vec(accel, 1000);
        uart << endl;
        sched_sleep(1);
    }
    uart_getch();
    uart << endl;
}

void controlpanel_inscomp() {
    inscomp_start();

    while (!uart_avail()) {
        INSCompDebugState state = inscomp_get_debug_state();

        dump_rpy(quat_to_rpy(state.quat));
        dump_vec(state.bias_gyro, 1000);
        dump_vec(state.bias_accel, 1000);
        uart << state.acc_norm_err*1e6f;
        uart << endl;
        sched_sleep(1);
    }

    uart_getch();
    uart << endl;
}

void controlpanel_spektrum() {
    bool valid=true;
    while (!uart_avail()) {
        if (!spektrum_valid()) {
            if (valid) {
                valid = false;
                uart << "Lost signal" << endl;
            }
        } else {
            valid = true;
            SpektrumSample sample = spektrum_sample(false);

            uart << "N " << sample.headernum << '\t';
            for (int i=0; i<8; i++)
                uart << sample.channel[i] << '\t';
            uart << endl;
        }

        sched_sleep(50);
    }

    uart_getch();
    uart << endl;
}

void controlpanel_esctest() {
    int escnum = -1;
    bool selectmsg = false;

    while (!uart_avail() && spektrum_valid()) {
        SpektrumSample speksample = spektrum_sample(false);
        int roll = speksample.channel[1];
        int pitch = speksample.channel[2];
        int throttle = speksample.channel[0];
        int yaw = speksample.channel[3];

        if (escnum == -1) {
            if (!selectmsg) {
                uart << "Select an ESC with the right stick" << endl;
                selectmsg = true;
            }
            if (roll > 800) {
                if (pitch > 800) {
                    escnum = (int)ESC::FRONT_LEFT;
                } else if (pitch < 200) {
                    escnum = (int)ESC::REAR_LEFT;
                }
            } else if (roll < 200) {
                if (pitch > 800) {
                    escnum = (int)ESC::FRONT_RIGHT;
                } else if (pitch < 200) {
                    escnum = (int)ESC::REAR_RIGHT;
                }
            }

            if (escnum != -1) {
                uart << "ESC " << escnum << " selected" << endl;
                sched_sleep(1000);
                selectmsg = false;
            }
        } else {
            if (yaw < 200 || yaw > 800) {
                uart << "ESC " << escnum << " disarmed" << endl;

                esc_off(escnum);
                escnum = -1;
            } else {
                int pwm = static_cast<int>((throttle-200.0f)/630.0f * 1500);
                if (pwm < 0)
                    pwm = 0;
                else if (pwm > 1500)
                    pwm = 1500;
                uart << "PWM " << pwm << endl;
                esc_set(escnum, pwm);
            }
        }

        sched_sleep(20);
    }

    esc_all_off();
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

