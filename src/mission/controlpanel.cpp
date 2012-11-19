#include "mission/controlpanel.h"
#include "nav/calibration.h"
#include "nav/ins.h"
#include "nav/inscomp.h"
#include "nav/controller.h"
#include "math/orientation.h"
#include "drivers/uart.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "drivers/spektrum.h"
#include "drivers/esc.h"
#include "drivers/board.h"
#include "kernel/sched.h"
#include <string.h>

static char buf[128];
static void dump_rpy(const VectorF<3> &rpy);
template <typename MatT>
void dump_mat(const MatT &t, int scale);
template <typename VecT>
void dump_vec(const VecT &t, int scale);

static uint16_t throttle_to_pwm(uint16_t throttle);

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
        } else if (strcmp(buf, "spektrum cal") == 0) {
            controlpanel_spektrum_cal();
        } else if (strcmp(buf, "esc test") == 0) {
            controlpanel_esc_test();
        } else if (strcmp(buf, "esc sensors") == 0) {
            controlpanel_esc_sensors();
        } else if (strcmp(buf, "esc inscomp") == 0) {
            controlpanel_esc_inscomp();
        } else if (strcmp(buf, "esc map") == 0) {
            controlpanel_esc_map();
        } else if (strcmp(buf, "controller") == 0) {
            controlpanel_controller();
        } else if (strcmp(buf, "controller test") == 0) {
            controlpanel_controller_test();
        } else if (strcmp(buf, "controller start") == 0) {
            controller_start();
            uart << "controller started" << endl;
        } else if (strcmp(buf, "controller stop") == 0) {
            controller_stop();
            esc_all_off();
            uart << "controller stop" << endl;
        } else if (strcmp(buf, "controller gains") == 0) {
            ControllerGains gains = controller_get_gains();
            uart << "P "; dump_vec(gains.p, 1000); uart << "D "; dump_vec(gains.d, 1000); uart << endl;
        } else if (strcmp(buf, "bat") == 0) {
            uart << board_get_voltage() * 1000 << " mV" << endl;
        } else if (strcmp(buf, "switch") == 0) {
            uart << "switch " << (board_switch() ? "on" : "off") << endl;
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

void controlpanel_spektrum_cal() {
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
            VectorF<4> cal = calibration_spektrum(sample);

            uart << "N " << sample.headernum << '\t';
            dump_vec(cal, 1000);
            uart << endl;
        }

        sched_sleep(50);
    }

    uart_getch();
    uart << endl;
}

void controlpanel_esc_test() {
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
                int pwm = throttle_to_pwm(throttle);
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

void controlpanel_esc_sensors() {
    while (!uart_avail() && spektrum_valid()) {
        MPUSample mpusample = mpu_sample();
        MagSample magsample = mag_sample(false);

        for (int i=0; i<3; i++)
            uart << mpusample.gyro[i] << '\t';
        for (int i=0; i<3; i++)
            uart << mpusample.accel[i] << '\t';
        for (int i=0; i<3; i++)
            uart << magsample.field[i] << '\t';

        SpektrumSample speksample = spektrum_sample(false);
        uint16_t pwm = throttle_to_pwm(speksample.channel[0]);
        for (int i=0; i<4; i++)
            esc_set(i, pwm);

        uart << pwm << endl;
    }

    esc_all_off();
    uart_getch();
    uart << endl;
}

void controlpanel_esc_inscomp() {
    while (!uart_avail() && spektrum_valid()) {
        INSCompDebugState state = inscomp_get_debug_state();

        dump_rpy(quat_to_rpy(state.quat));
        dump_vec(state.bias_gyro, 1000);
        uart << state.acc_norm_err*1e6f << '\t';

        SpektrumSample speksample = spektrum_sample(false);
        int pwm = throttle_to_pwm(speksample.channel[0]);
        for (int i=0; i<4; i++)
            esc_set(i, pwm);

        uart << pwm << endl;
        sched_sleep(10);
    }

    esc_all_off();
    uart_getch();
    uart << endl;
}

void controlpanel_esc_map() {
   while (!uart_avail() && spektrum_valid()) {
        SpektrumSample speksample = spektrum_sample(false);
        VectorF<4> out = calibration_spektrum(speksample);
        out.slice<3, 1>(0, 0) = .25*out.slice<3, 1>(0, 0);
        VectorF<4> motors = motor_map(out);

        uint16_t pwm[4];
        calibration_esc(motors, pwm, 0);

        esc_set_all(pwm);
        for (int i=0; i<4; i++)
            uart << pwm[i] << '\t';
        uart << endl;
        sched_sleep(10);
    }

    esc_all_off();
    uart_getch();
    uart << endl;
}

void controlpanel_controller() {
    controller_start();

    while (!uart_avail()) {
        if (controller_running()) {
            ControllerDebug state = controller_get_debug();

            dump_vec(state.pout, 1000);
            dump_vec(state.dout, 1000);
            dump_vec(state.motors, 1000);
            uart << endl;
        } else {
            uart << "Controller disabled" << endl;
        }
        sched_sleep(2);
    }
    controller_stop();

    uart_getch();
    uart << endl;
}

void controlpanel_controller_test() {
    ControllerSetpoint set;
    set.mode = ControllerMode::ATTITUDE;
    set.rate_d = ZeroMatrix<float, 3, 1>();
    set.att_d = Quaternion{ 1, 0, 0, 0 };
    set.thrust_d = .25;
    controller_set(set);

    controlpanel_controller();
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

static uint16_t throttle_to_pwm(uint16_t throttle) {
    int pwm = static_cast<int>((throttle-200.0f)/630.0f * 1500);
    if (pwm < 0)
        pwm = 0;
    else if (pwm > 1500)
        pwm = 1500;
    return pwm;
}
