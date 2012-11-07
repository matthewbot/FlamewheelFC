#include "nav/attitude.h"
#include "nav/ins.h"
#include "nav/calibration.h"
#include "math/attitude_ekf.h"
#include "math/orientation.h"
#include "drivers/mpu.h"
#include "kernel/sched.h"
#include "kernel/sync.h"
#include "kernel/kernel.h"

// state
static EKFState ekf_state;
static int reset_ctr;
static bool running;
static Mutex mutex;
static Signal signal;

// constants
static const float ekf_P_init[] = { 1e-1, 1e-1, 1e-1, 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3 };

// task
static Task attitude_task;
DECLARE_TASK_STACK(attitude_stack, 16*1024);
DECLARE_TASK_FUNC(attitude_func);

void attitude_init() {
    attitude_task.setup("attitude", Task::HIGH, attitude_func, nullptr, attitude_stack, sizeof(attitude_stack));
    KernelCriticalSection crit;
    sched_add_task(attitude_task);
}

void attitude_start_from_triad() {
    MPUSample sample = mpu_sample_averaged(500);
    MagSample magsample = mag_sample_averaged(100);
    VectorF<3> accel = calibration_accel(sample.accel);
    VectorF<3> mag = calibration_mag(magsample.field);
    VectorF<3> gyro = calibration_gyro(sample.gyro);
    Quaternion quat = rot_to_quat(triad_algorithm(accel, mag));

    Lock lock(mutex);
    ins_reset(quat, gyro);
    ekf_state.x = ZeroMatrix<float, 9, 1>();
    ekf_state.P = diag(ConstMatrix<float, 9, 1>(ekf_P_init));
    reset_ctr = 0;
    running = true;
    signal.notify_all();
}

void attitude_start() {
    Lock lock(mutex);
    running = true;
    signal.notify_all();
}

void attitude_stop() {
    Lock lock(mutex);
    running = false;
}

AttitudeState attitude_get_state() {
    AttitudeState state;

    Lock lock(mutex);
    Quaternion quat_err;
    quat_err[0] = 1;
    quat_err.slice<3, 1>(1, 0) = ekf_state.x.slice<3, 1>(0, 0);
    state.quat = quat_mult(quat_err, ins_get_quaternion());
    quat_norm(state.quat);
    state.rate = ins_get_rate() - ekf_state.x.slice<3, 1>(3, 0);
    return state;
}

void attitude_func(void *unused) {
    while (true) {
        {
            Lock lock(mutex);
            while (!running)
                signal.wait(lock);
        }

        int start = sched_now();
        MPUSample mpu = mpu_sample();
        MagSample mag = mag_sample();
        Quaternion ins = ins_get_quaternion();
        EKFState new_state = attitude_ekf(ekf_state, calibration_gyro(mpu.gyro), calibration_accel(mpu.accel), calibration_mag(mag.field), ins, true, true, .01);

        {
            Lock lock(mutex);
            ekf_state = new_state;
            if (++reset_ctr >= 100) {
                Quaternion quat_err;
                quat_err[0] = 1;
                quat_err.slice<3, 1>(1, 0) = ekf_state.x.slice<3, 1>(0, 0);
                VectorF<3> bias_err = ekf_state.x.slice<3, 1>(3, 0);
                ekf_state.x.slice<6, 1>(0, 0) = ZeroMatrix<float, 6, 1>();
                ins_correct(quat_err, bias_err);
                reset_ctr = 0;
            }
        }

        sched_sleep(10 - (sched_now() - start));
    }
}
