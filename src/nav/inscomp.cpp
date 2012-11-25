#include "nav/inscomp.h"
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
static float acc_norm_err;
static int reset_ctr;
static int iteration_ctr;
static bool running;
static bool skip;
static Mutex mutex;
static Signal signal;

// constants
static const float ekf_P_init[] = { 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3, 1e4, 1e4, 1e4 };

// task
static Task inscomp_task;
DECLARE_TASK_STACK(inscomp_stack, 16*1024);
DECLARE_TASK_FUNC(inscomp_func);

void inscomp_init() {
    inscomp_reset();

    inscomp_task.setup("inscomp", Task::HIGH, inscomp_func, nullptr, inscomp_stack, sizeof(inscomp_stack));
    KernelCriticalSection crit;
    sched_add_task(inscomp_task);
}

void inscomp_start_triad() {
    inscomp_stop();
    ins_start_triad();
    inscomp_reset();
    inscomp_start();
}

void inscomp_start() {
    ins_start();
    Lock lock(mutex);
    running = true;
    signal.notify_all();
}

void inscomp_stop() {
    ins_stop();
    running = false;
}

void inscomp_reset() {
    Lock lock(mutex);
    ekf_state.x = ZeroMatrix<float, 9, 1>();
    ekf_state.P = diag(ConstMatrix<float, 9, 1>(ekf_P_init));
    reset_ctr = 0;
    iteration_ctr = 0;
    skip = true;
}

bool inscomp_running() {
    return running;
}

INSCompState inscomp_get_state() {
    INSCompState state;

    Lock lock(mutex);
    state.quat = quat_mult(ins_get_quaternion(), ekf_state.err_quat());
    quat_norm(state.quat);
    state.rate = ins_get_rate() - ekf_state.err_gyro_bias();
    return state;
}

INSCompDebugState inscomp_get_debug_state() {
    INSCompDebugState state;

    Lock lock(mutex);
    state.quat = quat_mult(ins_get_quaternion(), ekf_state.err_quat());
    quat_norm(state.quat);
    state.rate = ins_get_rate() - ekf_state.err_gyro_bias();
    state.bias_gyro = ins_get_rate_bias() + ekf_state.err_gyro_bias();
    state.bias_mag = ekf_state.mag_bias();
    state.acc_norm_err = acc_norm_err;
    return state;
}

void inscomp_func(void *unused) {
    while (true) {
        EKFState new_state;
        {
            Lock lock(mutex);
            while (!running)
                signal.wait(lock);
            new_state = ekf_state;
        }

        int start = sched_now();
        MagSample mag = mag_sample(true);
        Quaternion ins = ins_get_quaternion();
        VectorF<3> rate = ins_get_rate();
        VectorF<3> accel = ins_get_accel();

        float new_acc_norm_err = norm(accel)-9.8f;
        new_acc_norm_err *= new_acc_norm_err;

        bool acc_en = true;
        if (iteration_ctr > 500)
            acc_en = new_acc_norm_err < 0.05f;

        bool ok = attitude_ekf(new_state, rate, accel, calibration_mag(mag.field), ins, true, acc_en, .01);

        if (!ok)
            kernel_halt("attitude_ekf failed");

        {
            Lock lock(mutex);
            if (running && !skip) {
                ekf_state = new_state;
                acc_norm_err = new_acc_norm_err;
                iteration_ctr++;
                if (++reset_ctr >= 100) {
                    ins_correct(ekf_state.err_quat(), ekf_state.err_gyro_bias());
                    ekf_state.x.slice<6, 1>(0, 0) = ZeroMatrix<float, 6, 1>();
                    reset_ctr = 0;
                }
            }
            skip = false;
        }

        sched_sleep(10 - (sched_now() - start));
    }
}
