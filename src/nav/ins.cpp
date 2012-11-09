#include "nav/ins.h"
#include "nav/calibration.h"
#include "drivers/mpu.h"
#include "kernel/sync.h"
#include "kernel/kernel.h"

// state
static Quaternion quat;
static VectorF<3> rate;
static VectorF<3> bias;
static bool running;
static bool skip;
static Mutex mutex;
static Signal signal;

// task
static Task ins_task;
DECLARE_TASK_STACK(ins_stack, 4*1024);
DECLARE_TASK_FUNC(ins_func);

void ins_init() {
    ins_reset();
    ins_task.setup("ins", Task::HIGH-0x10, ins_func, nullptr, ins_stack, sizeof(ins_stack));
    KernelCriticalSection crit;
    sched_add_task(ins_task);
}

void ins_start() {
    Lock lock(mutex);
    running = true;
    signal.notify_all();
}

void ins_start_triad() {
    MPUSample sample = mpu_sample_averaged(500);
    MagSample magsample = mag_sample_averaged(100);
    VectorF<3> accel = calibration_accel(sample.accel);
    VectorF<3> mag = calibration_mag(magsample.field);
    VectorF<3> gyro = calibration_gyro(sample.gyro);
    Quaternion quat = rot_to_quat(triad_algorithm(accel, mag));
    ins_reset(quat, gyro);
}

void ins_stop() {
    running = false;
}

bool ins_running() {
    return running;
}

Quaternion ins_get_quaternion() {
    Quaternion ret;
    {
        Lock lock(mutex);
        ret = quat;
    }
    return ret;
}

VectorF<3> ins_get_rate() {
    VectorF<3> ret;
    {
        Lock lock(mutex);
        ret = rate;
    }
    return ret;
}

VectorF<3> ins_get_bias() {
    VectorF<3> ret;
    {
        Lock lock(mutex);
        ret = bias;
    }
    return ret;
}

void ins_correct(const Quaternion &quat_err, const VectorF<3> &bias_err) {
    Lock lock(mutex);
    quat = quat_mult(quat, quat_err);
    quat_norm(quat);
    bias = bias + bias_err;
    skip = true;
}

void ins_reset() {
   Lock lock(mutex);
   quat = {1, 0, 0, 0};
   bias = {0, 0, 0};
   skip = true;
}

void ins_reset(const Quaternion &new_quat, const VectorF<3> &new_bias) {
    Lock lock(mutex);
    quat = new_quat;
    bias = new_bias;
    skip = true;
}

void ins_func(void *unused) {
    while (true) {
        {
            Lock lock(mutex);
            while (!running)
                signal.wait(lock);
        }

        MPUSample sample = mpu_sample();

        VectorF<3> newrate = calibration_gyro(sample.gyro)-bias;
        Quaternion newquat = quat_int(quat, rate, 1e-3);
        quat_norm(newquat);

        Lock lock(mutex);
        if (running && !skip) {
            rate = newrate;
            quat = newquat;
        }
        skip = false;
    }
}
