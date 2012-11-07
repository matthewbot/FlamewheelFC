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
static Mutex mutex;
static Signal signal;

// task
static Task ins_task;
DECLARE_TASK_STACK(ins_stack, 4*1024);
DECLARE_TASK_FUNC(ins_func);

void ins_init() {
    quat = {1, 0, 0, 0};
    ins_task.setup("ins", Task::HIGH-0x10, ins_func, nullptr, ins_stack, sizeof(ins_stack));
    KernelCriticalSection crit;
    sched_add_task(ins_task);
}

void ins_start() {
    Lock lock(mutex);
    running = true;
    signal.notify_all();
}

void ins_stop() {
    Lock lock(mutex);
    running = false;
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
    bias = bias + bias_err;
}

void ins_reset(const Quaternion &new_quat, const VectorF<3> &new_bias) {
    Lock lock(mutex);
    quat = new_quat;
    bias = new_bias;
}

void ins_func(void *unused) {
    while (true) {
        VectorF<3> b;
        {
            Lock lock(mutex);
            while (!running)
                signal.wait(lock);
            b = bias;
        }

        MPUSample sample = mpu_sample();
        VectorF<3> newrate = calibration_gyro(sample.gyro)-b;
        Quaternion newquat = quat_int(quat, newrate, 1e-3);
        quat_norm(newquat);

        Lock lock(mutex);
        rate = newrate;
        quat = newquat;
    }
}
