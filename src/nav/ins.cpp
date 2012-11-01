#include "nav/ins.h"
#include "nav/calibration.h"
#include "drivers/mpu.h"
#include "kernel/sync.h"
#include "kernel/kernel.h"

// state
static Quaternion quat;
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
    ins_task.setup("ins", Task::HIGH, ins_func, nullptr, ins_stack, sizeof(ins_stack));
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
    signal.notify_all();
}

Quaternion ins_get_quaternion() {
    Quaternion ret;
    {
        Lock lock(mutex);
        ret = quat;
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
        Quaternion q;
        VectorF<3> b;
        {
            Lock lock(mutex);
            while (!running)
                signal.wait(lock);
            q = quat;
            b = bias;
        }

        MPUSample sample = mpu_sample();
        VectorF<3> w = calibration_gyro(sample.gyro);
        Quaternion qprime = quat_int(q, w-b, 1e-3);
        qprime = static_cast<float>(1.0/norm(qprime))*qprime;

        {
            Lock lock(mutex);
            quat = qprime;
        }
    }
}
