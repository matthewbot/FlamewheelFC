#include "nav/ins.h"
#include "nav/calibration.h"
#include "drivers/mpu.h"
#include "kernel/sync.h"
#include "kernel/kernel.h"

// nav state
static Quaternion quat;
static VectorF<3> rate;
static VectorF<3> rate_bias;
static VectorF<3> accel;

// filter state
static constexpr int filter_len = 128;
static int16_t filter_accel[filter_len][3];
static int16_t filter_gyro[filter_len][3];
static int filter_pos;

// config state
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

VectorF<3> ins_get_accel() {
    VectorF<3> ret;
    {
        Lock lock(mutex);
        ret = accel;
    }
    return ret;
}

VectorF<3> ins_get_rate_bias() {
    VectorF<3> ret;
    {
        Lock lock(mutex);
        ret = rate_bias;
    }
    return ret;
}

void ins_correct(const Quaternion &quat_err, const VectorF<3> &gyro_bias_err) {
    Lock lock(mutex);
    quat = quat_mult(quat, quat_err);
    quat_norm(quat);
    rate_bias = rate_bias + gyro_bias_err;
    skip = true;
}

void ins_reset() {
   Lock lock(mutex);
   quat = {1, 0, 0, 0};
   rate_bias = {0, 0, 0};
   skip = true;
}

void ins_reset(const Quaternion &new_quat, const VectorF<3> &new_rate_bias) {
    Lock lock(mutex);
    quat = new_quat;
    rate_bias = new_rate_bias;
    skip = true;
}

void ins_func(void *unused) {
    while (true) {
        VectorF<3> cur_rate_bias;
        {
            Lock lock(mutex);
            while (!running)
                signal.wait(lock);
            cur_rate_bias = rate_bias;
        }

        MPUSample sample = mpu_sample();
        for (int i=0; i<3; i++) {
            filter_accel[filter_pos][i] = sample.accel[i];
            filter_gyro[filter_pos][i] = sample.gyro[i];
        }
        filter_pos = (filter_pos+1) % filter_len;

        int accel_sum[3] = { 0, 0, 0 };
        int gyro_sum[3] = { 0, 0, 0 };
        for (int f=0; f<filter_len; f++) {
            for (int i=0; i<3; i++) {
                accel_sum[i] += filter_accel[f][i];
                gyro_sum[i] += filter_gyro[f][i];
            }
        }

        int16_t accel_filt[3];
        int16_t gyro_filt[3];
        for (int i=0; i<3; i++) {
            accel_filt[i] = accel_sum[i] / filter_len;
            gyro_filt[i] = gyro_sum[i] / filter_len;
        }
        VectorF<3> new_accel = calibration_accel(accel_filt);
        VectorF<3> new_rate = calibration_gyro(gyro_filt) - cur_rate_bias;

        VectorF<3> raw_rate = calibration_gyro(sample.gyro)-cur_rate_bias;
        Quaternion new_quat = quat_int(quat, raw_rate, 1e-3);
        quat_norm(new_quat);

        Lock lock(mutex);
        if (running && !skip) {
            rate = new_rate;
            accel = new_accel;
            quat = new_quat;
        }
        skip = false;
    }
}
