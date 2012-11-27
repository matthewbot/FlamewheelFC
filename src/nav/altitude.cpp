#include "nav/altitude.h"
#include "nav/calibration.h"
#include "drivers/sonar.h"
#include "kernel/sync.h"
#include "kernel/kernel.h"

// altitude state
static float altitude;
static float rate;

// config state
static bool running;
static Mutex mutex;
static Signal signal;

// task
static Task altitude_task;
DECLARE_TASK_STACK(altitude_stack, 4*1024);
DECLARE_TASK_FUNC(altitude_func);

void altitude_init() {
    altitude_task.setup("altitude", Task::HIGH, altitude_func, nullptr, altitude_stack, sizeof(altitude_stack));
    KernelCriticalSection crit;
    sched_add_task(altitude_task);
}

void altitude_start() {
    Lock lock(mutex);
    running = true;
    signal.notify_all();
}

void altitude_stop() {
    Lock lock(mutex);
    running = false;
}

float altitude_get() {
    Lock lock(mutex);
    return altitude;
}

float altitude_get_rate() {
    Lock lock(mutex);
    return rate;
}

static void altitude_func(void *unused) {
    float prev_altitude = -calibration_sonar(sonar_sample(true));
    int prev_time = sched_now();

    while (true) {
        {
            Lock lock(mutex);
            if (!running)
                signal.wait(lock);
        }

        uint16_t sample = sonar_sample(true);
        float new_altitude = -calibration_sonar(sample);
        int new_time = sched_now();
        float new_rate = (new_altitude - prev_altitude) / (new_time - prev_time) * 1000;

        Lock lock(mutex);
        altitude = new_altitude;
        rate = new_rate;
        prev_altitude = new_altitude;
        prev_time = new_time;
    }
}
