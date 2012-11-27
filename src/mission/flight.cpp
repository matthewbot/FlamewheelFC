#include "mission/flight.h"
#include "nav/controller.h"
#include "nav/calibration.h"
#include "nav/inscomp.h"
#include "drivers/board.h"
#include "drivers/spektrum.h"
#include "drivers/rgbled.h"
#include "drivers/esc.h"
#include "math/orientation.h"
#include "kernel/task.h"
#include "kernel/kernel.h"

// state
static float heading;
static bool enabled;

// task
static Task flight_task;
DECLARE_TASK_STACK(flight_stack, 2*1024);
DECLARE_TASK_FUNC(flight_func);

static void wait_arm();
static void beep();
static void fly();

void flight_init() {
    flight_task.setup("flight", Task::MED, flight_func, nullptr, flight_stack, sizeof(flight_stack));
    if (board_switch()) {
        KernelCriticalSection crit;
        sched_add_task(flight_task);
        enabled = true;
    }
}

bool flight_enabled() {
    return enabled;
}

static void flight_func(void *unused) {
    while (true) {
        rgbled_set(0xFF0000, 100);
        wait_arm();

        rgbled_set(0xFF6000, 100);
        beep();
        inscomp_start_triad();

        rgbled_set(0x30FF30, 100);
        fly();
    }
}

static void wait_arm() {
    int armsamples=0;
    bool allow_arm = false;

    while (armsamples < 5) {
        sched_sleep(100);
        if (!spektrum_valid())
            continue;

        SpektrumSample sample = spektrum_sample(false);
        VectorF<4> vec = calibration_spektrum(sample);
        if (vec[3] < .05f && fabsf(vec[2]) > .95f) {
            if (allow_arm)
                armsamples++;
        } else {
            allow_arm = true;
            armsamples = 0;
        }
    }
}

static const uint16_t tones[] = { 1046, 1174, 1318, 1397 };

static void beep() {
    for (int i; i<4; i++) {
        board_buzzer(tones[i]);
        sched_sleep(100);
    }
    board_buzzer(0);
}

static void fly() {
    int disarm_cycles=0;
    INSCompState state = inscomp_get_state();
    heading = quat_to_yaw(state.quat);

    while (disarm_cycles < 5) {
        SpektrumSample sample = spektrum_sample();
        VectorF<4> vec = calibration_spektrum(sample);
        if (!spektrum_valid())
            vec = { 0, 0, 0, 0 }; // TODO handle controller dropout better

        if (vec[3] > .1f) {
            disarm_cycles = 0;
            heading += vec[2]*.05f;
            if (heading < -(float)M_PI)
                heading += (float)2*M_PI;
            else if (heading > (float)M_PI)
                heading -= (float)2*M_PI;

            float roll = vec[0]*.8f;
            float pitch = vec[1]*.8f;

            ControllerSetpoint setpoint;
            setpoint.mode = ControllerMode::ATTITUDE;
            setpoint.rate_d = { 0, 0, 0 };
            setpoint.att_d = rpy_to_quat(VectorF<3>{roll, pitch, heading});
            setpoint.thrust_d = vec[3];

            controller_set(setpoint);
            if (!controller_running())
                controller_start();
        } else {
            if (fabsf(vec[2]) > .95f)
                disarm_cycles++;
            else
                disarm_cycles = 0;
            controller_stop();
            esc_all_off();
        }
    }
}


