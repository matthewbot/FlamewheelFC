#include "mission/basestation.h"
#include "mission/messages.h"
#include "nav/attitude.h"
#include "drivers/xbee.h"
#include "kernel/sched.h"
#include "kernel/kernel.h"
#include <stdint.h>

// state
static StatusMsg msg;
static bool valid;

// task
static Task basestation_task;
DECLARE_TASK_STACK(basestation_stack, 4*1024);
DECLARE_TASK_FUNC(basestation_func);

// helper functions
static int16_t float16(float f, int e);
static void send_status_message();

void basestation_init() {
    basestation_task.setup("basestation", Task::LOW, basestation_func, nullptr, basestation_stack, sizeof(basestation_stack));
    KernelCriticalSection crit;
    sched_add_task(basestation_task);
}

static void send_status_message() {
    AttitudeDebugState attstate = attitude_get_debug_state();
    VectorF<3> rpy = quat_to_rpy(attstate.quat);

    msg.id = MSGID_STATUS;

    msg.roll = float16(rpy[0], 4);
    msg.pitch = float16(rpy[1], 4);
    msg.yaw = float16(rpy[2], 4);

    msg.roll_bias = float16(attstate.bias_gyro[0], 4);
    msg.pitch_bias = float16(attstate.bias_gyro[1], 4);
    msg.yaw_bias = float16(attstate.bias_gyro[2], 4);

    msg.accel_x_bias = float16(attstate.bias_accel[0], 4);
    msg.accel_y_bias = float16(attstate.bias_accel[1], 4);
    msg.accel_z_bias = float16(attstate.bias_accel[2], 4);

    XBeeSendResponse resp = xbee_send(1, reinterpret_cast<const char *>(&msg), sizeof(msg));
    valid = (resp == XBeeSendResponse::SUCCESS);
}

static int16_t float16(float f, int e) {
    float val = f * pow(10, e);
    if (val > INT16_MAX)
        return INT16_MAX;
    else if (val < INT16_MIN)
        return INT16_MIN;
    else
        return static_cast<int>(val);
}

void basestation_func(void *unused) {
    sched_sleep(5000); // give the XBee time to initialize
    while (true) {
        sched_sleep(500);
        send_status_message();
    }
}
