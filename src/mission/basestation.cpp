#include "mission/basestation.h"
#include "mission/messages.h"
#include "nav/inscomp.h"
#include "nav/controller.h"
#include "nav/altitude.h"
#include "drivers/xbee.h"
#include "drivers/board.h"
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
    INSCompDebugState attstate = inscomp_get_debug_state();
    VectorF<3> rpy = quat_to_rpy(attstate.quat);
    ControllerDebug controllerdebug = controller_get_debug();
    ControllerGains controllergains = controller_get_gains();

    msg.id = MSGID_STATUS;

    msg.roll = float16(rpy[0], 4);
    msg.pitch = float16(rpy[1], 4);
    msg.yaw = float16(rpy[2], 4);

    msg.roll_rate = float16(attstate.rate[0], 4);
    msg.pitch_rate = float16(attstate.rate[1], 4);
    msg.yaw_rate = float16(attstate.rate[2], 4);

    msg.roll_bias = float16(attstate.bias_gyro[0], 4);
    msg.pitch_bias = float16(attstate.bias_gyro[1], 4);
    msg.yaw_bias = float16(attstate.bias_gyro[2], 4);

    msg.roll_p = float16(controllerdebug.pout[0], 4);
    msg.pitch_p = float16(controllerdebug.pout[1], 4);
    msg.yaw_p = float16(controllerdebug.pout[2], 4);

    msg.roll_d = float16(controllerdebug.dout[0], 4);
    msg.pitch_d = float16(controllerdebug.dout[1], 4);
    msg.yaw_d = float16(controllerdebug.dout[2], 4);

    msg.gain_roll_p = float16(controllergains.p[0], 4);
    msg.gain_pitch_p = float16(controllergains.p[1], 4);
    msg.gain_yaw_p = float16(controllergains.p[2], 4);

    msg.gain_roll_d = float16(controllergains.d[0], 4);
    msg.gain_pitch_d = float16(controllergains.d[1], 4);
    msg.gain_yaw_d = float16(controllergains.d[2], 4);

    msg.mag_x_bias = float16(attstate.bias_mag[0], 4);
    msg.mag_y_bias = float16(attstate.bias_mag[1], 4);
    msg.mag_z_bias = float16(attstate.bias_mag[2], 4);

    msg.esc_fl = float16(controllerdebug.motors[0], 4);
    msg.esc_fr = float16(controllerdebug.motors[1], 4);
    msg.esc_rr = float16(controllerdebug.motors[2], 4);
    msg.esc_rl = float16(controllerdebug.motors[3], 4);

    msg.altitude = float16(altitude_get(), 3);
    msg.altitude_rate = float16(altitude_get_rate(), 3);

    msg.battery = float16(board_get_voltage(), 3);

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
        int start = sched_now();
        send_status_message();
        sched_sleep(50 - (sched_now() - start));

        if (xbee_receive_avail()) {
            char buf[128];
            XBeeReceiveHeader header;
            int got = xbee_receive(buf, header);
            if (buf[0] == MSGID_GAINS) {
                const GainsMessage &msg = *(GainsMessage *)buf;

                ControllerGains gains;
                gains.p = (1e-4f) * VectorF<3>{ (float)msg.roll_p, (float)msg.pitch_p, (float)msg.yaw_p };
                gains.d = (1e-4f) * VectorF<3>{ (float)msg.roll_d, (float)msg.pitch_d, (float)msg.yaw_d };
                controller_set_gains(gains);
            }
        }
    }
}
