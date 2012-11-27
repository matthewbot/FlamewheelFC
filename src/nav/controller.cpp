#include "nav/controller.h"
#include "nav/inscomp.h"
#include "nav/calibration.h"
#include "math/orientation.h"
#include "drivers/esc.h"
#include "kernel/sched.h"
#include "kernel/sync.h"
#include "kernel/kernel.h"

// controller state
static ControllerSetpoint setpoint;
static ControllerDebug debug;

// controller gains
static ControllerGains gains;

// config state
static bool running;
static bool manual_thrust;
static Mutex mutex;
static Signal signal;

// task
static Task controller_task;
DECLARE_TASK_STACK(controller_stack, 4*1024);
DECLARE_TASK_FUNC(controller_func);

void controller_init() {
    controller_reset();
    controller_task.setup("controller", Task::HIGH, controller_func, nullptr, controller_stack, sizeof(controller_stack));
    KernelCriticalSection crit;
    sched_add_task(controller_task);
}

void controller_reset() {
    Lock lock(mutex);
    running = false;
    manual_thrust = false;

    gains.p = VectorF<3>{.15, .15, .2};
    gains.d = VectorF<3>{.04, .04, .05};
}

void controller_start() {
    Lock lock(mutex);
    running = true;
    signal.notify_all();
}

void controller_stop() {
    Lock lock(mutex);
    running = false;
    debug.pout = ZeroMatrix<float, 4, 1>();
    debug.dout = ZeroMatrix<float, 4, 1>();
    debug.out = ZeroMatrix<float, 4, 1>();
    debug.motors = ZeroMatrix<float, 4, 1>();
}

bool controller_running() {
    return running;
}

void controller_set(const ControllerSetpoint &new_setpoint) {
    Lock lock(mutex);
    setpoint = new_setpoint;
}

ControllerDebug controller_get_debug() {
    Lock lock(mutex);
    return debug;
}

ControllerGains controller_get_gains() {
    Lock lock(mutex);
    return gains;
}

void controller_set_gains(const ControllerGains &new_gains) {
    Lock lock(mutex);
    gains = new_gains;
}

void controller_func(void *unused) {
    while (true) {
        ControllerSetpoint cur_setpoint;
        ControllerGains cur_gains;

        {
            Lock lock(mutex);
            while (!running)
                signal.wait(lock);
            cur_setpoint = setpoint;
            cur_gains = gains;
        }

        INSCompState state = inscomp_get_state();

        VectorF<4> pout = ZeroMatrix<float, 4, 1>();
        VectorF<4> dout = ZeroMatrix<float, 4, 1>();

        dout.slice<3, 1>(0, 0) = diag(cur_gains.d)*(setpoint.rate_d - state.rate);

        if (cur_setpoint.mode >= ControllerMode::ATTITUDE) {
            Quaternion err = quat_mult(quat_conj(state.quat), setpoint.att_d);
            quat_norm(err);
            float angle;
            VectorF<3> axis = quat_to_axisangle(err, angle);
            if (angle > (float)M_PI) {
                angle -= (float)(2*M_PI);
            }
            pout.slice<3, 1>(0, 0) = angle*diag(cur_gains.p)*axis;
        }

        if (cur_setpoint.mode >= ControllerMode::ALTITUDE) {
            // TODO
        } else {
            pout[3] = cur_setpoint.thrust_d;
        }

        VectorF<4> out = pout + dout;
        VectorF<4> motors = motor_map(out);

        uint16_t pwm[4];
        calibration_esc(motors, pwm, 200);
        esc_set_all(pwm);

        {
            Lock lock(mutex);
            if (running) {
                debug.pout = pout;
                debug.dout = dout;
                debug.out = out;
                debug.motors = motors;
            }
        }

        sched_sleep(2);
    }
}
