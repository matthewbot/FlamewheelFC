#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "nav/calibration.h"
#include "nav/ins.h"
#include "nav/attitude.h"
#include "math/orientation.h"

#include "math/attitude_ekf.h"

#include "drivers/uart.h"
#include "kernel/sched.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0x40FF40, 100);

    sched_sleep(1000);
    mag_init();
    mpu_init();

    ins_init();
    attitude_init();

    ins_start();
    attitude_start_from_triad();

    while (true) {
        AttitudeDebugState state = attitude_get_debug_state();
        VectorF<3> rpy = quat_to_rpy(state.quat);
        uart << rad_to_deg(rpy[0]) << "\t" << rad_to_deg(rpy[1]) << "\t" << rad_to_deg(rpy[2]) << "\t";
        uart << state.bias_gyro[0]*1e3f << "\t" << state.bias_gyro[1]*1e3f << "\t" << state.bias_gyro[2]*1e3f << "\t";
        uart << state.bias_accel[0]*1e3f << "\t" << state.bias_accel[1]*1e3f << "\t" << state.bias_accel[2]*1e3f << "\t";

        uart << endl;
        sched_sleep(10);
    }
}
