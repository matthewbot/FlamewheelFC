#include "mission/controlpanel.h"
#include "mission/basestation.h"
#include "nav/ins.h"
#include "nav/attitude.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/mag.h"
#include "drivers/mpu.h"
#include "drivers/spektrum.h"
#include "drivers/esc.h"
#include "drivers/xbee.h"
#include <stdint.h>
#include <stm32f4xx.h>

int main() {
    uart_init();
    esc_init();
    rgbled_init();
    rgbled_set(0x40FF40, 100);
    spektrum_init();
    xbee_init();
    mag_init();
    mpu_init();
    ins_init();
    attitude_init();
    basestation_init();

    controlpanel_run();
}
