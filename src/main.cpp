#include "mission/controlpanel.h"
#include "mission/basestation.h"
#include "nav/ins.h"
#include "nav/inscomp.h"
#include "nav/controller.h"
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
    rgbled_set(0xFF8000, 100);
    spektrum_init();
    xbee_init();
    mag_init();
    mpu_init();
    ins_init();
    inscomp_init();
    controller_init();
    basestation_init();

    controlpanel_run();
}
