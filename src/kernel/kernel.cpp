#include "kernel.h"
#include "task.h"
#include "sched.h"
#include "debug.h"
#include <stm32f4xx.h>

static int criticalcount;

void kernel_enter_critical() {
    __set_BASEPRI(KERNEL_IRQ_PRIORITY);
    criticalcount++;
}

void kernel_leave_critical() {
    if (--criticalcount == 0)
        __set_BASEPRI(0);
}

void kernel_halt(const char *msg) {
    debug_init();
    __disable_irq();

    while (true) {
        debug_puts("HALTED\r\n");
        debug_puts(msg);
        debug_puts("\r\n");
        for (int i=0; i<3; i++)
            debug_setLED(i, true);
        debug_delay(10000);
        for (int i=0; i<3; i++)
            debug_setLED(i, false);
        debug_delay(10000);
    }
}
