#include <stm32f4xx.h>
#include <stdint.h>
#include "kernel/kernel.h"
#include "drivers/uart.h"
#include "drivers/rgbled.h"
#include "drivers/xbee.h"

int main() {
    uart_init();
    rgbled_init();
    rgbled_set(0xFF00A0, 100);
    xbee_init();

    while (true) {
        uart << endl << "Waiting for message" << endl;

        char buf[120];
        XBeeReceiveHeader header;
        int got = xbee_receive(buf, header);
        uart << "Got " << got << " from " << header.addr << endl;
        buf[got] = '\0';
        uart << "Msg: " << buf << endl;

        uart << "Sleeping" << endl;
        sched_sleep(2000);

        uart << "Sending" << endl;
        xbee_send(1, buf, got);
    }
}
