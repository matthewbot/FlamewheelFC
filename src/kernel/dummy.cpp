#include <stm32f4xx.h>
#include <stdint.h>
#include "sched.h"

static void delay() {
	for (int i=0; i<0x00FFFFFF; i++) {
		__asm("nop");
	}
}

int main() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER = 0x55000000; // configure PD12-PD15 as outputs
	const uint32_t pinmask = (1 << 13) | (1 << 15);
	while (true) {
		GPIOD->BSRRL = pinmask; // turn on pins
		sched_sleep(1000);
		GPIOD->BSRRH = pinmask; // turn off pins
		sched_sleep(500);
	}
}
