#include <stm32f4xx.h>
#include <stdint.h>

static void delay() {
	for (int i=0; i<0x00FFFFFF; i++) {
		__asm("nop");
	}
}

void kernel_startup() __attribute__((naked, noreturn));
void kernel_startup() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER = 0x55000000; // configure PD12-PD15 as outputs
	const uint32_t pinmask = (1 << 13) | (1 << 15);
	while (true) {
		GPIOD->BSRRL = pinmask; // turn on pins
		delay();
		GPIOD->BSRRH = pinmask; // turn off pins
		delay();
	}
}
