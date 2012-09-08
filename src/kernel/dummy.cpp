#include <stm32f4xx.h>
#include <stdint.h>
#include "sched.h"
#include "kernel.h"

static void delay() {
	for (int i=0; i<0x00FFFFFF; i++) {
		asm("nop");
	}
}

DECLARE_TASK_STACK(test_stack, 1024);
static Task test;
static void test_func(void *unused);

int main() {
	test.setup("test", Task::LOW, test_func, NULL, test_stack, sizeof(test_stack));
	{
		KernelCriticalSection crit;
		sched_add_task(test);
	}

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	__DMB();
	GPIOD->MODER = 0x55000000; // configure PD12-PD15 as outputs

	const uint32_t pinmask = (1 << 13) | (1 << 15);
	while (true) {
		GPIOD->BSRRL = pinmask; // turn on pins
		sched_sleep(1000);
		GPIOD->BSRRH = pinmask; // turn off pins
		sched_sleep(500);
	}
}

static void test_func(void *) {
	while (true) {
		GPIOD->BSRRL = (1 << 12);
		sched_sleep(150);
		GPIOD->BSRRH = (1 << 12);
		sched_sleep(150);
	}
}
