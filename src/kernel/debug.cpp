#include "kernel/debug.h"
#include <stm32f4xx.h>

static const int baud = 115200;
static const float bauddiv = 84e6 / (baud*16);
static const uint32_t brr = bauddiv*16 + 0.5f;

#define RCC_AHB1ENR_CCMDATARAMEN (1 << 20)

void debug_init() {
	RCC->AHB1ENR = RCC_AHB1ENR_CCMDATARAMEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB2ENR = RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_USART1EN;
	__DMB();

	GPIOA->AFR[1] = 0x00000770;
	GPIOA->AFR[0] = 0x00000000;
	GPIOA->ODR = 0x0000;
	GPIOA->MODER = 0xA8280000;
	GPIOB->AFR[1] = 0x00000000;
	GPIOB->AFR[0] = 0x00000000;
	GPIOB->ODR = 0x0032;
	GPIOB->MODER = 0x00000524;

	USART1->BRR = brr;
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void debug_setLED(int led, bool on) {
	static const uint8_t pins[] = {1, 4, 5};
	const uint32_t mask = 1 << pins[led];
	if (on)
		GPIOB->BSRRH = mask;
	else
		GPIOB->BSRRL = mask;
}

void debug_putch(char ch) {
	while (!(USART1->SR & USART_SR_TXE)) { }
	USART1->DR = ch;
}

void debug_puts(const char *str) {
	while (*str)
		debug_putch(*str++);
}

void debug_puthex(uint32_t num) {
	static const char hexnums[] = "0123456789ABCDEF";

	debug_puts("0x");
	for (int i=7; i>=0; i--) {
		int shiftamt = i*4;
		int bits = (num & (0xF << shiftamt)) >> shiftamt;
		debug_putch(hexnums[bits]);
	}
}

struct Frame {
	uint32_t r4, r5, r6, r7, r8, r9, r10, r11;
	uint32_t r0, r1, r2, r3, r12;
	uint32_t lr;
	uint32_t ip;
	uint32_t psr;
};

static void body(Frame &frame) __attribute__((naked, noreturn));
static void printreg(const char *name, uint32_t val);

extern "C" void handler_fault() __attribute__((naked, noreturn));
extern "C" void handler_fault() {
	asm volatile("cpsid i;"
	             "tst lr, 0b0100;"
	             "ittee ne;"
	             "mrsne r0, psp;"
	             "stmdbne r0!, {r4-r11};"
	             "stmdbeq sp!, {r4-r11};"
	             "moveq r0, sp;"
	             "b %[body];"
	             ::
	              [body] "i" (&body));
	__builtin_unreachable();
}

static void body(Frame &frame) {
	debug_init();

	while (true) {
		debug_puts("Fault\r\n");
		printreg("ICSR ", SCB->ICSR);
		printreg("SHCSR ", SCB->SHCSR);
		printreg("CFSR ", SCB->CFSR);
		printreg("HFSR ", SCB->HFSR);
		printreg("BFAR ", SCB->BFAR);
		printreg("MMFAR ", SCB->MMFAR);
		printreg("IP ", frame.ip);
		printreg("R0 ", frame.r0);
		printreg("R1 ", frame.r1);
		printreg("R2 ", frame.r2);
		printreg("R3 ", frame.r3);
		printreg("R4 ", frame.r4);
		printreg("R5 ", frame.r5);
		printreg("R6 ", frame.r6);
		printreg("R7 ", frame.r7);
		printreg("R8 ", frame.r8);
		printreg("R9 ", frame.r9);
		printreg("R10 ", frame.r10);
		printreg("R11 ", frame.r11);
		printreg("R12 ", frame.r12);
		printreg("LR ", frame.lr);
		printreg("PSR ", frame.psr);
		debug_delay(10000000);
	}
}

static void printreg(const char *name, uint32_t val) {
	debug_puts(name);
	debug_puthex(val);
	debug_puts("\r\n");
}

void debug_delay(long cycles) {
	while (cycles--)
		asm volatile("nop");
}
