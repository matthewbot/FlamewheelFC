#include "drivers/uart.h"
#include "drivers/util.h"
#include "math/int.h"
#include "kernel/kernel.h"
#include <stdlib.h>
#include <stm32f4xx.h>

static constexpr int PIN_TX = 9;
static constexpr int PIN_RX = 10;
static constexpr int AF_USART1 = 7;

static constexpr int baud = 1152000;
static constexpr float bauddiv = 84e6 / (baud*16);
static constexpr uint32_t brr = bauddiv*16 + 0.5f;

static char buf[256];
static uint8_t readpos;
static uint8_t writepos;
static uint8_t count;

void uart_init() {
	// enable clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	__DMB();

	// set up USART
	USART1->BRR = brr;
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	util_enable_irq(USART1_IRQn, 0xFF);

	// set up GPIOs
	GPIOA->AFR[1] |= AFRH(PIN_TX, AF_USART1) | AFRH(PIN_RX, AF_USART1);
	GPIOA->MODER |= MODER_AF(PIN_TX) | MODER_AF(PIN_RX);
}

void uart_puts(const char *out) {
	KernelCriticalSection crit;
	while (*out != '\0' && count < sizeof(buf)) {
		buf[writepos] = *out++;
		writepos = (writepos+1) % sizeof(buf);
		count++;
	}
	USART1->CR1 |= USART_CR1_TXEIE;
}

void uart_putint(int i) {
	char buf[16];
	char *pos = itoan(i, buf, sizeof(buf));
	uart_puts(pos);
}

extern "C" void irq_usart1() {
	uint32_t sr = USART1->SR;
	if (sr & USART_SR_TXE) {
		if (readpos == writepos) {
			USART1->CR1 &= ~USART_CR1_TXEIE;
			return;
		}

		USART1->DR = buf[readpos];
		readpos = (readpos+1) % sizeof(buf);
		count--;

		if (readpos == writepos)
			USART1->CR1 &= ~USART_CR1_TXEIE;
	}
}
