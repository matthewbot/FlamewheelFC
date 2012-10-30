#include "drivers/uart.h"
#include "drivers/util.h"
#include "drivers/stm32f4xx_exts.h"
#include "math/int.h"
#include "kernel/sched.h"
#include <stdlib.h>

// pin constants
static constexpr int PIN_TX = 9;
static constexpr int PIN_RX = 10;
static constexpr int AF_USART1 = 7;

// uart constants
static constexpr USART_TypeDef *usart = USART1;
static constexpr uint32_t brr = UART_BRR(84e6, 1152000);

static RingBuffer<uint8_t, 128> buf;

void uart_init() {
    // enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    __DMB();

    // set up USART
    usart->BRR = brr;
    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    util_enable_irq(USART1_IRQn, IRQ_PRI_LOW);

    // set up GPIOs
    GPIOA->AFR[1] |= AFRH(PIN_TX, AF_USART1) | AFRH(PIN_RX, AF_USART1);
    GPIOA->MODER |= MODER_AF(PIN_TX) | MODER_AF(PIN_RX);
}

void uart_puts(const char *out) {
    IRQCriticalSection<USART1_IRQn> crit;
    while (*out != '\0') {
        if (buf.full())
            kernel_halt("uart_puts buffer full");
        buf.put(*out++);
    }
    usart->CR1 |= USART_CR1_TXEIE;
}

void uart_putint(int i) {
    char buf[16];
    char *pos = itoan(i, buf, sizeof(buf));
    uart_puts(pos);
}

extern "C" void irq_usart1() {
    if (buf.empty()) {
        usart->CR1 &= ~USART_CR1_TXEIE;
        return;
    }

    usart->DR = buf.get();

    if (buf.empty())
        usart->CR1 &= ~USART_CR1_TXEIE;
}
