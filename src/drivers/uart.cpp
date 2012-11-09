#include "drivers/uart.h"
#include "drivers/util.h"
#include "drivers/stm32f4xx_exts.h"
#include "math/int.h"
#include "kernel/sched.h"
#include "kernel/kernel.h"
#include "kernel/sync.h"
#include <stdlib.h>

// pin constants
static constexpr int PIN_TX = 9;
static constexpr int PIN_RX = 10;
static constexpr int AF_USART1 = 7;

// uart constants
static constexpr USART_TypeDef *usart = USART1;
static constexpr uint32_t brr = UART_BRR(84e6, 1152000);

static RingBuffer<uint8_t, 128> txbuf;
static RingBuffer<uint8_t, 128> rxbuf;
static Signal rxbuf_signal;

void uart_init() {
    // enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    __DMB();

    // set up USART
    usart->BRR = brr;
    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    util_enable_irq(USART1_IRQn, IRQ_PRI_KERNEL);

    // set up GPIOs
    GPIOA->AFR[1] |= AFRH(PIN_TX, AF_USART1) | AFRH(PIN_RX, AF_USART1);
    GPIOA->MODER |= MODER_AF(PIN_TX) | MODER_AF(PIN_RX);
}

void uart_putch(char ch) {
    KernelCriticalSection crit;
    if (ch == '\n')
        uart_putch('\r');
    if (!txbuf.full())
        txbuf.put(ch);
    usart->CR1 |= USART_CR1_TXEIE;
}

void uart_puts(const char *out) {
    while (*out != '\0')
        uart_putch(*out++);
}

void uart_putint(int i) {
    char txbuf[16];
    char *pos = itoan(i, txbuf, sizeof(txbuf));
    uart_puts(pos);
}

char uart_getch() {
    char ch;
    while (true) {
        KernelCriticalSection crit;
        if (!rxbuf.empty()) {
            ch = rxbuf.get();
            break;
        }
        rxbuf_signal.wait();
    }

    if (ch == '\r')
        uart_putch('\n');
    else
        uart_putch(ch);
    return ch;
}

void uart_gets(char *buf, size_t cnt) {
    cnt--; // one character for the null
    while (cnt--) {
        char ch = uart_getch();
        if (ch == '\r')
            break;
        *buf++ = ch;
    }
    *buf = '\0';
}

extern "C" void irq_usart1() {
    uint32_t sr = usart->SR;
    usart->SR = 0;

    if (sr & USART_SR_TXE) {
        if (!txbuf.empty())
            usart->DR = txbuf.get();
        else
            usart->CR1 &= ~USART_CR1_TXEIE;
    }
    if (sr & USART_SR_RXNE) {
        uint8_t byte = usart->DR;
        if (!rxbuf.full()) {
            rxbuf.put(byte);
            rxbuf_signal.notify_all();
        }
    }
}
