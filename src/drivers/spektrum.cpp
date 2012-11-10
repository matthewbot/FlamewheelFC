#include "drivers/spektrum.h"
#include "drivers/stm32f4xx_exts.h"
#include "drivers/util.h"
#include "kernel/sync.h"
#include "kernel/sched.h"

// pin constants
static constexpr int PIN_TX = 12;
static constexpr int PIN_RX = 2;
static constexpr int AF_UART5 = 8;

// timer constants
static constexpr TIM_TypeDef *tim = TIM5;

// uart constants
static constexpr USART_TypeDef *usart = UART5;
static constexpr uint32_t brr = UART_BRR(42e6, 115200);

// state
static Signal signal;
static SpektrumSample sample;
static uint32_t sampletick;
static uint8_t buf[16];
static int bufpos;

static int bindctr;
static bool bindstate;

static void setup_recv();
static void setup_bind();

void spektrum_init() {
    // enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN | RCC_APB1ENR_TIM5EN;
    __DMB();

    // set up timer
    tim->ARR = 1000;
    tim->DIER = TIM_DIER_UIE;
    util_enable_irq(TIM5_IRQn, IRQ_PRI_KERNEL);

    // set up USART
    usart->BRR = brr;
    usart->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    util_enable_irq(UART5_IRQn, IRQ_PRI_MED);

    // set up GPIOs
    GPIOC->AFR[1] |= AFRH(PIN_TX, AF_UART5);
    GPIOC->MODER |= MODER_AF(PIN_TX);
    GPIOD->AFR[0] |= AFRL(PIN_RX, AF_UART5);

    setup_recv();
}

static void setup_recv() {
    tim->PSC = 84e6 / (1000 * 1e3);
    tim->CR1 = TIM_CR1_OPM;
    GPIOD->MODER &= ~MODER_MASK(PIN_RX);
    GPIOD->MODER |= MODER_AF(PIN_RX);
}

static void setup_bind() {
    tim->PSC = 84e6 / (1000 * 8.3333333e3);
    tim->CR1 = 0;
    GPIOD->MODER &= ~MODER_MASK(PIN_RX);
    GPIOD->MODER |= MODER_OUT(PIN_RX);
}

SpektrumSample spektrum_sample(bool block) {
    if (block)
        signal.wait();

    KernelCriticalSection crit;
    return sample;
}

bool spektrum_valid() {
    return sched_now() - sampletick < 100;
}

extern "C" void irq_uart5() {
    uint32_t sr = usart->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t val = usart->DR;
        if (bufpos < sizeof(buf))
            buf[bufpos++] = val;
        tim->CNT = 0;
        tim->CR1 |= TIM_CR1_CEN;
    }
}

static void irq_tim5_recv() {
    for (int i=0; i<8; i++)
        sample.channel[i] = -1;

    if (bufpos == 16) {
        for (int i=2; i<bufpos; i+=2) {
            uint16_t val = (buf[i] << 8) | buf[i+1];
            unsigned int chan = val >> 10;
            val &= 0x3FF;
            if (chan < 8)
                sample.channel[chan] = val;
        }
        sample.headernum = buf[1];
        sampletick = sched_now();
    }

    signal.notify_all();
    bufpos = 0;
}

void spektrum_bind() {
    setup_bind();
    bindctr = 6;
    tim->CR1 |= TIM_CR1_CEN;
}

static void irq_tim5_bind() {
    bindstate = !bindstate;
    if (bindstate) {
        GPIOD->BSRRL = (1 << PIN_RX);
    } else {
        GPIOD->BSRRH = (1 << PIN_RX);
        if (--bindctr == 0)
            setup_recv();
    }
}

extern "C" void irq_tim5() {
    tim->SR = 0;
    if (bindctr > 0)
        irq_tim5_bind();
    else
        irq_tim5_recv();
}
