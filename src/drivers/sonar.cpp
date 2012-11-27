#include "drivers/sonar.h"
#include "drivers/util.h"
#include "drivers/stm32f4xx_exts.h"
#include "kernel/kernel.h"
#include "kernel/sync.h"

// state
static uint16_t sample;
static Signal signal;

// pin constants
static constexpr int PIN_SONAR = 9;
static constexpr int AF_TIM8 = 3;

void sonar_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    __DMB();

    TIM8->PSC = 168e6 / 1e6;
    TIM8->ARR = 0xFFFF;
    TIM8->CCMR2 = TIM_CCMR2_CC4S_TI4 | TIM_CCMR2_CC3S_TI4 | TIM_CCMR2_IC4F_0 | TIM_CCMR2_IC4F_1 | TIM_CCMR2_IC3F_0 | TIM_CCMR2_IC3F_1;
    TIM8->CCER = TIM_CCER_CC3P | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM8->DIER = TIM_DIER_CC3IE;
    TIM8->CR1 = TIM_CR1_CEN;

    GPIOC->AFR[1] |= AFRH(PIN_SONAR, AF_TIM8);
    GPIOC->MODER |= MODER_AF(PIN_SONAR);

    util_enable_irq(TIM8_CC_IRQn, IRQ_PRI_KERNEL);
}

uint16_t sonar_sample(bool wait) {
    if (wait)
        signal.wait();
    return sample;
}

extern "C" void irq_tim8_cc() {
    uint16_t rise = TIM8->CCR4;
    uint16_t fall = TIM8->CCR3;
    if (fall > rise)
        sample = fall - rise;
    else
        sample = (0xFFFF - rise) + fall;
    signal.notify_all();
}

