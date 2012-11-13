#include "drivers/esc.h"
#include "drivers/util.h"
#include "drivers/stm32f4xx_exts.h"
#include "kernel/kernel.h"

// pin constants
static constexpr int PIN_0 = 6;
static constexpr int PIN_1 = 7;
static constexpr int PIN_2 = 8;
static constexpr int PIN_3 = 9;
static constexpr int AF_TIM4 = 2;

// timer constants
static constexpr TIM_TypeDef *tim = TIM4;

void esc_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    __DMB();

    tim->PSC = 84e6 / (2500 * 400);
    tim->ARR = 2500;
    tim->CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    tim->CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
    tim->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    tim->CR1 = TIM_CR1_CEN;

    GPIOB->AFR[0] |= AFRL(PIN_0, AF_TIM4) | AFRL(PIN_1, AF_TIM4);
    GPIOB->AFR[1] |= AFRH(PIN_2, AF_TIM4) | AFRH(PIN_3, AF_TIM4);
    GPIOB->MODER |= MODER_AF(PIN_0) | MODER_AF(PIN_1) | MODER_AF(PIN_2) | MODER_AF(PIN_3);

    esc_all_off();
}

void esc_all_off() {
    for (int i=0; i<4; i++)
        esc_off(i);
}

void esc_set(int esc, int pwm) {
    __IO uint32_t &reg = *(&tim->CCR1 + esc);
    reg = 1000+pwm;
}
