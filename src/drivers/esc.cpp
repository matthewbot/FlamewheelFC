#include "drivers/esc.h"
#include "drivers/util.h"
#include "drivers/stm32f4xx_exts.h"
#include "kernel/kernel.h"

static constexpr int PIN_0 = 6;
static constexpr int PIN_1 = 7;
static constexpr int PIN_2 = 8;
static constexpr int PIN_3 = 9;
static constexpr int AF_TIM4 = 2;

void esc_init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	__DMB();

	TIM4->PSC = 84e6 / (2500 * 400);
	TIM4->ARR = 2500;
	TIM4->CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
	TIM4->CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC4PE;
	TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM4->CR1 = TIM_CR1_CEN;

	GPIOB->AFR[0] |= AFRL(PIN_0, AF_TIM4) | AFRL(PIN_1, AF_TIM4);
	GPIOB->AFR[1] |= AFRH(PIN_2, AF_TIM4) | AFRH(PIN_2, AF_TIM4);
	GPIOB->MODER |= MODER_AF(PIN_0) | MODER_AF(PIN_1) | MODER_AF(PIN_2) | MODER_AF(PIN_3);
}

void esc_arm() {
	for (int i=0; i<4; i++)
		esc_set(i, 0);
	sched_sleep(5000);
}

void esc_set(int esc, int pwm) {
	__IO uint32_t &reg = *(&TIM4->CCR1 + esc);
	reg = 1000+pwm;
}
