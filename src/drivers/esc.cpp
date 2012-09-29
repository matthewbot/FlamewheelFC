#include "drivers/esc.h"
#include "drivers/util.h"
#include "kernel/kernel.h"
#include <stm32f4xx.h>

// TODO group these into a header
#define TIM_CCMR1_OC1M_PWM1 (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
#define TIM_CCMR1_OC2M_PWM1 (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)
#define TIM_CCMR2_OC3M_PWM1 (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2)
#define TIM_CCMR2_OC4M_PWM1 (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2)

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

	GPIOB->AFR[0] |= AFRL(7, 2) | AFRL(6, 2);
	GPIOB->AFR[1] |= AFRH(9, 2) | AFRH(8, 2);
	GPIOB->MODER |= MODER_AF(9) | MODER_AF(8) | MODER_AF(7) | MODER_AF(6);
}

void esc_arm() {
	for (int i=0; i<4; i++)
		esc_set(i, 0);
	kernel_sleep(5000);
}

void esc_set(int esc, int pwm) {
	__IO uint32_t &reg = *(&TIM4->CCR1 + esc);
	reg = 1000+pwm;
}
