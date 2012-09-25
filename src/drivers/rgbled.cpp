#include "rgbled.h"
#include <stm32f4xx.h>

static uint32_t rgb;
static uint16_t period;
static uint16_t fadectr;

#define TIM_CCMR1_OC1M_PWM1 (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
#define TIM_CCMR1_OC2M_PWM1 (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)
#define TIM_CCMR2_OC3M_PWM1 (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2)

void rgbled_init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	__DMB();

	TIM1->PSC = 84e6 / 255e3;
	TIM1->ARR = 255;
	TIM1->CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1;
	TIM1->CCMR2 = TIM_CCMR2_OC3M_PWM1;
	TIM1->CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
	TIM1->DIER = TIM_DIER_UIE;
	TIM1->CR1 = TIM_CR1_CEN;

	NVIC->ISER[0] = TIM1_UP_TIM10_IRQn;

	GPIOB->AFR[1] |= 0x11100000;
	GPIOB->MODER |= 0xA8000000;
}

void rgbled_set(uint32_t newrgb, uint16_t newperiod) {
	rgb = newrgb;
	period = newperiod;
}

extern "C" void irq_tim1_up_tim10() {
	uint32_t sr = TIM1->SR;
	TIM1->SR = 0;
	if (!(sr & TIM_SR_UIF))
		return;

	if (++fadectr >= period)
		fadectr = 0;

	uint8_t r = (rgb & 0xFF0000) >> 16;
	uint8_t g = (rgb & 0x00FF00) >> 8;
	uint8_t b = (rgb & 0x0000FF);

	if (period > 0) {
		uint16_t halfperiod = period/2;
		uint16_t fade;
		if (fadectr < halfperiod)
			fade = fadectr;
		else
			fade = period - fadectr;

		r = (uint8_t)((r*fade)/halfperiod);
		g = (uint8_t)((g*fade)/halfperiod);
		b = (uint8_t)((b*fade)/halfperiod);
	}

	TIM1->CCR1 = r;
	TIM1->CCR2 = g;
	TIM1->CCR3 = b;
}
