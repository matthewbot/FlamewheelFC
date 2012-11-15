#include "drivers/board.h"
#include "drivers/stm32f4xx_exts.h"
#include "drivers/util.h"

// pin constants
static constexpr int PIN_SWITCH = 15;
static constexpr int PIN_BUZZER = 15;
static constexpr int PIN_VBAT = 0;
static constexpr int PIN_LED0 = 1;
static constexpr int PIN_LED1 = 4;
static constexpr int PIN_LED2 = 5;

// tim constants
static constexpr TIM_TypeDef *tim = TIM2;
static constexpr int AF_TIM2 = 1;

void board_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    __DMB();

    ADC1->SMPR1 = 0x7;
    ADC1->SQR3 = 8;
    ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    tim->PSC = 84e6 / (1000000);
    tim->CCMR1 = TIM_CCMR1_OC1M_TOGGLE;
    tim->CCR1 = 1;
    tim->CCER = TIM_CCER_CC1E;

    GPIOA->AFR[1] &= ~AFRH_MASK(PIN_BUZZER);
    GPIOA->MODER &= ~MODER_MASK(PIN_BUZZER);
    GPIOA->PUPDR &= ~PUPDR_MASK(PIN_BUZZER);
    GPIOA->AFR[1] |= AFRH(PIN_BUZZER, AF_TIM2);
    GPIOA->MODER |= MODER_AF(PIN_BUZZER);

    GPIOB->BSRRL = (1 << PIN_LED0) | (1 << PIN_LED1) | (1 << PIN_LED2);
    GPIOB->MODER |= MODER_AN(PIN_VBAT) | MODER_OUT(PIN_LED0) | MODER_OUT(PIN_LED1) | MODER_OUT(PIN_LED2);
}

bool board_switch() {
    return (GPIOC->IDR & (1 << PIN_SWITCH)) != 0;
}

void board_set_led(int led, bool on) {
    int mask;
    if (led == 0)
        mask = 1 << PIN_LED0;
    else if (led == 1)
        mask = 1 << PIN_LED1;
    else
        mask = 1 << PIN_LED2;

    if (on)
        GPIOB->BSRRH = mask;
    else
        GPIOB->BSRRL = mask;
}

float board_get_voltage() {
    uint16_t val = ADC1->DR;
    return val/100.0f - 1.0f;
}

void board_buzzer(int freq) {
    if (freq > 0) {
        tim->ARR = 1000000 / freq;
        tim->CNT = 0;
        tim->CR1 |= TIM_CR1_CEN;
    } else {
        tim->CR1 &= ~TIM_CR1_CEN;
        tim->CNT = 0;
    }
}
