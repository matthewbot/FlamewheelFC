#include "drivers/board.h"
#include "drivers/stm32f4xx_exts.h"
#include "drivers/util.h"

// pin constants
static constexpr int PIN_SWITCH = 15;
static constexpr int PIN_VBAT = 0;
static constexpr int PIN_LED0 = 1;
static constexpr int PIN_LED1 = 4;
static constexpr int PIN_LED2 = 5;

void board_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    __DMB();

    ADC1->SMPR1 = 0x7;
    ADC1->SQR3 = 8;
    ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;

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
