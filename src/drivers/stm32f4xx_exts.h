#ifndef FC_DRIVERS_EXTRASTM32_H
#define FC_DRIVERS_EXTRASTM32_H

#include <stm32f4xx.h>

#define SPI_CR1_BR_DIV128 (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

#define DMA_SxCR_CHSEL_Pos 25
#define DMA_SxCR_DIR_MEM2PER DMA_SxCR_DIR_0

#define TIM_CCMR1_OC1M_PWM1 (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
#define TIM_CCMR1_OC2M_PWM1 (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)
#define TIM_CCMR2_OC3M_PWM1 (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2)
#define TIM_CCMR2_OC4M_PWM1 (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2)

#define TIM_CCMR1_OC1M_TOGGLE (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 )
#define TIM_CCMR1_OC1M_FORCE_OFF (TIM_CCMR1_OC1M_2)

#define TIM_CCMR2_CC4S_TI4 (TIM_CCMR2_CC4S_0)
#define TIM_CCMR2_CC3S_TI4 (TIM_CCMR2_CC3S_1)

#endif
