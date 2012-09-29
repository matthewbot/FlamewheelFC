#include "drivers/mpu.h"
#include "drivers/util.h"
#include "kernel/kernel.h"
#include <stm32f4xx.h>

#define SPI_CR1_BR_DIV128 (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

void mpu_init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	__DMB();

	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_DIV128 | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SPE;

	mpu_ss(false);
	GPIOA->AFR[0] |= AFRL(5, 5) | AFRL(6, 5) | AFRL(7, 5);
	GPIOA->MODER |= MODER_OUT(4) | MODER_AF(5) | MODER_AF(6) | MODER_AF(7);
}

uint8_t mpu_read_reg(uint8_t reg) {
	mpu_ss(true);
	mpu_spi(reg | 0x80);
	uint8_t val = mpu_spi(0);
	mpu_ss(false);
	kernel_sleep(1);
	return val;
}

void mpu_write_reg(uint8_t reg, uint8_t val) {
	mpu_ss(true);
	mpu_spi(reg);
	mpu_spi(val);
	mpu_ss(false);
	kernel_sleep(1);
}

void mpu_ss(bool ss) {
	if (ss)
		GPIOA->BSRRH = (1 << 4);
	else
		GPIOA->BSRRL = (1 << 4);
	__DMB();
}

uint8_t mpu_spi(uint8_t out) {
	while (!(SPI1->SR & SPI_SR_TXE)) { }
	SPI1->DR = out;
	while (!(SPI1->SR & SPI_SR_RXNE)) { }
	return SPI1->DR;
}
