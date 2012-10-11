#include "drivers/mpu.h"
#include "drivers/util.h"
#include "kernel/kernel.h"
#include "kernel/debug.h"
#include <stm32f4xx.h>

#define SPI_CR1_BR_DIV128 (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

// state
static AccelFS accel_fs;
static GyroFS gyro_fs;
static uint8_t dlpf;
static uint8_t samplerate_div;
static MPUSample sample;

// pin constants
static constexpr int PIN_NSS = 4;
static constexpr int PIN_SCK = 5;
static constexpr int PIN_MISO = 6;
static constexpr int PIN_MOSI = 7;
static constexpr int PIN_INT = 4;
static constexpr int AF_SPI1 = 5;

// MPU registers
static constexpr int REG_SMPLRT_DIV = 25;
static constexpr int REG_CONFIG = 26;
static constexpr int REG_GYRO_CONFIG = 27;
static constexpr int REG_ACCEL_CONFIG = 28;
static constexpr int REG_INT_PIN_CFG = 55;
static constexpr int REG_INT_ENABLE = 56;
static constexpr int REG_ACCEL_XOUT_H = 59;
static constexpr int REG_USER_CTRL = 106;
static constexpr int REG_PWR_MGMT_1 = 107;
static constexpr int REG_PWR_MGMT_2 = 108;
static constexpr int REG_WHO_AM_I = 117;

// utility functions
//static uint8_t readreg(uint8_t reg);
static void writereg(uint8_t reg, uint8_t val);
static uint16_t spi16(uint16_t val);
static uint8_t spi(uint8_t val);
static void ss(bool val);

void mpu_init() {
	// configure clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_SPI1EN;
	__DMB();

	// enable SPI
	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_DIV128 | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SPE;

	// configure GPIOs
	GPIOA->BSRRL = (1 << PIN_NSS);
	GPIOA->AFR[0] |= AFRL(PIN_SCK, AF_SPI1) | AFRL(PIN_MISO, AF_SPI1) | AFRL(PIN_MOSI, AF_SPI1);
	GPIOA->MODER |= MODER_OUT(PIN_NSS) | MODER_AF(PIN_SCK) | MODER_AF(PIN_MISO) | MODER_AF(PIN_MOSI);

	// enable EXTI
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC;
	EXTI->RTSR |= (1 << PIN_INT); // enable rising edge
	EXTI->IMR |= (1 << PIN_INT); // enable interrupt from EXTI

	// set up MPU
	mpu_reset(AccelFS::FS4G, GyroFS::FS500DS, 1, 0);

	util_enable_irq(EXTI0_IRQn + PIN_INT, 0x20); // enable interrupt in NVIC
}

void mpu_reset(AccelFS new_accel_fs, GyroFS new_gyro_fs, uint8_t new_dlpf, uint8_t new_samplerate_div) {
	accel_fs = new_accel_fs;
	gyro_fs = new_gyro_fs;
	dlpf = new_dlpf;
	samplerate_div = new_samplerate_div;

	writereg(REG_PWR_MGMT_1, 1 << 7); // reset MPU
	kernel_sleep(100);
	writereg(REG_PWR_MGMT_1, 3); // clock source is z gyro
	writereg(REG_PWR_MGMT_2, 0);
	writereg(REG_USER_CTRL, (1 << 4)); // disable I2C interface

	if (readreg(REG_WHO_AM_I) != 0x68) // check WHO_AM_I register
		kernel_halt("MPU failed to read WHO_AM_I");

	writereg(REG_SMPLRT_DIV, samplerate_div); // set sample rate divisor
	writereg(REG_CONFIG, dlpf); // set DLPF
	writereg(REG_GYRO_CONFIG, (int)gyro_fs << 3); // set gyro FS_SEL
	writereg(REG_ACCEL_CONFIG, (int)accel_fs << 3); // set accel FS_SEL
	writereg(REG_INT_PIN_CFG, (1 << 5) | (1 << 4)); // turn on LATCH_INT and INT_RD_CLEAR
	writereg(REG_INT_ENABLE, (1 << 0)); // turn on DATA_RDY_EN
}

MPUSample mpu_sample(uint32_t samplenum) {
	return sample;
}

extern "C" void irq_ext4() {
	sample.samplenum++;
	ss(true);
	spi(REG_ACCEL_XOUT_H | (1 << 7));
	for (int i=0; i<3; i++)
		sample.accel[i] = spi16(0);
	sample.temp = spi16(0);
	for (int i=0; i<3; i++)
		sample.gyro[i] = spi16(0);
	ss(false);
   	EXTI->PR = (1 << 4);
}

uint8_t readreg(uint8_t reg) {
	ss(true);
	spi(reg | (1 << 7));
	uint8_t val = spi(0);
	ss(false);
	util_delay(100);
	return val;
}

static void writereg(uint8_t reg, uint8_t val) {
	ss(true);
	spi(reg);
	spi(val);
	ss(false);
	util_delay(100);
}

static void ss(bool ss) {
	if (ss)
		GPIOA->BSRRH = (1 << 4);
	else
		GPIOA->BSRRL = (1 << 4);
	__DMB();
}

static uint8_t spi(uint8_t val) {
	while (!(SPI1->SR & SPI_SR_TXE)) { }
	SPI1->DR = val;
	while (!(SPI1->SR & SPI_SR_RXNE)) { }
	return SPI1->DR;
}

static uint16_t spi16(uint16_t val) {
	uint16_t data = spi(val >> 8) << 8;
	data |= spi(val & 0xFF);
	return data;
}
