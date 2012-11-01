#include "drivers/mag.h"
#include "drivers/util.h"
#include "stm32f4xx_exts.h"

static constexpr int PIN_SDA = 10;
static constexpr int PIN_SCL = 11;
static constexpr int PIN_DRDY = 12;
static constexpr int AF_I2C2 = 4;

static constexpr I2C_TypeDef *i2c = I2C2;
static constexpr int addr_read = 0x3D;
static constexpr int addr_write = 0x3C;

void mag_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    __DMB();

    i2c->CR2 = 42;
    i2c->TRISE = 500;
    i2c->CCR = 210;
    i2c->CR1 = I2C_CR1_PE;

    GPIOB->OTYPER |= (1 << PIN_SDA) | (1 << PIN_SCL);
    GPIOB->AFR[1] |= AFRH(PIN_SDA, AF_I2C2) | AFRH(PIN_SCL, AF_I2C2);
    GPIOB->MODER |= MODER_AF(PIN_SDA) | MODER_AF(PIN_SCL);
}

static void start(uint8_t addr) {
    while (true) {
        i2c->SR1 = 0;
        i2c->CR1 |= I2C_CR1_START;
        uint32_t sr;
        do {
            sr = i2c->SR1;
        } while (!(sr & (I2C_SR1_SB | I2C_SR1_BERR)));

        if (sr & I2C_SR1_BERR) {
            i2c->SR1 = 0;
            i2c->CR1 |= I2C_CR1_STOP;
            while (i2c->CR1 & I2C_CR1_STOP) { }
            i2c->CR1 = 0;
            __DMB();
            i2c->CR1 = I2C_CR1_PE;
            __DMB();
            continue;
        }

        i2c->DR = addr;
        __DMB();
        do {
            sr = i2c->SR1;
        } while (!(sr & (I2C_SR1_ADDR | I2C_SR1_AF)));

        if (sr & I2C_SR1_AF) {
            i2c->SR1 = 0;
            i2c->CR1 |= I2C_CR1_STOP;
            while (i2c->CR1 & I2C_CR1_STOP) { }
            continue;
        }

        (void)i2c->SR2;
        break;
    }
}

uint8_t mag_readreg(uint8_t addr) {
    start(addr_write);

    i2c->DR = addr;
    start(addr_read);
    i2c->CR1 |= I2C_CR1_STOP;
    while (!(i2c->SR1 & I2C_SR1_RXNE)) { }

    return i2c->DR;
}

void mag_writereg(uint8_t addr, uint8_t val) {
    start(addr_write);

    i2c->DR = addr;
    while (!(i2c->SR1 & I2C_SR1_TXE)) { }
    i2c->DR = val;
    while (!(i2c->SR1 & I2C_SR1_TXE)) { }
    i2c->CR1 |= I2C_CR1_STOP;
}
