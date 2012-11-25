#include "drivers/i2c.h"
#include "drivers/util.h"
#include "drivers/stm32f4xx_exts.h"
#include "kernel/sync.h"

// state
enum class State { DONE, RECEIVING, SENDING };
static State state;
static uint8_t addr;
static const uint8_t *out_buf;
static size_t out_len;
static uint8_t *in_buf;
static size_t in_len;
static Callback callback;

// DMA constants
static constexpr int rx_stream_num = 2;
static constexpr DMA_Stream_TypeDef *rx_stream = DMA1_Stream2;
static constexpr int tx_stream_num = 7;
static constexpr DMA_Stream_TypeDef *tx_stream = DMA1_Stream7;

// Pin constants
static constexpr int PIN_SDA = 10;
static constexpr int PIN_SCL = 11;
static constexpr int AF_I2C2 = 4;

// I2C constants
constexpr I2C_TypeDef *i2c = I2C2;

void i2c_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    __DMB();

    // configure I2C
    i2c->CR2 = 42;
    i2c->TRISE = 500;
    i2c->CCR = 210;
    i2c->CR1 = I2C_CR1_PE;
    i2c->CR2 = I2C_CR2_ITERREN;
    util_enable_irq(I2C2_EV_IRQn, IRQ_PRI_HIGH);
    util_enable_irq(I2C2_ER_IRQn, IRQ_PRI_KERNEL);

    // configure receive stream
    rx_stream->PAR = (uint32_t)&i2c->DR;
    rx_stream->CR = (7 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
    util_enable_irq(DMA1_Stream2_IRQn, IRQ_PRI_KERNEL);

    // configure transmit stream
    tx_stream->PAR = (uint32_t)&i2c->DR;
    tx_stream->CR = (7 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_MEM2PER;

    // configure GPIO
    GPIOB->OTYPER |= (1 << PIN_SDA) | (1 << PIN_SCL);
    GPIOB->AFR[1] |= AFRH(PIN_SDA, AF_I2C2) | AFRH(PIN_SCL, AF_I2C2);
    GPIOB->MODER |= MODER_AF(PIN_SDA) | MODER_AF(PIN_SCL);
}

void i2c_async_send(uint8_t new_addr, const uint8_t *new_out_buf, size_t new_out_len, Callback new_callback) {
    addr = new_addr;
    state = State::SENDING;
    callback = new_callback;
    tx_stream->M0AR = (uint32_t)new_out_buf;
    tx_stream->NDTR = new_out_len;
    i2c->CR2 |= I2C_CR2_DMAEN | I2C_CR2_ITEVTEN;
    i2c->CR1 |= I2C_CR1_START;
}

void i2c_async_receive(uint8_t new_addr, uint8_t *new_in_buf, size_t new_in_len, Callback new_callback) {
    addr = new_addr;
    state = State::RECEIVING;
    callback = new_callback;
    rx_stream->M0AR = (uint32_t)new_in_buf;
    rx_stream->NDTR = new_in_len;
    i2c->CR2 |= I2C_CR2_DMAEN | I2C_CR2_ITEVTEN;
    i2c->CR1 |= I2C_CR1_START;
}

void i2c_async_done() {
    state = State::DONE;
    i2c->CR1 |= I2C_CR1_STOP;
    i2c->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_ITEVTEN);
}

// called after we've received a sample
extern "C" void irq_dma1_stream2() {
    DMA1->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2;
    i2c->CR1 &= ~I2C_CR1_ACK;
    i2c->CR2 &= ~I2C_CR2_LAST;
    callback();
}

// called when start is sent or when address acked
extern "C" void irq_i2c2_ev() {
    uint32_t sr = i2c->SR1;
    if (sr & I2C_SR1_SB) {
        i2c->DR = (state == State::RECEIVING) ? ((addr << 1) | 1) : (addr << 1);
    } else if (sr & I2C_SR1_ADDR) {
        if (state == State::RECEIVING) {
            i2c->CR1 |= I2C_CR1_ACK;
            i2c->CR2 |= I2C_CR2_LAST;
            rx_stream->CR |= DMA_SxCR_EN;
        } else {
            tx_stream->CR |= DMA_SxCR_EN;
        }
        (void)i2c->SR2; // clear ADDR
    } else if (sr & I2C_SR1_BTF) {
        if (state == State::SENDING) {
            DMA1->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7;
            callback();
        }
    }
}

extern "C" void irq_i2c2_er() {
    if (state == State::DONE)
        kernel_halt("I2C Error during synchronous mode");

    uint32_t sr = i2c->SR1;
    if (sr & (I2C_SR1_BERR | I2C_SR1_AF)) {
        i2c->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_AF);
        rx_stream->CR &= ~DMA_SxCR_EN;
        tx_stream->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2;
        i2c->CR1 &= ~I2C_CR1_ACK;
        i2c->CR2 &= ~I2C_CR2_LAST;
        if (i2c->SR2 & I2C_SR2_MSL) {
            i2c->CR1 |= I2C_CR1_STOP;
            while (i2c->CR1 & I2C_CR1_STOP) { }
        }
        i2c->CR1 |= I2C_CR1_START;
    }
}

void i2c_polling_start(uint8_t addr, bool read) {
    i2c->CR1 |= I2C_CR1_START;
    while (!(i2c->SR1 & I2C_SR1_SB)) { }
    i2c->DR = (addr << 1) | read;
    while (!(i2c->SR1 & I2C_SR1_ADDR)) { }
    (void)i2c->SR2;
}

uint8_t i2c_polling_read(bool ack) {
    if (ack)
        i2c->CR1 |= I2C_CR1_ACK;
    else
        i2c->CR1 &= ~I2C_CR1_ACK;
    while (!(i2c->SR1 & I2C_SR1_RXNE)) { }
    return i2c->DR;
}

void i2c_polling_write(uint8_t val) {
    i2c->DR = val;
    while (!(i2c->SR1 & I2C_SR1_TXE)) { }
}

void i2c_polling_stop() {
    i2c->CR1 |= I2C_CR1_STOP;
}
