#include "drivers/mag.h"
#include "drivers/util.h"
#include "kernel/sync.h"
#include "stm32f4xx_exts.h"

// state
static MagSample sample;
enum class State { RECEIVING, SENDING };
static State state;
static uint8_t read_buf[6];
static Signal signal;
static bool initialized;

// DMA constants
static constexpr int rx_stream_num = 2;
static constexpr DMA_Stream_TypeDef *rx_stream = DMA1_Stream2;
static constexpr int tx_stream_num = 7;
static constexpr DMA_Stream_TypeDef *tx_stream = DMA1_Stream7;

// Pin constants
static constexpr int PIN_SDA = 10;
static constexpr int PIN_SCL = 11;
static constexpr int PIN_DRDY = 12;
static constexpr int AF_I2C2 = 4;

// I2C constants
constexpr I2C_TypeDef *i2c = I2C2;
static constexpr int addr_read = 0x3D;
static constexpr int addr_write = 0x3C;

// Mag registers
static constexpr int REG_CONFIGA = 0;
static constexpr int REG_CONFIGB = 1;
static constexpr int REG_MODE = 2;
static constexpr int REG_DATAX = 3;
static constexpr int REG_IDA = 10;
static const uint8_t read_cmd[] = {REG_MODE, 0x01};

static void start(uint8_t addr);
static uint8_t read(bool ack=true);
static void write(uint8_t reg);
static void stop();

void mag_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __DMB();

    // configure I2C
    i2c->CR2 = 42;
    i2c->TRISE = 500;
    i2c->CCR = 210;
    i2c->CR1 = I2C_CR1_PE;
    i2c->CR2 = I2C_CR2_ITERREN;

    // configure receive stream
    rx_stream->PAR = (uint32_t)&i2c->DR;
    rx_stream->M0AR = (uint32_t)read_buf;
    rx_stream->NDTR = sizeof(read_buf);
    rx_stream->CR = (7 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
    util_enable_irq(DMA1_Stream2_IRQn, IRQ_PRI_KERNEL);

    // configure transmit stream
    tx_stream->PAR = (uint32_t)&i2c->DR;
    tx_stream->M0AR = (uint32_t)read_cmd;
    tx_stream->NDTR = sizeof(read_cmd);
    tx_stream->CR = (7 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_MEM2PER;

    // configure GPIO
    GPIOB->OTYPER |= (1 << PIN_SDA) | (1 << PIN_SCL);
    GPIOB->AFR[1] |= AFRH(PIN_SDA, AF_I2C2) | AFRH(PIN_SCL, AF_I2C2);
    GPIOB->MODER |= MODER_AF(PIN_SDA) | MODER_AF(PIN_SCL);

    // configure mag
    start(addr_write);
    write(REG_IDA); // read the IDA register
    start(addr_read);
    if (read(false) != 'H')
        kernel_halt("Failed to identify magnetometer");
    start(addr_write);
    write(REG_CONFIGA);
    write(0); // config a
    write(0x40); // config b
    stop();

    initialized = true;

    // enable EXTI
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PB;
    EXTI->RTSR |= (1 << PIN_DRDY);
    EXTI->IMR |= (1 << PIN_DRDY);

    // finish configuring I2C
    i2c->CR2 |= I2C_CR2_DMAEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    __DMB();
    util_enable_irq(I2C2_EV_IRQn, IRQ_PRI_HIGH);
    util_enable_irq(I2C2_ER_IRQn, IRQ_PRI_KERNEL);

    // enable interrupt in NVIC
    state = State::RECEIVING;
    i2c->CR1 |= I2C_CR1_START;
}

MagSample mag_sample(bool block) {
    KernelCriticalSection crit;
    if (block)
        signal.wait();
    return sample;
}

MagSample mag_sample_averaged(int samples) {
    int32_t field_accum[3] = { 0, 0, 0 };
    for (int s=0; s<samples; s++) {
        signal.wait();
        for (int i=0; i<3; i++)
            field_accum[i] += sample.field[i];
    }

    MagSample ret = sample;
    for (int i=0; i<3; i++)
        ret.field[i] = field_accum[i] / samples;
    return ret;
}

// called when DRDY pin goes high
extern "C" void irq_exti1510() {
    state = State::RECEIVING;
    i2c->CR1 |= I2C_CR1_START;
    EXTI->PR = (1 << PIN_DRDY);
    util_disable_irq(EXTI15_10_IRQn);
}

// called after we've received a sample
extern "C" void irq_dma1_stream2() {
    const uint8_t *pos = read_buf;
    sample.num++;
    sample.field[0] = pos[0] << 8 | pos[1]; pos += 2;
    sample.field[2] = pos[0] << 8 | pos[1]; pos += 2; // sensor goes X, Z, Y according to documentation
    sample.field[1] = pos[0] << 8 | pos[1];
    signal.notify_all();

    DMA1->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2;
    i2c->CR1 &= ~I2C_CR1_ACK;
    i2c->CR2 &= ~I2C_CR2_LAST;
    state = State::SENDING;
    i2c->CR1 |= I2C_CR1_START;
}

// called when start is sent or when address acked
extern "C" void irq_i2c2_ev() {
    uint32_t sr = i2c->SR1;
    if (sr & I2C_SR1_SB) {
        i2c->DR = (state == State::RECEIVING) ? addr_read : addr_write;
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
            state = State::RECEIVING;
            i2c->CR1 |= I2C_CR1_STOP;
            util_enable_irq(EXTI15_10_IRQn, IRQ_PRI_HIGH);
        }
    }
}

extern "C" void irq_i2c2_er() {
    if (!initialized)
        kernel_halt("I2C Error before initialization");

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
        state = State::RECEIVING;
    }
}

static void start(uint8_t addr) {
    i2c->CR1 |= I2C_CR1_START;
    while (!(i2c->SR1 & I2C_SR1_SB)) { }
    i2c->DR = addr;
    while (!(i2c->SR1 & I2C_SR1_ADDR)) { }
    (void)i2c->SR2;
}

static void stop() {
    i2c->CR1 |= I2C_CR1_STOP;
}

static void write(uint8_t val) {
    i2c->DR = val;
    while (!(i2c->SR1 & I2C_SR1_TXE)) { }
}

static uint8_t read(bool ack) {
    if (ack)
        i2c->CR1 |= I2C_CR1_ACK;
    else
        i2c->CR1 &= ~I2C_CR1_ACK;
    while (!(i2c->SR1 & I2C_SR1_RXNE)) { }
    return i2c->DR;
}
