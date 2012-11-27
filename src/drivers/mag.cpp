#include "drivers/mag.h"
#include "drivers/util.h"
#include "drivers/i2c.h"
#include "drivers/i2c_shared.h"
#include "kernel/sync.h"
#include "stm32f4xx_exts.h"

// state
static MagSample sample;
static uint8_t read_buf[6];
static Signal signal;

// Pin constants
static constexpr int PIN_DRDY = 12;

// Mag registers
static constexpr uint8_t addr = 0x1E;
static constexpr int REG_CONFIGA = 0;
static constexpr int REG_CONFIGB = 1;
static constexpr int REG_MODE = 2;
static constexpr int REG_DATAX = 3;
static constexpr int REG_IDA = 10;
static const uint8_t read_cmd[] = {REG_MODE, 0x01};

// i2c callbacks
static void callback_received();
extern "C" void irq_exti1510();

void mag_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __DMB();

    // configure mag
    i2c_shared_wait();
    i2c_shared_lock();
    i2c_polling_start(addr, false);
    i2c_polling_write(REG_IDA); // read the IDA register
    i2c_polling_start(addr, true);
    if (i2c_polling_read(false) != 'H')
        kernel_halt("Failed to identify magnetometer");
    i2c_polling_start(addr, false);
    i2c_polling_write(REG_CONFIGA);
    i2c_polling_write(0); // config a
    i2c_polling_write(0x40); // config b
    i2c_polling_stop();
    i2c_shared_unlock();

    // enable EXTI
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PB;
    EXTI->RTSR |= (1 << PIN_DRDY);
    EXTI->IMR |= (1 << PIN_DRDY);
    util_enable_irq(EXTI15_10_IRQn, IRQ_PRI_HIGH);

    // jumpstart the async process
    if (!(GPIOB->IDR & (1 << PIN_DRDY))) {
        i2c_shared_wait();
        i2c_shared_lock();
        i2c_async_send(addr, read_cmd, sizeof(read_cmd), i2c_shared_done_unlock); // if DRDY is low, send a command
    } else {
        irq_exti1510(); // if DRDY is high, run its IRQ since EXTI looks for edges not levels
    }
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
    i2c_shared_lock();
    i2c_async_receive(addr, read_buf, sizeof(read_buf), callback_received);
    EXTI->PR = (1 << PIN_DRDY);
}

// called after we've received a sample
static void callback_received() {
    const uint8_t *pos = read_buf;
    sample.num++;
    sample.field[0] = pos[0] << 8 | pos[1]; pos += 2;
    sample.field[2] = pos[0] << 8 | pos[1]; pos += 2; // sensor goes X, Z, Y according to documentation
    sample.field[1] = pos[0] << 8 | pos[1];
    signal.notify_all();

    i2c_async_send(addr, read_cmd, sizeof(read_cmd), i2c_shared_done_unlock);
}
