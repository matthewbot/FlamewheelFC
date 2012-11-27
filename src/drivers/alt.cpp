#include "drivers/alt.h"
#include "drivers/i2c.h"
#include "drivers/i2c_shared.h"
#include "drivers/stm32f4xx_exts.h"
#include "drivers/util.h"
#include "kernel/sync.h"
#include "kernel/sched.h"

// state
static AltEEPROM eeprom;
static AltSample sample;
static Signal signal;
enum class State { TEMP, PRESSURE };
static State state;
static uint8_t read_buf[2];
static int temp;

// Pin constants
static constexpr int PIN_EOC = 5;

// Alt registers
static constexpr uint8_t addr = 0x77;
static constexpr int REG_AC1_MSB = 0xAA;
static constexpr int REG_CTRL = 0xF4;
static constexpr int REG_DATA_MSB = 0xF6;

// Commands
static const uint8_t cmd_conv_temp[] = { REG_CTRL, 0x2E };
static const uint8_t cmd_conv_pressure[] = { REG_CTRL, 0x34 };
static const uint8_t cmd_read[] = { REG_DATA_MSB };

static void callback_read_sent();
static void callback_received();
extern "C" void irq_exti95();

void alt_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __DMB();

    // read EEPROM
    sched_sleep(10);
    i2c_shared_wait();
    i2c_shared_lock();
    i2c_polling_start(addr, false);
    i2c_polling_write(REG_AC1_MSB);
    i2c_polling_start(addr, true);
    for (int i=0; i<11; i++) {
        uint8_t msb = i2c_polling_read(true);
        uint8_t lsb = i2c_polling_read(i != 10);
        eeprom.vals[i] = (msb << 8) | lsb;
    }
    i2c_polling_stop();
    i2c_shared_unlock();

    // enable EXTI
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PC;
    EXTI->RTSR |= (1 << PIN_EOC);
    EXTI->IMR |= (1 << PIN_EOC);
    util_enable_irq(EXTI9_5_IRQn, IRQ_PRI_HIGH);

    // jumpstart the async process
    if (!(GPIOC->IDR & (1 << PIN_EOC))) {
        i2c_shared_wait();
        i2c_shared_lock();
        i2c_async_send(addr, cmd_conv_temp, sizeof(cmd_conv_temp), i2c_shared_done_unlock);
    } else {
        irq_exti95();
    }
}

AltSample alt_sample(bool wait) {
    if (wait)
        signal.wait();
    return sample;
}

const AltEEPROM &alt_get_eeprom() { return eeprom; }

// called when EOSC pin goes high
extern "C" void irq_exti95() {
    i2c_shared_lock();
    i2c_async_send(addr, cmd_read, sizeof(cmd_read), callback_read_sent);
    EXTI->PR = (1 << PIN_EOC);
}

// called when read command is sent
static void callback_read_sent() {
    i2c_async_receive(addr, read_buf, sizeof(read_buf), callback_received);
}

// called when data is received
static void callback_received() {
    int val = (read_buf[0] << 8) | read_buf[1];

    if (state == State::TEMP) {
        temp = val;
        i2c_async_send(addr, cmd_conv_pressure, sizeof(cmd_conv_pressure), i2c_shared_done_unlock);
        state = State::PRESSURE;
    } else {
        sample.ut = temp;
        sample.up = val;
        signal.notify_all();
        i2c_async_send(addr, cmd_conv_temp, sizeof(cmd_conv_temp), i2c_shared_done_unlock);
        state = State::TEMP;
    }
}
