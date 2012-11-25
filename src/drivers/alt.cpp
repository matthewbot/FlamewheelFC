#include "drivers/alt.h"
#include "drivers/i2c.h"
#include "drivers/stm32f4xx_exts.h"
#include "kernel/sync.h"

// state
static AltEEPROM eeprom;
static AltSample sample;
static Signal signal;

// Pin constants
static constexpr int PIN_EOC = 5;

// Alt registers
static constexpr uint8_t addr = 0x77;
static constexpr int REG_AC1_MSB = 0xAA;
static constexpr int REG_CTRL = 0xF4;
static constexpr int REG_DATA_MSB = 0xF6;

void alt_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    __DMB();

    // read EEPROM
    i2c_polling_start(addr, false);
    i2c_polling_write(REG_AC1_MSB);
    i2c_polling_start(addr, true);
    for (int i=0; i<11; i++) {
        uint8_t msb = i2c_polling_read(true);
        uint8_t lsb = i2c_polling_read(i != 10);
        eeprom.vals[i] = (msb << 8) | lsb;
    }
    i2c_polling_stop();
}
