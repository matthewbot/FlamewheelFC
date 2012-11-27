#include "drivers/i2c_shared.h"
#include "drivers/i2c.h"
#include "drivers/stm32f4xx_exts.h"
#include "drivers/util.h"
#include "kernel/sched.h"

static bool lock;

void i2c_shared_lock() {
    __disable_irq();
    util_disable_irq(EXTI15_10_IRQn); // disable all IRQs that might start to use the I2C... ugly to centralize them here :/
    util_disable_irq(EXTI9_5_IRQn);
    lock = true;
    __enable_irq();
}

void i2c_shared_unlock() {
    util_enable_irq(EXTI15_10_IRQn);
    util_enable_irq(EXTI9_5_IRQn);
    lock = false;
}

void i2c_shared_done_unlock() {
    i2c_async_done();
    i2c_shared_unlock();
}

void i2c_shared_wait() {
    while (lock)
        sched_sleep(5);
}
