#ifndef FC_DRIVERS_I2C_H
#define FC_DRIVERS_I2C_H

#include <stdint.h>
#include <stdlib.h>

typedef void (*Callback)();

void i2c_init();
void i2c_async_send(uint8_t addr, const uint8_t *out_buf, size_t out_len, Callback callback);
void i2c_async_receive(uint8_t addr, uint8_t *in_buf, size_t in_len, Callback callback);
void i2c_async_done();

void i2c_polling_start(uint8_t addr, bool read);
uint8_t i2c_polling_read(bool ack);
void i2c_polling_write(uint8_t val);
void i2c_polling_stop();

#endif
