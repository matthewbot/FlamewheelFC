#ifndef FC_DRIVERS_I2C_SHARED_H
#define FC_DRIVERS_I2C_SHARED_H

void i2c_shared_lock();
void i2c_shared_unlock();
void i2c_shared_done_unlock();

void i2c_shared_wait();

#endif
