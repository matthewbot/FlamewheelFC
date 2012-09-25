#ifndef FC_DRIVERS_RGBLED
#define FC_DRIVERS_RGBLED

#include <stdint.h>

void rgbled_init();
void rgbled_set(uint32_t newrgb, uint16_t newperiod);

#endif
