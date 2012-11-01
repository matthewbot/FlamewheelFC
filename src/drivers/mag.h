#ifndef FC_DRIVERS_MAG_H
#define FC_DRIVERS_MAG_H

#include <stdint.h>

void mag_init();

struct MagSample {
    uint32_t num;
    int16_t x, y, z;
};

MagSample mag_sample();

#endif
