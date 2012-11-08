#ifndef FC_DRIVERS_MAG_H
#define FC_DRIVERS_MAG_H

#include <stdint.h>

void mag_init();

struct MagSample {
    uint32_t num;
    int16_t field[3];
};

MagSample mag_sample(bool block=true);
MagSample mag_sample_averaged(int samples);

#endif
