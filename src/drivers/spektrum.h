#ifndef FC_DRIVERS_SPEKTRUM_H
#define FC_DRIVERS_SPEKTRUM_H

#include <stdint.h>

void spektrum_init();
void spektrum_bind();

struct SpektrumSample {
    uint8_t headernum;
    int16_t channel[8];
};

SpektrumSample spektrum_sample(bool block=true);
bool spektrum_valid();

#endif
