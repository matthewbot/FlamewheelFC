#ifndef FC_DRIVERS_ALT_H
#define FC_DRIVERS_ALT_H

#include <stdint.h>

void alt_init();

union AltEEPROM {
    struct {
        int16_t AC1;
        int16_t AC2;
        int16_t AC3;
        uint16_t AC4;
        uint16_t AC5;
        uint16_t AC6;
        int16_t B1;
        int16_t B2;
        int16_t MB;
        int16_t MC;
        int16_t MD;
    };
    int16_t vals[11];
} __attribute__((packed));

const AltEEPROM &alt_get_eeprom();

struct AltSample {
    int ut;
    int up;
};

AltSample alt_sample(bool wait=true);

#endif
