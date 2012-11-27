#ifndef FC_DRIVERS_SONAR_H
#define FC_DRIVERS_SONAR_H

#include <stdint.h>

void sonar_init();
uint16_t sonar_sample(bool wait=false);

#endif
