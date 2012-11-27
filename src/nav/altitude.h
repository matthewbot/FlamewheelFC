#ifndef FC_NAV_ALTITUDE_H
#define FC_NAV_ALTITUDE_H

void altitude_init();

void altitude_start();
void altitude_stop();

float altitude_get();
float altitude_get_rate();

#endif
