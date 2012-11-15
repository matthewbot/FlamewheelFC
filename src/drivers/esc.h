#ifndef FC_DRIVERS_ESC_H
#define FC_DRIVERS_ESC_H

#include <stdint.h>

void esc_init();

enum class ESC { FRONT_LEFT, FRONT_RIGHT, REAR_RIGHT, REAR_LEFT };

void esc_set(int esc, uint16_t pwm);
void esc_set_all(const uint16_t (&pwms)[4]);

void esc_all_off();
inline void esc_off(int esc) { esc_set(esc, 0); }

#endif
