#ifndef FC_DRIVERS_ESC_H
#define FC_DRIVERS_ESC_H

void esc_init();

enum class ESC { FRONT_LEFT, FRONT_RIGHT, REAR_RIGHT, REAR_LEFT };

void esc_set(int esc, int pwm);

void esc_all_off();
inline void esc_off(int esc) { esc_set(esc, 0); }

#endif
