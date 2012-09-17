#ifndef KERNEL_DEBUG_H
#define KERNEL_DEBUG_H

#include <stdint.h>

void debug_init();
void debug_setLED(int led, bool on=true);
void debug_putch(char ch);
void debug_puts(const char *str);
void debug_puthex(uint32_t num);
void debug_delay(long cycles);

#endif
