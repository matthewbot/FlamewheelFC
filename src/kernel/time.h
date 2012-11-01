#ifndef FC_KERNEL_TIME_H
#define FC_KERNEL_TIME_H

#include <stdint.h>

struct HighResTime {
    uint32_t ticks;
    uint32_t systick;
};

HighResTime time_get();
uint32_t time_diff(const HighResTime &a, const HighResTime &b); // difference between highrestime in clock cycles

#endif
