#include "kernel/time.h"
#include "kernel/sched.h"
#include <stm32f4xx.h>

HighResTime time_get() { return { sched_now(), SysTick->VAL }; }
uint32_t time_diff(const HighResTime &a, const HighResTime &b) {
    uint32_t ticks = (a.ticks - b.ticks)*168000;
    ticks += (int32_t)(b.systick - a.systick);
    return ticks;
}
