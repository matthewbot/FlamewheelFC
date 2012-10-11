#ifndef KERNEL_KERNEL_H
#define KERNEL_KERNEL_H

#include <stdint.h>
#include "kernel/sched.h"

static const uint8_t KERNEL_IRQ_PRIORITY = 0xA0;

void kernel_enter_critical();
void kernel_leave_critical();

struct KernelCriticalSection {
	KernelCriticalSection() { kernel_enter_critical(); }
	~KernelCriticalSection() { kernel_leave_critical(); }
};

inline uint32_t kernel_now() { return sched_now(); }
inline Task *kernel_current_task() { return sched_current_task(); }
void kernel_sleep(uint32_t ticks);
inline void kernel_yield() { sched_yield(); }
void kernel_halt(const char *msg);

#endif
