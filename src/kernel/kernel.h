#ifndef KERNEL_KERNEL_H
#define KERNEL_KERNEL_H

#include <stdint.h>
#include "kernel/sched.h"

constexpr uint8_t KERNEL_IRQ_PRIORITY = 0xF0;

void kernel_enter_critical();
void kernel_leave_critical();

struct KernelCriticalSection {
	KernelCriticalSection() { kernel_enter_critical(); }
	~KernelCriticalSection() { kernel_leave_critical(); }
};

void kernel_halt(const char *msg);

#endif
