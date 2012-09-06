#ifndef KERNEL_KERNEL_H
#define KERNEL_KERNEL_H

#include "sched.h"

static const uint8_t KERNEL_IRQ_PRIORITY = 0xA0;

void kernel_start() __attribute__((noreturn));

void kernel_enter_critical();
void kernel_leave_critical();

struct KernelCriticalSection {
	KernelCriticalSection() { kernel_enter_critical(); }
	~KernelCriticalSection() { kernel_leave_critical(); }
};


#endif
