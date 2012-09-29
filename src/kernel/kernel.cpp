#include "kernel.h"
#include "task.h"
#include "sched.h"
#include <stm32f4xx.h>

static int criticalcount;

void kernel_enter_critical() {
	__set_BASEPRI(KERNEL_IRQ_PRIORITY);
	criticalcount++;
}

void kernel_leave_critical() {
	if (--criticalcount == 0)
		__set_BASEPRI(0);
}

void kernel_sleep(uint32_t ticks) {
	Task *curtask = sched_current_task();

	KernelCriticalSection crit;
	sched_remove_task(*curtask);
	sched_add_task_tick(*curtask, ticks);
}
