#ifndef KERNEL_SCHED_H
#define KERNEL_SCHED_H

#include "task.h"

typedef uint32_t Tick;

Tick sched_now();
Task *sched_current_task();
void sched_sleep(Tick ticks);
void sched_yield();

// must be invoked in kernel critical section
void sched_add_task(Task &task);
void sched_remove_task(Task &task);
void sched_add_task_tick(Task &task, Tick tick);
void sched_cancel_task_tick(Task &task);

// kernel APIs only
void sched_add_task_initial(Task &task);
void sched_start() __attribute__((noreturn));
void sched_tick();

#endif
