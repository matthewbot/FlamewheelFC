#ifndef KERNEL_SCHED_H
#define KERNEL_SCHED_H

#include "task.h"

// task APIs
inline uint32_t sched_now() { extern uint32_t curtick; return curtick; }
inline Task *sched_current_task() { extern Task *curtask; return curtask; }
void sched_yield();
void sched_sleep(uint32_t ticks);

// scheduler APIs
void sched_add_task(Task &task);
void sched_remove_task(Task &task);
void sched_add_task_tick(Task &task, uint32_t tick);
void sched_cancel_task_tick(Task &task);

// kernel startup APIs
void sched_add_task_initial(Task &task);
void sched_start() __attribute__((noreturn));
void sched_tick();

#endif
