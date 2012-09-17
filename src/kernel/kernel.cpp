#include "kernel.h"
#include "task.h"
#include "sched.h"
#include <stm32f4xx.h>

static Task idle_task;
DECLARE_TASK_STACK(idle_stack, 1024);
static void idle_func(void *unused) __attribute__((noreturn));
static Task main_task;
DECLARE_TASK_STACK(main_stack, 4*1024);
static void main_func(void *unused) __attribute__((noreturn));

void kernel_start() {
	SCB->SHP[7] = SCB->SHP[10] = SCB->SHP[11] = KERNEL_IRQ_PRIORITY;
	SCB->SHCSR = SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk;

	SysTick->LOAD = 168000 - 1;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;

	idle_task.setup("idle", Task::MIN, idle_func, nullptr, idle_stack, sizeof(idle_stack));
	sched_add_task_initial(idle_task);
	main_task.setup("main", Task::LOW, main_func, nullptr, main_stack, sizeof(main_stack));
	sched_add_task_initial(main_task);
	sched_start();
}

static int criticalcount;

void kernel_enter_critical() {
	__set_BASEPRI(KERNEL_IRQ_PRIORITY);
	criticalcount++;
}

void kernel_leave_critical() {
	if (--criticalcount == 0)
		__set_BASEPRI(0);
}

extern "C" void handler_systick() {
	sched_tick();
}

static void idle_func(void *unused) {
	while (true)
		__WFI();
}

extern int main();

static void main_func(void *unused) {
	main();
	sched_remove_task(main_task);
	sched_yield();
	__builtin_unreachable();
}
