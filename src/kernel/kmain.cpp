#include "kernel/kmain.h"
#include "kernel/kernel.h"
#include "kernel/task.h"
#include "kernel/sched.h"
#include <stm32f4xx.h>

static Task idle_task;
DECLARE_TASK_STACK(idle_stack, 64);
DECLARE_TASK_FUNC(idle_func);
static Task main_task;
DECLARE_TASK_STACK(main_stack, 4*1024);
DECLARE_TASK_FUNC(main_func);

void kmain() {
    SCB->SHP[2] = SCB->SHP[7] = SCB->SHP[10] = SCB->SHP[11] = 0xF0;
    SCB->SHCSR = SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk;
    SCB->CCR = SCB_CCR_DIV_0_TRP_Msk;
    FPU->FPCCR = 0;

    SysTick->LOAD = 168000 - 1;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;

    idle_task.setup("idle", Task::MIN, idle_func, nullptr, idle_stack, sizeof(idle_stack));
    sched_add_task_initial(idle_task);
    main_task.setup("main", Task::LOW, main_func, nullptr, main_stack, sizeof(main_stack));
    sched_add_task_initial(main_task);
    sched_start();
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
