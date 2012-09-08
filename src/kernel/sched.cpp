#include "sched.h"
#include "task.h"
#include "kernel.h"
#include <stddef.h>
#include <stm32f4xx.h>

static Task *curtask;
static Tick curtick;

Tick sched_now() {
	return curtick;
}

void sched_sleep(Tick ticks) {
	KernelCriticalSection crit;
	sched_remove_task(*curtask);
	sched_add_task_tick(*curtask, curtick + ticks);
}

void sched_yield() {
	SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

Task *sched_current_task() { return curtask; }

static Task *nexttask;
static ListNode schedlist;
static ListNode ticklist;

static Task *schedlist_head() {
	return &schedlist.next->getParent(&Task::list);
}

static Task *ticklist_head() {
	if (ticklist.next == nullptr)
		return nullptr;
	return &ticklist.next->getParent(&Task::list);
}

void sched_add_task(Task &task) {
	if (task.priority < schedlist_head()->priority)
		sched_yield();

	task.insert_list_priority_sorted(schedlist);
	task.state = Task::State::SCHEDULED;
}

void sched_remove_task(Task &task) {
	if (&task == curtask || &task == nexttask)
		nexttask = task.list_next();

	task.remove_list();
	task.state = Task::State::NONE;

	if (&task == curtask)
		sched_yield();
}

void sched_add_task_tick(Task &task, Tick tick) {
	task.waketick = tick;
	task.state = Task::State::SLEEP;
	task.insert_list_waketick_sorted(ticklist);
}

void sched_cancel_task_tick(Task &task) {
	task.state = Task::State::NONE;
	task.remove_list();
}

void sched_add_task_initial(Task &task) {
	task.list.insert_after(schedlist);
}

__attribute__((noreturn))
void sched_start() {
	curtask = schedlist_head();
	asm volatile("svc 0");
	__builtin_unreachable();
}

extern "C" void handler_svcall() __attribute__((naked));
extern "C" void handler_svcall() {
	asm("ldr sp, =__main_stack_end;" // reset stack pointer, it'll be used for IRQs from now on
	    "ldr r0, =%[curtask];" // get address of current task pointer
	    "ldr r0, [r0];" // get current task
	    "ldr r1, [r0, %[sp]];" // get current task stack pointer
	    "ldmia r1!, {r4-r11};" // load upper registers
	    "msr psp, r1;" // set the program stack pointer
	    "mov lr, #0xFFFFFFFD;" //
	    "bx lr;"
	    ::
	     [sp] "i" (offsetof(Task, sp)),
	     [curtask] "i" (&curtask));
}

static void sched();

extern "C" void handler_pendsv() __attribute__((naked));
extern "C" void handler_pendsv() {
	asm("ldr r0, =%[curtask];" // get the address of the current task control pointer
	    "ldr r0, [r0];" // read it to get the address of the current task control
	    "push {r0, lr};" // save the current task and its EXC_RETURN

		"bl %[sched];" // update the current task pointer
	    "pop {r0, lr};" // restore previous task and its EXC_RETURN
		"ldr r1, =%[curtask];" // get the address of the current task pointer
		"ldr r1, [r1];" // read it to get the current task control
	    "cmp r1, r0;" // compare the previous and current task pointer
	    "beq .end;" // if they're the same, no context switch, go to the end

	    "mrs r2, psp;" // get program stack pointer
	    "stmdb r2!, {r4-r11};" // store high registers to program stack pointer
		"str r2, [r0, %[sp]];" // save program stack pointer in previous task control

		"ldr r2, [r1, %[sp]];" // read the stack pointer of the current task control
	    "ldmia r2!, {r4-r11};" // restore the high registers
	    "msr psp, r2;" // set program stack pointer

	    ".end:"
	    "bx lr;" // end ISR (microcode pops lower regs)
	    ::
	     [sp] "i" (offsetof(Task, sp)),
	     [sched] "i" (&sched),
	     [curtask] "i" (&curtask));
}

static void sched() {
	if (nexttask != NULL) {
		curtask = nexttask;
		nexttask = NULL;
	} else {
		curtask = curtask->list_next();
	}

	Task *headtask = schedlist_head();
	if (curtask == NULL || curtask->priority > headtask->priority)
		curtask = headtask;
}

void sched_tick() {
	curtick++;

	Task *pos = ticklist_head();
	while (pos != nullptr && pos->waketick <= curtick) {
		Task *pos_next = pos->list_next();
		pos->remove_list();
		sched_add_task(*pos);
		pos = pos_next;
	}
}
