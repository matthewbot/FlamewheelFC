#ifndef KERNEL_TASK_H
#define KERNEL_TASK_H

#include <stdint.h>
#include <string.h>

struct TaskPerf {
	uint32_t curcycles;
	uint32_t prevcycles;
};

typedef void (*TaskFunc)(void *);

struct TaskRegs {
	uint32_t r4, r5, r6, r7, r8, r9, r10, r11;
	uint32_t r0, r1, r2, r3, r12, r14;
	uint32_t pc;
	uint32_t xPSR;
};

struct TaskFPURegs {
	float s[32];
	uint32_t FPSCR;
};

struct ListNode {
	ListNode *next;
	ListNode *prev;

	template <typename T>
	T &getParent(ListNode T::*mem) {
		return *(T *)((int)this - (int)&(((T *)0)->*mem));
	}

	void insert_before(ListNode &node);
	void insert_after(ListNode &node);
	void insert_end(ListNode &node);
	void remove();

	template <typename V, typename T>
	void insert_sorted(ListNode &node, ListNode T::*mem, V T::*field) {
		ListNode *cur = &node;
		V thisval = getParent(mem).*field;
		while (cur->next != NULL && thisval > cur->next->getParent(mem).*field)
			cur = cur->next;
		insert_after(*cur);
	}
};

struct Task {
	enum class State : uint8_t { NONE, SCHEDULED, SLEEP };

	typedef uint8_t Priority;
	static const Priority MIN = 0xFF;
	static const Priority LOW = 0xB0;
	static const Priority MED = 0x80;
	static const Priority HIGH = 0x40;
	static const Priority MAX = 0;

	ListNode list;
	ListNode tasks;

	union {
		void *sp;
		TaskRegs *regs;
	};
	TaskFPURegs fpuregs;

	char name[14];
	uint8_t priority;
	State state;

	uint32_t waketick;

	TaskPerf perf;
	uint16_t stackguard;

	static const uint16_t STACKGUARD_VALUE = 0xABCD;

	void insert_list_priority_sorted(ListNode &node);
	void insert_list_waketick_sorted(ListNode &node);
	void remove_list() { list.remove(); }
	Task *list_next() { return list.next ? &list.next->getParent(&Task::list) : nullptr; }
	Task *list_prev() { return list.prev ? &list.prev->getParent(&Task::list) : nullptr; }

	void setup(const char *name, uint8_t priority, TaskFunc func, void *funcdata, void *stack, size_t stacksize);
};

#define DECLARE_TASK_STACK(name, bytes) static uint8_t name[bytes] __attribute__((section(".stack")))
#define DECLARE_TASK_FUNC(name) static void name(void *unused) __attribute__((noreturn))

#endif
