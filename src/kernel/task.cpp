#include "task.h"

void ListNode::insert_before(ListNode &node) {
    next = &node;
    prev = node.prev;
    prev->next = this;
    next->prev = this;
}

void ListNode::insert_after(ListNode &node) {
    next = node.next;
    prev = &node;
    prev->next = this;
    next->prev = this;
}

void ListNode::insert_end(ListNode &node) {
    ListNode *cur = &node;
    while (cur->next != nullptr)
        cur = cur->next;
    insert_after(*cur);
}

void ListNode::remove() {
    if (next != nullptr)
        next->prev = prev;
    if (prev != nullptr)
        prev->next = next;
    next = nullptr;
    prev = nullptr;
}

void Task::insert_list_priority_sorted(ListNode &node) {
    list.insert_sorted(node, &Task::list, &Task::priority);
}

void Task::insert_list_waketick_sorted(ListNode &node) {
    list.insert_sorted(node, &Task::list, &Task::waketick);
}

void Task::setup(const char *name, uint8_t priority, TaskFunc func, void *funcdata, void *stack, size_t stacksize) {
    regs = (TaskRegs *)((char *)stack + stacksize - sizeof(TaskRegs));
    regs->xPSR = 0x01000000;
    regs->pc = (uint32_t)func;
    regs->r0 = (uint32_t)funcdata;
    this->priority = priority;
    strncpy(this->name, name, sizeof(this->name));
}
