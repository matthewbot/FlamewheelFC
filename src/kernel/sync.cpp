#include "sync.h"
#include "sched.h"
#include "kernel.h"

void Mutex::reset() {
    owner = nullptr;
    wakelist.next = wakelist.prev = nullptr;
}

void Mutex::wait() {
    Task *curtask = sched_current_task();

    while (true) {
        KernelCriticalSection crit;
        if (owner == nullptr) {
            owner = curtask;
            break;
        }

        sched_remove_task(*curtask);
        curtask->insert_list_priority_sorted(wakelist);
        sched_yield();
    }
}

void Mutex::release() {
    KernelCriticalSection crit;
    owner = nullptr;
    if (wakelist.next) {
        Task &waketask = wakelist.next->getParent(&Task::list);
        waketask.remove_list();
        sched_add_task(waketask);
    }
}

void Signal::reset() {
    notifylist.next = notifylist.prev = nullptr;
}

void Signal::wait() {
    Task *curtask = sched_current_task();

    KernelCriticalSection crit;
    sched_remove_task(*curtask);
    curtask->insert_list_priority_sorted(notifylist);
    sched_yield();
}

void Signal::wait(Lock &lock) {
    Task *curtask = sched_current_task();

    {
        KernelCriticalSection crit;
        lock.unlock();
        sched_remove_task(*curtask);
        curtask->insert_list_priority_sorted(notifylist);
        sched_yield();
    }

    lock.lock();
}

void Signal::notify() {
    KernelCriticalSection crit;
    if (notifylist.next) {
        Task &waketask = notifylist.next->getParent(&Task::list);
        waketask.remove_list();
        sched_add_task(waketask);
    }
}

void Signal::notify_all() {
    KernelCriticalSection crit;
    while (notifylist.next) {
        Task &waketask = notifylist.next->getParent(&Task::list);
        waketask.remove_list();
        sched_add_task(waketask);
    }
}
