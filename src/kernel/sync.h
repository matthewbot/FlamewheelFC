#ifndef FC_KERNEL_SYNC_H
#define FC_KERNEL_SYNC_H

#include "task.h"

class Mutex {
public:
    void reset();

    void wait();
    void release();
    bool is_owned() const { return owner != nullptr; }

private:
    Task *owner;
    ListNode wakelist;
};

class Lock {
public:
    Lock(Mutex &mutex) : mutex(mutex) { lock(); }
    ~Lock() { unlock(); }

    void lock() { mutex.wait(); }
    void unlock() { mutex.release(); }

private:
    Mutex &mutex;
};

class Signal {
public:
    void reset();

    void wait();
    void wait(Lock &lock);
    void notify();
    void notify_all();

private:
    ListNode notifylist;
};

#endif
