#ifndef FC_DRIVERS_UTIL_H
#define FC_DRIVERS_UTIL_H

#include "kernel/kernel.h"
#include <stdint.h>
#include <stdlib.h>
#include <stm32f4xx.h>

// GPIO utilities
constexpr uint32_t AFRL(int pin, int af) { return af << (4*pin); }
constexpr uint32_t AFRH(int pin, int af) { return af << (4*(pin-8)); }

constexpr uint32_t MODER_OUT(int pin) { return 1 << (2*pin); }
constexpr uint32_t MODER_AF(int pin) { return 2 << (2*pin); }
constexpr uint32_t MODER_AN(int pin) { return 3 << (2*pin); }
constexpr uint32_t MODER_MASK(int pin) { return 3 << (2*pin); }

constexpr uint32_t PUPDR_UP(int pin) { return 1 << (2*pin); }
constexpr uint32_t PUPDR_DOWN(int pin) { return 2 << (2*pin); }
constexpr uint32_t PUPDR_MASK(int pin) { return 3 << (2*pin); }

// GPIO utilities
constexpr uint32_t UART_BRR(int clk, int baud) { return (float)clk / baud + 0.5f; }

// NVIC utilities
constexpr uint8_t IRQ_PRI_KERNEL = 0xF;
constexpr uint8_t IRQ_PRI_LOW = 0xE;
constexpr uint8_t IRQ_PRI_MED = 0x9;
constexpr uint8_t IRQ_PRI_HIGH = 0x4;

inline void util_enable_irq(int irqn) {
    NVIC->ISER[irqn / 32] = 1 << (irqn % 32);
}

inline void util_enable_irq(int irqn, uint8_t priority) {
    util_enable_irq(irqn);
    if (priority != 0)
        NVIC->IP[irqn] = priority << 4;
}

inline void util_disable_irq(int irqn) {
    NVIC->ICER[irqn / 32] = 1 << (irqn % 32);
}

// Misc stuff
inline void util_delay(int cycles) {
    while (cycles--) { __asm volatile("nop"); }
}

template <typename T, size_t N>
class RingBuffer {
public:
    void reset() {
        readpos = writepos = cnt = 0;
    }

    bool full() const {
        return cnt == N;
    }

    bool empty() const {
        return cnt == 0;
    }

    size_t count() const {
        return cnt;
    }

    void put(T val) {
        buf[writepos++] = val;
        writepos %= N;
        cnt++;
    }

    T get() {
        T val = buf[readpos++];
        readpos %= N;
        cnt--;
        return val;
    }

private:
    int writepos;
    int readpos;
    int cnt;
    T buf[N];
};

template <int IRQ>
struct IRQCriticalSection : public KernelCriticalSection {
    IRQCriticalSection() { util_disable_irq(IRQ); }
    ~IRQCriticalSection() { util_enable_irq(IRQ); }
};

#endif
