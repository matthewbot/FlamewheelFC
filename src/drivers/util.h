#ifndef FC_DRIVERS_UTIL_H
#define FC_DRIVERS_UTIL_H

#include <stdint.h>
#include <stm32f4xx.h>

constexpr uint32_t AFRL(int pin, int af) { return af << (4*pin); }
constexpr uint32_t AFRH(int pin, int af) { return af << (4*(pin-8)); }

constexpr uint32_t MODER_OUT(int pin) { return 1 << (2*pin); }
constexpr uint32_t MODER_AF(int pin) { return 2 << (2*pin); }
constexpr uint32_t MODER_AN(int pin) { return 3 << (2*pin); }

inline void util_enable_irq(int irqn, uint8_t priority) {
	NVIC->ISER[irqn / 32] = 1 << (irqn % 32);
	if (priority != 0)
		NVIC->IP[irqn] = priority << 4;
}

inline void util_disable_irq(int irqn) {
	NVIC->ICER[irqn / 32] = 1 << (irqn % 32);
}

inline void util_delay(int cycles) {
	while (cycles--) { __asm volatile("nop"); }
}

#endif
