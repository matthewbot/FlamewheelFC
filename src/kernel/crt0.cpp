#include "linker.h"
#include "kernel.h"
#include <stdint.h>
#include <cstdlib>
#include "stm32f4xx.h"

static void setup_clock();
static void copy(uint32_t *start, uint32_t *end, const uint32_t *load);
static void fill(uint32_t *start, const uint32_t *end, uint32_t val);

extern "C" void handler_reset() __attribute__((naked, noreturn));
extern "C" void handler_reset() {
	setup_clock();
	copy(&__data_start, &__data_end, &__data_load);
	fill(&__bss_start, &__bss_end, 0);
	kernel_start();
}

#define RCC_CFGR_RTCPRE_POS 16
#define RCC_PLLCFGR_PLLQ_POS 24
#define RCC_PLLCFGR_PLLP_POS 16
#define RCC_PLLCFGR_PLLN_POS 6
#define RCC_PLLCFGR_PLLM_POS 0

static void setup_clock() {
	FLASH->ACR =
		FLASH_ACR_LATENCY_5WS |  // Need 5 wait states at 168 Mhz
		FLASH_ACR_DCEN | // enable data cache
		FLASH_ACR_ICEN | // enable instruction cache
		FLASH_ACR_PRFTEN; // enable prefetching
	RCC->CFGR = (8 << RCC_CFGR_RTCPRE_POS) | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4; // Setup APB2 to 84 Mhz and APB1 to 48 Mhz
	RCC->PLLCFGR =
		(7 << RCC_PLLCFGR_PLLQ_POS) | // 336 / 7 = 48 Mhz for USB/RNG
		(0 << RCC_PLLCFGR_PLLP_POS) | // 336 / (0+2) = 168 Mhz for system clock
		(168 << RCC_PLLCFGR_PLLN_POS) | // 2 * 168 = 336 Mhz for VCO
		(8 << RCC_PLLCFGR_PLLM_POS); // 16 / 8 = 2 Mhz for VCO input
	RCC->CR |= RCC_CR_PLLON; // enable PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)) { } // wait for it to come up
	RCC->CFGR |= RCC_CFGR_SW_PLL; // set PLL as system clock source
}

static void copy(uint32_t *start, uint32_t *end, const uint32_t *load) {
	while (start != end)
		*start++ = *load++;
}

static void fill(uint32_t *start, const uint32_t *end, uint32_t val) {
	while (start != end)
		*start++ = val;
}

extern "C" void handler_fault() __attribute__((weak));
extern "C" void handler_fault() {
	while (true) { }
}

#define DECLARE_HANDLER(name) extern "C" void handler_ ## name() __attribute((weak, alias("handler_fault")))
DECLARE_HANDLER(svcall);
DECLARE_HANDLER(pendsv);
DECLARE_HANDLER(systick);
DECLARE_HANDLER(usagefault);
#undef DECLARE_HANDLER

#define DECLARE_IRQ(name) extern "C" void irq_ ## name() __attribute__((weak, alias("handler_fault")))
DECLARE_IRQ(wwdg);
DECLARE_IRQ(pvd);
DECLARE_IRQ(tamp_stamp);
DECLARE_IRQ(rtc_wkup);
DECLARE_IRQ(flash);
DECLARE_IRQ(rcc);
DECLARE_IRQ(ext0);
DECLARE_IRQ(ext1);
DECLARE_IRQ(ext2);
DECLARE_IRQ(ext3);
DECLARE_IRQ(ext4);
DECLARE_IRQ(dma1_stream0);
DECLARE_IRQ(dma1_stream1);
DECLARE_IRQ(dma1_stream2);
DECLARE_IRQ(dma1_stream3);
DECLARE_IRQ(dma1_stream4);
DECLARE_IRQ(dma1_stream5);
DECLARE_IRQ(dma1_stream6);
DECLARE_IRQ(adc);
DECLARE_IRQ(can1_tx);
DECLARE_IRQ(can1_rx0);
DECLARE_IRQ(can1_rx1);
DECLARE_IRQ(can1_cse);
DECLARE_IRQ(exti95);
DECLARE_IRQ(tim1_brk_tim9);
DECLARE_IRQ(tim1_up_tim10);
DECLARE_IRQ(tim1_trg_com_tim11);
DECLARE_IRQ(tim1_cc);
DECLARE_IRQ(tim2);
DECLARE_IRQ(tim3);
DECLARE_IRQ(tim4);
DECLARE_IRQ(i2c1_ev);
DECLARE_IRQ(i2c1_er);
DECLARE_IRQ(i2c2_ev);
DECLARE_IRQ(i2c2_er);
DECLARE_IRQ(spi1);
DECLARE_IRQ(spi2);
DECLARE_IRQ(usart1);
DECLARE_IRQ(usart2);
DECLARE_IRQ(usart3);
DECLARE_IRQ(exti1510);
DECLARE_IRQ(rtc_alarm);
DECLARE_IRQ(otg_fs_wkup);
DECLARE_IRQ(tim8_brk_tim12);
DECLARE_IRQ(tim8_up_tim13);
DECLARE_IRQ(tim8_trg_com_tim14);
DECLARE_IRQ(tim8_cc);
DECLARE_IRQ(dma1_stream7);
DECLARE_IRQ(fsmc);
DECLARE_IRQ(sdio);
DECLARE_IRQ(tim5);
DECLARE_IRQ(spi3);
DECLARE_IRQ(uart4);
DECLARE_IRQ(uart5);
DECLARE_IRQ(tim6_dac);
DECLARE_IRQ(tim7);
DECLARE_IRQ(dma2_stream0);
DECLARE_IRQ(dma2_stream1);
DECLARE_IRQ(dma2_stream2);
DECLARE_IRQ(dma2_stream3);
DECLARE_IRQ(dma2_stream4);
DECLARE_IRQ(eth);
DECLARE_IRQ(eth_wkup);
DECLARE_IRQ(can2_tx);
DECLARE_IRQ(can2_rx0);
DECLARE_IRQ(can2_rx1);
DECLARE_IRQ(can2_sce);
DECLARE_IRQ(otg_fs);
DECLARE_IRQ(dma2_stream5);
DECLARE_IRQ(dma2_stream6);
DECLARE_IRQ(dma2_stream7);
DECLARE_IRQ(usart6);
DECLARE_IRQ(i2c3_ev);
DECLARE_IRQ(i2c3_er);
DECLARE_IRQ(otg_hs_ep1_out);
DECLARE_IRQ(otg_hs_ep1_in);
DECLARE_IRQ(otg_hs_wkup);
DECLARE_IRQ(hs);
DECLARE_IRQ(hcmi);
DECLARE_IRQ(cryp);
DECLARE_IRQ(hash_rng);
DECLARE_IRQ(fpu);
#undef DECLARE_IRQ

extern "C" {
	struct ISRVector {
		void *sp;
		void (*exceptions[15])();
		void (*interrupts[82])();
	};

	const ISRVector isr_vector __attribute__ ((section(".isr_vector"), used)) = {
		&__main_stack_end,
		{
			&handler_reset,
			&handler_fault,
			&handler_fault,
			&handler_fault,
			&handler_fault,
			&handler_fault,
			nullptr,
			nullptr,
			nullptr,
			nullptr,
			&handler_svcall,
			nullptr,
			nullptr,
			&handler_pendsv,
			&handler_systick,
		}, {
			&irq_wwdg,
			&irq_pvd,
			&irq_tamp_stamp,
			&irq_rtc_wkup,
			&irq_flash,
			&irq_rcc,
			&irq_ext0,
			&irq_ext1,
			&irq_ext2,
			&irq_ext3,
			&irq_ext4,
			&irq_dma1_stream0,
			&irq_dma1_stream1,
			&irq_dma1_stream2,
			&irq_dma1_stream3,
			&irq_dma1_stream4,
			&irq_dma1_stream5,
			&irq_dma1_stream6,
			&irq_adc,
			&irq_can1_tx,
			&irq_can1_rx0,
			&irq_can1_rx1,
			&irq_can1_cse,
			&irq_exti95,
			&irq_tim1_brk_tim9,
			&irq_tim1_up_tim10,
			&irq_tim1_trg_com_tim11,
			&irq_tim1_cc,
			&irq_tim2,
			&irq_tim3,
			&irq_tim4,
			&irq_i2c1_ev,
			&irq_i2c1_er,
			&irq_i2c2_ev,
			&irq_i2c2_er,
			&irq_spi1,
			&irq_spi2,
			&irq_usart1,
			&irq_usart2,
			&irq_usart3,
			&irq_exti1510,
			&irq_rtc_alarm,
			&irq_otg_fs_wkup,
			&irq_tim8_brk_tim12,
			&irq_tim8_up_tim13,
			&irq_tim8_trg_com_tim14,
			&irq_tim8_cc,
			&irq_dma1_stream7,
			&irq_fsmc,
			&irq_sdio,
			&irq_tim5,
			&irq_spi3,
			&irq_uart4,
			&irq_uart5,
			&irq_tim6_dac,
			&irq_tim7,
			&irq_dma2_stream0,
			&irq_dma2_stream1,
			&irq_dma2_stream2,
			&irq_dma2_stream3,
			&irq_dma2_stream4,
			&irq_eth,
			&irq_eth_wkup,
			&irq_can2_tx,
			&irq_can2_rx0,
			&irq_can2_rx1,
			&irq_can2_sce,
			&irq_otg_fs,
			&irq_dma2_stream5,
			&irq_dma2_stream6,
			&irq_dma2_stream7,
			&irq_usart6,
			&irq_i2c3_ev,
			&irq_i2c3_er,
			&irq_otg_hs_ep1_out,
			&irq_otg_hs_ep1_in,
			&irq_otg_hs_wkup,
			&irq_hs,
			&irq_hcmi,
			&irq_cryp,
			&irq_hash_rng,
			&irq_fpu
		}
	};
}
