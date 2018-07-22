/*
 * delay.c
 *
 */

#include "delay.h"
#include "stm32f4xx.h"

void /*__attribute__((section(".fastcode")))*/
_delay_us(uint16_t us)
{
	if (!us) return;    // 1 cycle, or 2 when taken

	uint32_t start = DWT->CYCCNT;
	// prefer this for cores with fast hardware multiplication
	int32_t delay = (int32_t)(F_CPU_MHZ) * us - 25;

	while ((int32_t)(DWT->CYCCNT - start) < delay)
		;
}


void /*__attribute__((section(".fastcode")))*/
_delay_ms(uint16_t ms)
{
	if (!ms) return;    // 1 cycle, or 2 when taken

	uint32_t start = DWT->CYCCNT;
	int32_t delay = (int32_t)(F_CPU_MHZ * 1000) * ms - 25;

	while ((int32_t)(DWT->CYCCNT - start) < delay)
		;
}
