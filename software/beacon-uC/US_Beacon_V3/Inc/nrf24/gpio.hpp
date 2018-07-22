/*
 * gpio.hpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rca
 */

#ifndef NRF24_GPIO_HPP_
#define NRF24_GPIO_HPP_

#include <stm32f407xx.h>
#include "utils.hpp"

template<intptr_t PortAddr, uint8_t Pin>
struct GpioOut
{
	static_assert(Pin < 16, "pin out of range");

	static volatile GPIO_TypeDef* const Port;

	static constexpr uint16_t mask  = 0x1 << Pin;
	static constexpr uint32_t mask2 = 0x3 << (Pin * 2);

	static void xpcc_always_inline set()
	{
		Port->BSRR = mask;
	}

	static void xpcc_always_inline reset()
	{
		Port->BSRR = (uint32_t(mask) << 16);
	}

	static bool xpcc_always_inline isSet()
	{
		return (Port->ODR & mask);
	}

	static void xpcc_always_inline toggle()
	{
		if (isSet()) {
			reset();
		} else {
			set();
		}
	}
};

template<intptr_t PortAddr, uint8_t Pin>
volatile GPIO_TypeDef* const GpioOut<PortAddr, Pin>::Port = (volatile GPIO_TypeDef* const) PortAddr;

#endif /* NRF24_GPIO_HPP_ */
