// coding: utf-8
/* Copyright (c) 2013, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC_STM32_SPI_BASE_HPP
#define XPCC_STM32_SPI_BASE_HPP

#include <stdint.h>
#include <stm32f407xx.h>
#include "../register.hpp"

/**
 * @ingroup 	stm32f407vg
 * @defgroup	stm32f407vg_spi SPI
 */

namespace xpcc
{

namespace stm32
{

/**
 * Base class for the SPI classes
 *
 * Provides some common enum that do not depend on the specific SPI.
 *
 * @author Kevin Laeufer
 * @internal
 * @ingroup		stm32f407vg_spi
 */
class SpiBase
{
public:
	enum class
	Interrupt : uint32_t
	{
		RxBufferNotEmpty	= SPI_CR2_RXNEIE,
		TxBufferEmpty		= SPI_CR2_TXEIE,
		Error				= SPI_CR2_ERRIE,
	};
	XPCC_FLAGS32(Interrupt);

	enum class
	InterruptFlag : uint32_t
	{
		TxBufferEmpty		= SPI_SR_TXE,
		RxBufferNotEmpty	= SPI_SR_RXNE,
		CrcError			= SPI_SR_CRCERR,
		ModeFaultError		= SPI_SR_MODF,
		OverrunError		= SPI_SR_OVR,
		Busy				= SPI_SR_BSY,
		//FrameFormatError	= SPI_SR_FRE,
	};
	XPCC_FLAGS32(InterruptFlag);

	enum class
	MasterSelection : uint32_t
	{
		Slave 	= 0,			///< Configure SPI as Slave
		Master 	= SPI_CR1_MSTR,	///< Configure SPI as Master
		All 	= Master,
	};

	enum class
	DataMode : uint32_t
	{
		Mode0 = 0b00,			///< clock normal,   sample on rising  edge
		Mode1 = SPI_CR1_CPHA,	///< clock normal,   sample on falling edge
		Mode2 = SPI_CR1_CPOL,	///< clock inverted, sample on falling  edge
		Mode3 = SPI_CR1_CPOL | SPI_CR1_CPHA,
		///< clock inverted, sample on rising edge
		All = Mode3
	};

	enum class
	DataOrder : uint32_t
	{
		MsbFirst = 0b0,
		LsbFirst = SPI_CR1_LSBFIRST,
		All = LsbFirst,
	};

	enum class
	DataSize : uint32_t
	{
		Bit8  = 0,
		Bit16 = SPI_CR1_DFF,
		All   = Bit16,
	};

	enum class
	Prescaler : uint32_t
	{
		Div2 	= 0,
		Div4 	= SPI_CR1_BR_0,
		Div8 	= SPI_CR1_BR_1,
		Div16 	= SPI_CR1_BR_1 | SPI_CR1_BR_0,
		Div32 	= SPI_CR1_BR_2,
		Div64 	= SPI_CR1_BR_2 | SPI_CR1_BR_0,
		Div128 	= SPI_CR1_BR_2 | SPI_CR1_BR_1,
		Div256 	= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
	};

};

} // namespace stm32

} // namespace xpcc

#endif // XPCC_STM32_SPI_BASE_HPP
