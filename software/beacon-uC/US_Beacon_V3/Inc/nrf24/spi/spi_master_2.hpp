// coding: utf-8
/* Copyright (c) 2013, Roboterclub Aachen e.V.
* All Rights Reserved.
*
* The file is part of the xpcc library and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/
// ----------------------------------------------------------------------------

#ifndef XPCC_STM32_SPI_MASTER2_HPP
#define XPCC_STM32_SPI_MASTER2_HPP

#include "../spi_master.hpp"
#include "spi_hal_2.hpp"
#include "type_ids.hpp"
#include "../utils.hpp"

namespace xpcc
{

namespace stm32
{

/**
 * Serial peripheral interface (SPI2).
 *
 * Simple unbuffered implementation.
 *
 * @author	Niklas Hauser
 * @ingroup	stm32f407vg_spi
 */
class SpiMaster2 : public xpcc::SpiMaster
{
	static uint8_t state;
	static uint8_t count;
	static void *context;
	static ConfigurationHandler configuration;
public:
	static const TypeId::SpiMaster2Nss  Nss;
	static const TypeId::SpiMaster2Sck  Sck;
	static const TypeId::SpiMaster2Miso Miso;
	static const TypeId::SpiMaster2Mosi Mosi;

	/// Spi Data Mode, Mode0 is the most common mode
	enum class
	DataMode : uint32_t
	{
		Mode0 = 0b00,			///< clock normal,   sample on rising  edge
		Mode1 = SPI_CR1_CPHA,	///< clock normal,   sample on falling edge
		Mode2 = SPI_CR1_CPOL,	///< clock inverted, sample on falling  edge
		Mode3 = SPI_CR1_CPOL | SPI_CR1_CPHA
		///< clock inverted, sample on rising edge
	};

	/// Spi Data Order, MsbFirst is the most common mode
	enum class
	DataOrder : uint32_t
	{
		MsbFirst = 0b0,
		LsbFirst = SPI_CR1_LSBFIRST
	};

public:
	// start documentation inherited
	template< uint32_t Spi2Clk, uint32_t baudrate,
			uint16_t tolerance = xpcc::Tolerance::FivePercent >
	static void
	initialize()
	{
		// calculate the nearest prescaler from the baudrate
		constexpr float desired = float(Spi2Clk) / baudrate;
		constexpr uint16_t nearest = (
				(desired >= (128 * 4.f/3)) ? 256 : (
				(desired >= ( 64 * 4.f/3)) ? 128 : (
				(desired >= ( 32 * 4.f/3)) ?  64 : (
				(desired >= ( 16 * 4.f/3)) ?  32 : (
				(desired >= (  8 * 4.f/3)) ?  16 : (
				(desired >= (  4 * 4.f/3)) ?   8 : (
				(desired >= (  2 * 4.f/3)) ?   4 :
											   2
				)))))));

		// check if we found a prescaler which generates a baudrate within the tolerance
		assertBaudrateInTolerance<
				Spi2Clk / nearest,
				baudrate,
				tolerance >();

		// translate the prescaler into the bitmapping
		constexpr SpiHal2::Prescaler prescaler = (
				(nearest >= 256) ? SpiHal2::Prescaler::Div256 : (
				(nearest >= 128) ? SpiHal2::Prescaler::Div128 : (
				(nearest >=  64) ? SpiHal2::Prescaler::Div64  : (
				(nearest >=  32) ? SpiHal2::Prescaler::Div32  : (
				(nearest >=  16) ? SpiHal2::Prescaler::Div16  : (
				(nearest >=   8) ? SpiHal2::Prescaler::Div8   : (
				(nearest >=   4) ? SpiHal2::Prescaler::Div4   :
								   SpiHal2::Prescaler::Div2
				)))))));

		// initialize the Spi
		SpiHal2::initialize(prescaler);
		state = 0;
	}


	static xpcc_always_inline void
	setDataMode(DataMode mode)
	{
		SpiHal2::setDataMode(static_cast<SpiHal2::DataMode>(mode));
	}

	static xpcc_always_inline void
	setDataOrder(DataOrder order)
	{
		SpiHal2::setDataOrder(static_cast<SpiHal2::DataOrder>(order));
	}


	static uint8_t
	acquire(void *ctx, ConfigurationHandler handler = nullptr);

	static uint8_t
	release(void *ctx);

	static uint8_t
	transferBlocking(uint8_t data);

	static void
	transferBlocking(uint8_t *tx, uint8_t *rx, std::size_t length);
	// end documentation inherited
};

} // namespace stm32

} // namespace xpcc

#endif // XPCC_STM32_SPI_MASTER2_HPP
