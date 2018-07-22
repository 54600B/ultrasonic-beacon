// coding: utf-8
/* Copyright (c) 2013, Roboterclub Aachen e.V.
* All Rights Reserved.
*
* The file is part of the xpcc library and is released under the 3-clause BSD
* license. See the file `LICENSE` for the full license governing this code.
*/
// ----------------------------------------------------------------------------

#include "nrf24/spi/spi_master_2.hpp"
#include "nrf24/register.hpp"
#include "us_beacon_base.h"

// Bit0: single transfer state
// Bit1: block transfer state
uint8_t
xpcc::stm32::SpiMaster2::state(0);

uint8_t
xpcc::stm32::SpiMaster2::count(0);

void *
xpcc::stm32::SpiMaster2::context(nullptr);

xpcc::Spi::ConfigurationHandler
xpcc::stm32::SpiMaster2::configuration(nullptr);
// ----------------------------------------------------------------------------

uint8_t
xpcc::stm32::SpiMaster2::acquire(void *ctx, ConfigurationHandler handler)
{
	if (context == nullptr)
	{
		context = ctx;
		count = 1;
		// if handler is not nullptr and is different from previous configuration
		if (handler and configuration != handler) {
			configuration = handler;
			configuration();
		}
		return 1;
	}

	if (ctx == context)
		return ++count;

	return 0;
}

uint8_t
xpcc::stm32::SpiMaster2::release(void *ctx)
{
	if (ctx == context)
	{
		if (--count == 0)
			context = nullptr;
	}
	return count;
}
// ----------------------------------------------------------------------------

uint8_t
xpcc::stm32::SpiMaster2::transferBlocking(uint8_t data)
{
	// wait for previous transfer to finish
	while(!SpiHal2::isTransmitRegisterEmpty());

	// start transfer by copying data into register
	SpiHal2::write(data);

	while(!SpiHal2::isReceiveRegisterNotEmpty());

	uint8_t result = 0;
	SpiHal2::read(result);

	return result;
}

void
xpcc::stm32::SpiMaster2::transferBlocking(
		uint8_t* tx, uint8_t* rx, std::size_t length)
{
	for(std::size_t index = 0; index < length; ++index) {
		const auto result = transferBlocking(tx ? tx[index] : 0);
		if(rx) {
			rx[index] = result;
		}
	}
}
