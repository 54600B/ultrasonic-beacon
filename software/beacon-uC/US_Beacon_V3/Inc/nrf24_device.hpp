/*
 * nrf24_device.hpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rca
 */

#ifndef NRF24_DEVICE_HPP_
#define NRF24_DEVICE_HPP_

#include "hardware_version.h"

#include "nrf24/nrf24_phy.hpp"
#include "nrf24/nrf24_data.hpp"
#include "nrf24/nrf24_config.hpp"
#include "nrf24/gpio.hpp"
#include "nrf24/spi/spi_master_2.hpp"

template <hardware_version_t version>
struct Nrf24DeviceConfig
{
private:
	using GpioOutputB12 = GpioOut<GPIOB_BASE, 12>;
	using GpioOutputD1 = GpioOut<GPIOD_BASE, 1>;

	using Cs = GpioOutputB12;
	using Ce = GpioOutputD1;
	using Spi = xpcc::stm32::SpiMaster2;

public:
	using Phy = xpcc::Nrf24Phy<Spi, Cs, Ce>;
	using Data = xpcc::Nrf24Data<Phy>;
	using Config = xpcc::Nrf24Config<Phy>;
};

template <>
struct Nrf24DeviceConfig<V3_NRF>
{
private:
	using GpioOutputB12 = GpioOut<GPIOB_BASE, 12>;
	using GpioOutputB11 = GpioOut<GPIOB_BASE, 11>;

	using Cs = GpioOutputB12;
	using Ce = GpioOutputB11;
	using Spi = xpcc::stm32::SpiMaster2;

public:
	using Phy = xpcc::Nrf24Phy<Spi, Cs, Ce>;
	using Data = xpcc::Nrf24Data<Phy>;
	using Config = xpcc::Nrf24Config<Phy>;
};

using Nrf24Device = Nrf24DeviceConfig<pcb_hardware_version>;

#endif /* NRF24_DEVICE_HPP_ */
