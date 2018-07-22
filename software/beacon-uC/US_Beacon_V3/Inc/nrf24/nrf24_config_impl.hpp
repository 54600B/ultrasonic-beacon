// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */

#ifndef XPCC__NRF24_CONFIG_HPP
#   error "Don't include this file directly, use 'nrf24_config.hpp' instead!"
#endif

#include "nrf24_config.hpp"
#include "nrf24_definitions.hpp"

#include <stdint.h>


template<typename Nrf24Phy>
typename xpcc::Nrf24Config<Nrf24Phy>::Mode
xpcc::Nrf24Config<Nrf24Phy>::currentMode = Mode::Invalid;

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
void
xpcc::Nrf24Config<Nrf24Phy>::setMode(Mode mode)
{
	if(mode == currentMode) {
		// already in this mode
		return;
	}

	switch(mode) {
	case Mode::Rx:
		//XPCC_LOG_DEBUG << "[nrf24] Set mode Rx" << xpcc::endl;
//		Nrf24Phy::resetCe();
//		xpcc::delayMicroseconds(30);
		Nrf24Phy::flushRxFifo();
		Nrf24Phy::setBits(NrfRegister::CONFIG, Config::PRIM_RX);

		break;
	case Mode::Tx:
		//XPCC_LOG_DEBUG << "[nrf24] Set mode Tx" << xpcc::endl;

		Nrf24Phy::flushTxFifo();
		Nrf24Phy::clearBits(NrfRegister::CONFIG, Config::PRIM_RX);

		// pulsing CE seems to be necessary to enter TX mode
		//Nrf24Phy::pulseCe();
		break;
	case Mode::Invalid:
		// do nothing
		return;
	}

	// don't go to standby
	Nrf24Phy::setCe();

	currentMode = mode;
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
void
xpcc::Nrf24Config<Nrf24Phy>::setSpeed(Speed speed)
{
	switch (speed) {
	case Speed::kBps250:
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_HIGH);
		Nrf24Phy::setBits  (NrfRegister::RF_SETUP, RfSetup::RF_DR_LOW);
		break;
	case Speed::MBps1:
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_HIGH);
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_LOW);
		break;
	case Speed::MBps2:
		Nrf24Phy::setBits  (NrfRegister::RF_SETUP, RfSetup::RF_DR_HIGH);
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_LOW);
		break;
	}
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
typename xpcc::Nrf24Config<Nrf24Phy>::Speed
xpcc::Nrf24Config<Nrf24Phy>::getSpeed()
{
	const uint8_t rf_setup = Nrf24Phy::readRegister(NrfRegister::RF_SETUP);

	if(rf_setup & static_cast<uint8_t>(RfSetup::RF_DR_LOW)) {
		return Speed::kBps250;
	} else if(rf_setup & static_cast<uint8_t>(RfSetup::RF_DR_HIGH)) {
		return Speed::MBps2;
	} else {
		return Speed::MBps1;
	}
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setCrc(Crc crc)
{
	if(crc == Crc::NoCrc) {
		Nrf24Phy::clearBits(NrfRegister::CONFIG, Config::EN_CRC);
	} else {
		Nrf24Phy::setBits(NrfRegister::CONFIG, Config::EN_CRC);

		if (crc == Crc::Crc1Byte) {
			Nrf24Phy::clearBits(NrfRegister::CONFIG, Config::CRC0);
		} else if (crc == Crc::Crc2Byte) {
			Nrf24Phy::setBits(NrfRegister::CONFIG, Config::CRC0);
		}
	}
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setRfPower(RfPower power)
{
	uint8_t reg = Nrf24Phy::readRegister(NrfRegister::RF_SETUP);
	reg &= ~(static_cast<uint8_t>(RfSetup::RF_PWR));	// Clear bits
	reg |=  ((static_cast<uint8_t>(power)) << 1); 		// Set bits
	Nrf24Phy::writeRegister(NrfRegister::RF_SETUP, reg);
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setAutoRetransmitDelay(AutoRetransmitDelay delay)
{
	uint8_t reg = Nrf24Phy::readRegister(NrfRegister::SETUP_RETR);
	reg &= ~(static_cast<uint8_t>(SetupRetr::ARD));		// Clear bits
	reg |=  ((static_cast<uint8_t>(delay)) << 4); 		// Set bits
	Nrf24Phy::writeRegister(NrfRegister::SETUP_RETR, reg);
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setAutoRetransmitCount(AutoRetransmitCount count)
{
	uint8_t reg = Nrf24Phy::readRegister(NrfRegister::SETUP_RETR);
	reg &= ~(static_cast<uint8_t>(SetupRetr::ARC));		// Clear bits
	reg |=  (static_cast<uint8_t>(count)); 				// Set bits
	Nrf24Phy::writeRegister(NrfRegister::SETUP_RETR, reg);
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
void
xpcc::Nrf24Config<Nrf24Phy>::enablePipe(Pipe_t pipe, bool enableAutoAck)
{
	// Set payload width
	NrfRegister_t reg = NrfRegister::RX_PW_P0;
	reg.value += pipe.value;
	Nrf24Phy::writeRegister(reg, Nrf24Phy::getPayloadLength());

	const Flags_t pipeBit(1 << pipe.value);

	// Enable or disable auto acknowledgment
	if(enableAutoAck) {
		Nrf24Phy::setBits(NrfRegister::EN_AA, pipeBit);
	} else {
		Nrf24Phy::clearBits(NrfRegister::EN_AA, pipeBit);
	}

	// Enable pipe
	Nrf24Phy::setBits(NrfRegister::EN_RX_ADDR, pipeBit);
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy>
typename xpcc::Nrf24Config<Nrf24Phy>::Pipe_t
xpcc::Nrf24Config<Nrf24Phy>::getPayloadPipe()
{
	const uint8_t status = Nrf24Phy::readStatus();
	return Pipe_t((status & static_cast<uint8_t>(Status::RX_P_NO)) >> 1);
}
