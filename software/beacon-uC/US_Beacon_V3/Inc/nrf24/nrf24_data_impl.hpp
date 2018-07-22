// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */

#ifndef XPCC__NRF24_DATA_HPP
#   error "Don't include this file directly, use 'nrf24_data.hpp' instead!"
#endif

#include "nrf24_data.hpp"
#include <string.h>
#include <stdint.h>

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::WARNING

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
typename xpcc::Nrf24Data<Nrf24Phy, Clock>::BaseAddress
xpcc::Nrf24Data<Nrf24Phy, Clock>::baseAddress;

template<typename Nrf24Phy, typename Clock>
typename xpcc::Nrf24Data<Nrf24Phy, Clock>::Address
xpcc::Nrf24Data<Nrf24Phy, Clock>::broadcastAddress;

template<typename Nrf24Phy, typename Clock>
typename xpcc::Nrf24Data<Nrf24Phy, Clock>::Address
xpcc::Nrf24Data<Nrf24Phy, Clock>::ownAddress;

template<typename Nrf24Phy, typename Clock>
typename xpcc::Nrf24Data<Nrf24Phy, Clock>::Feedback
xpcc::Nrf24Data<Nrf24Phy, Clock>::feedbackLastPacket;

template<typename Nrf24Phy, typename Clock>
typename xpcc::Nrf24Data<Nrf24Phy, Clock>::Feedback
xpcc::Nrf24Data<Nrf24Phy, Clock>::feedbackCurrentPacket;

template<typename Nrf24Phy, typename Clock>
xpcc::Timeout
xpcc::Nrf24Data<Nrf24Phy, Clock>::sendingInterruptTimeout;

// -----------------------------------------------------------------------------


template<typename Nrf24Phy, typename Clock>
void
xpcc::Nrf24Data<Nrf24Phy, Clock>::initialize(BaseAddress base_address, Address own_address, Address broadcast_address)
{
	// Set base address and clear lower byte. When assembling full addresses,
	// each Address (1 byte) will be ORed to the lower byte of this base address
	baseAddress = base_address & ~(static_cast<uint64_t>(0xff));

	// reset sending feedback
	feedbackLastPacket.sendingFeedback = SendingFeedback::Undefined;
	feedbackCurrentPacket.sendingFeedback = SendingFeedback::Undefined;

	// initialize physical layer for max. payload size
	Phy::initialize(Phy::getMaxPayload());

	// Set to fixed address length of 5 byte for now
	Config::setAddressWidth(Config::AddressWidth::Byte5);

	// Setup addresses
	setAddress(own_address);
	setBroadcastAddress(broadcast_address);

	// Setup pipe 0 that will be used to receive acks and therefore configured
	// with an address to listen for. We want to already enable it, but not
	// setting an address could lead to erroneous packet coming from noise.
	// So we set it to the bitwise negated base address.
	Phy::setRxAddress(Pipe::PIPE_0, ~assembleAddress(0x55));

	// It was observed that when set to 'false' no packet is received until the
	// module sends a packet once, although we don't want to sent ACKs here.
	Config::enablePipe(Pipe::PIPE_0, true);

	// Enable feature 'EN_DYN_ACK' to be able to send packets without expecting
	// an ACK as response (used for transmitting to broadcast address)
	Config::enableFeatureNoAck();

	// Flush Fifos just to be sure
	Phy::flushRxFifo();
	Phy::flushTxFifo();

	// enable pipes for broadcast and own address
	// enabling auto ACK seems to be needed for the broadcast pipe too, although
	// broadcast packets will be sent with NO_ACK bit set
	Config::enablePipe(Pipe::PIPE_1, true);
	Config::enablePipe(Pipe::PIPE_2, true);

	// Configure some sensible defaults, may be changed later by user, but
	// should be consistent among all other nodes
	Config::setCrc(Config::Crc::Crc1Byte);
	Config::setSpeed(Config::Speed::kBps250);
	Config::setAutoRetransmitDelay(Config::AutoRetransmitDelay::us1000);
	Config::setAutoRetransmitCount(Config::AutoRetransmitCount::Retransmit15);
	Config::setRfPower(Config::RfPower::dBm0);

	// Only enable Data Ready interrupt for timestamping of incoming packets
	Config::disableInterrupt(Config::InterruptFlag::ALL);
	Config::enableInterrupt(Config::InterruptFlag::RX_DR);

	// Power up module in Rx mode
	Config::setMode(Config::Mode::Rx);
	Config::powerUp();

	// don't save power, always in Rx-mode or standby-2 (see table 15, p. 24)
	Phy::setCe();
}

template<typename Nrf24Phy, typename Clock>
void
xpcc::Nrf24Data<Nrf24Phy, Clock>::interruptHandler()
{
	// capture time for precise timing
	feedbackCurrentPacket.timestamp = Clock::now();
	feedbackCurrentPacket.timestampTrusted = true;
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
void
xpcc::Nrf24Data<Nrf24Phy, Clock>::setAddress(Address address)
{
	ownAddress = address;
	Phy::setRxAddress(Pipe::PIPE_2, assembleAddress(address));
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
void
xpcc::Nrf24Data<Nrf24Phy, Clock>::setBroadcastAddress(Address address)
{
	broadcastAddress = address;
	Phy::setRxAddress(Pipe::PIPE_1, assembleAddress(address));
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
bool
xpcc::Nrf24Data<Nrf24Phy, Clock>::sendPacket(Packet& packet, bool retransmit)
{
	if(not isReadyToSend()) {
		//XPCC_LOG_WARNING << "[nrf24] Warning: Not ready to send, current state: "
		//                << toStr(feedbackLastPacket.sendingFeedback) << xpcc::endl;
		return false;
	}

	// switch to Tx mode
	Config::setMode(Config::Mode::Tx);

	// assemble frame to transmit
	packet.setSource(ownAddress);

	const uint64_t destinationRawAddress = assembleAddress(packet.getDestination());

	// set receivers address as Tx address
	Phy::setTxAddress(destinationRawAddress);

	if(packet.getDestination() == getBroadcastAddress() || !retransmit) {
		Phy::writeTxPayloadNoAck(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));

		// as frame was sent without requesting an acknowledgment we cannot
		// determine the sending state
		feedbackLastPacket.sendingFeedback = SendingFeedback::Busy;

	} else {
		// set pipe 0's address to Tx address to receive ACK packet
		Phy::setRxAddress(Pipe::PIPE_0, destinationRawAddress);
		Config::enablePipe(Pipe::PIPE_0, true);

		Phy::writeTxPayload(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));

		// mark state as busy, so update() will switch back to Rx mode when
		// state has changed
		feedbackLastPacket.sendingFeedback = SendingFeedback::Busy;
	}

	// trigger transmission
	Phy::pulseCe();

	sendingInterruptTimeout.restart(sendingInterruptTimeoutMs);

	return true;
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
bool
xpcc::Nrf24Data<Nrf24Phy, Clock>::getPacket(Packet& packet)
{
	if(not isPacketAvailable()) {
		return false;
	}

	// when an interrupt has been missed because a pending received packet was
	// not yet pulled from the nrf24, we don't know when it arrived exactly, so
	// we tell the user that the timestamp cannot be trusted
	feedbackLastPacket.timestampTrusted = feedbackCurrentPacket.timestampTrusted;

	// feedbackLastPacket.timestamp must be accessed before the RX_DR interrupt
	// is acknowledged, because the interrupt handler might overwrite it
	// immediately. We therefore double buffer the feedback, so we can pass
	// `feedbackLastPacket` to the user after the call to `getPacket()`
	if(feedbackCurrentPacket.timestampTrusted) {
		feedbackLastPacket.timestamp = feedbackCurrentPacket.timestamp;
	} else {
		// No valid timestamp because we have missed the last interrupt,
		// probably because of queuing inside the nrf24.
		// Best effort approach: return average between now and last timestamp,
		// divide individually to avoid overflow
		feedbackLastPacket.timestamp = Timestamp(
		                        (feedbackCurrentPacket.timestamp.getTime() / 2) +
		                        (Clock::now().getTime() / 2));
	}

	// `currentFeedback` has now been migrated to `feedbackLastPacket` so we
	// reset it for the next cycle. The timestamp is retained to provide an
	// estimate when an interrupt is missed (see above)
	feedbackCurrentPacket.timestampTrusted = false;

	// acknowledge RX_DR interrupt so interrupt handler can be called again
	Phy::clearInterrupt(InterruptFlag::RX_DR);

	// read packet from nrf24 into user packet variable
	Phy::readRxPayload(reinterpret_cast<uint8_t*>(&packet));

	return true;
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
bool
xpcc::Nrf24Data<Nrf24Phy, Clock>::updateSendingState()
{
	// directly return if not busy, because nothing needs to be updated then
	if(feedbackLastPacket.sendingFeedback != SendingFeedback::Busy) {
		return true;
	}

	// read relevant status registers
	const uint8_t status = Phy::readStatus();

	if(status & static_cast<uint8_t>(Status::TX_DS)) {
		//XPCC_LOG_DEBUG << "[nrf24] Interrupt: TX_DS" << xpcc::endl;

		feedbackLastPacket.sendingFeedback = SendingFeedback::FinishedAck;

		// acknowledge TX_DS interrupt
		Phy::clearInterrupt(InterruptFlag::TX_DS);

		sendingInterruptTimeout.stop();

	} else if(status & static_cast<uint8_t>(Status::MAX_RT)) {
		//XPCC_LOG_DEBUG << "[nrf24] Interrupt: MAX_RT" << xpcc::endl;

		feedbackLastPacket.sendingFeedback = SendingFeedback::FinishedNack;

		// clear MAX_RT bit to enable further communication and flush Tx FIFO,
		// because the failed packet still resides there
		Phy::flushTxFifo();
		Phy::clearInterrupt(InterruptFlag::MAX_RT);

		sendingInterruptTimeout.stop();

	} else if(sendingInterruptTimeout.execute()) {
		//XPCC_LOG_ERROR << "[nrf24] IRQ timed out" << xpcc::endl;

		feedbackLastPacket.sendingFeedback = SendingFeedback::DontKnow;

		// We should flush the Tx FIFO because we have no clue if the packet
		// could be sent
		Phy::flushTxFifo();

	} else {
		// still busy
		return false;
	}

	// transmission is done, so we disable pipe 0 again so we don't overhear any
	// packets that are destined to that node (sent by others)
	Config::disablePipe(Pipe::PIPE_0);

	// provide sending feedback to user
	feedbackLastPacket.sendingFeedback = feedbackLastPacket.sendingFeedback;

	return true;
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
void
xpcc::Nrf24Data<Nrf24Phy, Clock>::update()
{
	// When sending state changed the communication has finished and we switch
	// back to Rx mode
	if(updateSendingState()) {
		// switch back to Rx mode
		Config::setMode(Config::Mode::Rx);
	}
}

// -----------------------------------------------------------------------------

template<typename Nrf24Phy, typename Clock>
bool
xpcc::Nrf24Data<Nrf24Phy, Clock>::isPacketAvailable()
{
	const uint8_t status = Phy::readStatus();
	const bool dataReady = status & static_cast<uint8_t>(Status::RX_DR);

	if(dataReady) {
		return true;
	}

	// there can still be packets in the Rx FIFO if queuing in nrf24 happened
	const uint8_t fifoStatus = Phy::readFifoStatus();
	const bool rxFifoEmpty = fifoStatus & static_cast<uint8_t>(FifoStatus::RX_EMPTY);

	if(not rxFifoEmpty) {
		return true;
	}

	return false;
}
