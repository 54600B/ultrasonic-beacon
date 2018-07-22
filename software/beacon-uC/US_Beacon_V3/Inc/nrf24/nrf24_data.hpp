// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC__NRF24_DATA_HPP
#define XPCC__NRF24_DATA_HPP

#include <stdint.h>
#include "timeout.hpp"
#include "utils.hpp"

#include "nrf24_phy.hpp"
#include "nrf24_config.hpp"
#include "nrf24_definitions.hpp"

namespace xpcc
{

/* Pipe layout:
 *
 * ===== 0 ===== : Used to receive ACKs when communicating directly with another node
 * ===== 1 ===== : Broadcast pipe, will determine upper 4 bytes of address of pipes 2 - 5
 * ===== 2 ===== : Own address
 * ===== 3 ===== : \
 * ===== 4 ===== :  | Separate connections to neighbouring nodes
 * ===== 5 ===== : /
 *
 * NOTE: connections on pipe 3-5 not yet implemented
 */

/// @ingroup	nrf24
/// @author		Daniel Krebs
template<typename Nrf24Phy, typename Clock = xpcc::Clock>
class Nrf24Data : xpcc::Nrf24Register
{
public:

	/// @{
	/// @ingroup	nrf24
	typedef uint64_t		BaseAddress;
	typedef uint8_t			Address;
	typedef xpcc::Timestamp	Timestamp;
	typedef Clock			ClockLower;
	/// @}

	/// @ingroup	nrf24
	enum class SendingFeedback
	{
		Busy,           ///< Waiting for ACK
		FinishedAck,    ///< Packet sent and ACK received
		FinishedNack,   ///< Packet sent but no ACK received in time
		DontKnow,       ///< Packet was sent without requesting ACK
		Failed,         ///< Packet could not be sent
		Undefined       ///< Initial state before a packet has been handled
	};

	static const char*
	toStr(SendingFeedback feedback) {
		switch(feedback) {
		case SendingFeedback::Busy:			return "Busy";
		case SendingFeedback::FinishedAck:	return "FinishedAck";
		case SendingFeedback::FinishedNack:	return "FinishedNack";
		case SendingFeedback::DontKnow:		return "DontKnow";
		case SendingFeedback::Failed:		return "Failed";
		default:
		case SendingFeedback::Undefined:	return "Undefined"; }
	}

	struct Feedback {
		// TX feedback
		/// only valid for sent packets
		SendingFeedback	sendingFeedback;

		// RX feedback
		/// only valid for received packets
		Timestamp timestamp;
		/// whether the corresponding interrupt has been missed or not
		bool timestampTrusted;
	};

	/// Header of Frame
	struct xpcc_packed Header
	{
		Address	src;
		Address	dest;
	};

	/// Data that will be sent over the air
	class xpcc_packed Packet
	{
		friend Nrf24Data;

	private:
		Header	header;
	public:
		uint8_t	payload[Nrf24Phy::getMaxPayload() - sizeof (Header)];

	public:
		static constexpr uint8_t
		getPayloadLength()
		{ return sizeof(payload); }

		xpcc_always_inline void
		setDestination(Address dest)
		{ header.dest = dest; }

		xpcc_always_inline Address
		getDestination() const
		{ return header.dest; }

		xpcc_always_inline Address
		getSource() const
		{ return header.src; }

	private:
		xpcc_always_inline void
		setSource(Address src)
		{ header.src = src; }
	};
	/// @}

	static constexpr uint8_t
	getFrameOverhead()
	{ return sizeof (Header); }

	static xpcc_always_inline uint8_t
	getDynamicPayloadLength()
	{ return Phy::getPayloadLength() - getFrameOverhead(); }

public:

	/* typedef config and physical layer for simplicity */
	typedef xpcc::Nrf24Config<Nrf24Phy> Config;
	typedef Nrf24Phy Phy;

	static void
	initialize(BaseAddress base_address,
	           Address own_address,
	           Address broadcast_address = 0xFF);

public:

	static bool
	sendPacket(Packet& packet, bool retransmit = true);

	static bool
	getPacket(Packet& packet);

	static xpcc_always_inline bool
	isReadyToSend()
	{ return feedbackLastPacket.sendingFeedback != SendingFeedback::Busy; }

	static bool
	isPacketAvailable();

	/// Feedback of last transaction, i.e. calls to `sendPacket()` and
	/// `getPacket()`.
	static xpcc_always_inline const Feedback&
	getFeedback()
	{ return feedbackLastPacket; }

	/// Call this function in your main loop
	static void
	update();

	/// Call this function from within IRQ pin interrupt
	/// You have to setup the IRQ pin yourself!
	static void
	interruptHandler();

	static xpcc_always_inline Address
	getAddress()
	{ return ownAddress; }

	/// Set own address
	static void
	setAddress(Address address);

	static xpcc_always_inline void
	setBroadcastAddress(Address address);

	static xpcc_always_inline Address
	getBroadcastAddress()
	{ return broadcastAddress; }

private:

	static inline uint64_t
	assembleAddress(Address address) {
		return static_cast<uint64_t>((uint64_t)baseAddress | (uint64_t)address);
	}

	static bool
	updateSendingState();

private:

	/**  Base address of the network
	 *
	 *   The first 3 bytes will be truncated, so the address is actually 5 bytes
	 *   long. The last byte will be used to address individual modules or
	 *   connections between them respectively. Use different base address for
	 *   separate networks, although it may impact performance seriously to run
	 *   overlapping networks.
	 *
	 *   Format:
	 *
	 *   | dont'care | base address | address |
	 *   | ---------------------------------- |
	 *   |   3 byte  |    4 byte    |  1 byte |
	 */
	static BaseAddress baseAddress;

	/// Packet with this destination will be sent without acknowledgements and
	/// delivered to all stations in vicinity.
	static Address broadcastAddress;

	/// Network address of this station
	static Address ownAddress;

	/// Internal state of feedback. Is double-buffered by `feedbackLastPacket`
	static Feedback feedbackCurrentPacket;

	/// Feedback for the last sent or received packet (refers to calls to
	/// `getPacket()` and `sendPacket()`).
	static Feedback feedbackLastPacket;

	/// Workaround when nrf24 module doesn't indicate finished transmission. The
	/// reason is still unknown, but without this timeout, the internal state
	/// machine of the driver would get stuck.
	static xpcc::Timeout sendingInterruptTimeout;

	/// How long to wait for a finished transmission before aborting (see
	/// `sendingInterruptTimeout` for more details).
	/// Depending on the amount of retransmissions and the auto retransmit delay
	/// this timeout can become quite large. Worst case is 4000us * 15 = 60ms
	/// plus some processing times. We consider the worst case here.
	static constexpr int sendingInterruptTimeoutMs = 65;
};

}	// namespace xpcc

#include "nrf24_data_impl.hpp"

#endif /* XPCC__NRF24_DATA_HPP */
