/*
 * nrf24.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: rca
 */

#include "nrf24.h"
#include "nrf24_device.hpp"
#include "us_beacon_base.h"
#include "serial_rf.h"

#include <array>

using Phy = Nrf24Device::Phy;
using Config = Nrf24Device::Config;
using Data = Nrf24Device::Data;

static_assert(Data::Packet::getPayloadLength() >= sizeof(struct Packet), "NRF packet size");

namespace {
	Data::Packet inPacket;
	Data::Packet outPacket;

	uint8_t ownAddress = 0x0;

	volatile uint16_t microSlotCounter = 0xFFFF;

	constexpr Data::BaseAddress base_address = 0xe7e7e7e700;
	constexpr Data::Address broadcast_address = 0xe7;

	constexpr Data::Address masterAddress = 0x0;

	// 8 slots per frame, 6 microslots (1 ms) per slot
	// => frame duration: 48ms < 50ms (20 Hz)
	constexpr uint16_t numSlots = 8;
	constexpr uint16_t slotsPerMicroSlot = 6;

	const auto syncIo = uart2_rx_PD6;

	std::array<Packet, numSlots> data;
}

static void enableSlotTimer()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	// APB1 timer clock: 144 MHz, illegal!!!
	// prescaler 2, overflow after 36000 cycles => 1kHz
	TIM4->PSC = 3;
	TIM4->ARR = 36000 - 1;

	// enable interrupt
	TIM4->DIER |= TIM_DIER_UIE;

	// TIM_CR1_URS: do not generate interrupt on forced update event
	// allows to reset counter and prescaler by setting the UG bit in TIMx_EGR
	TIM4->CR1 |= TIM_CR1_CEN | TIM_CR1_URS;

	NVIC_SetPriority(TIM4_IRQn, 4);
	NVIC_EnableIRQ(TIM4_IRQn);
}

extern "C" {

bool nrf24_init(uint8_t address)
{
	xpcc::stm32::SpiMaster2::initialize<36000000, 9000000>();

	ownAddress = address;
	Phy::initialize();
	Data::initialize(base_address, address, broadcast_address);
	Config::setChannel(0xF0);
	Config::enableFeatureNoAck();
	Config::setSpeed(Config::Speed::MBps2);
	Config::setAutoRetransmitCount(Config::AutoRetransmitCount::Disable);
	Phy::dumpRegisters();

	serial_rf_init();

	enableSlotTimer();

	return true;
}

void nrf24_set_data(const struct Packet* packet)
{
	TIM4->DIER &= ~TIM_DIER_UIE;
	memcpy(outPacket.payload, packet, sizeof(struct Packet));
	TIM4->DIER |= TIM_DIER_UIE;
}

/*bool nrf24_is_packet_available()
{
	return packetReceived;
}

bool nrf24_get_packet(struct Packet* packet, uint8_t* sourceId)
{

	bool received = false;

	// disable timer interrupt
	TIM4->DIER &= ~TIM_DIER_UIE;

	if(packetReceived) {
		packetReceived = false;

		if(packet) {
			memcpy(packet, inPacket.payload, sizeof(struct Packet));
		}

		if(sourceId) {
			*sourceId = inPacket.getSource();
		}

		received = true;
	}

	// enable timer interrupt
	TIM4->DIER |= TIM_DIER_UIE;

	return received;
}*/

void nrf24_get_packets(struct Packet packets[8])
{
	// disable timer interrupt
	TIM4->DIER &= ~TIM_DIER_UIE;

	memcpy(packets, data.data(), sizeof(Packet) * data.size());

	// enable timer interrupt
	TIM4->DIER |= TIM_DIER_UIE;
}

void nrf24_sync()
{
	// disable timer interrupt
	TIM4->DIER &= ~TIM_DIER_UIE;

	// reset timer counter, correctly resets internal prescaler counter
	TIM4->EGR |= TIM_EGR_UG;

	microSlotCounter = 0;

	// enable timer interrupt
	TIM4->DIER |= TIM_DIER_UIE;
}

static void update()
{
	Data::update();
	if(Data::isPacketAvailable()) {



		Data::getPacket(inPacket);
		const uint8_t index = inPacket.getSource();
		if(index < numSlots) {
			memcpy(&data[index], inPacket.payload, sizeof(Packet));
		}

		if (index == 3)toggle(led5_PC10);
	}
}

void TIM4_IRQHandler()
{
	// clear interrupt flag
	TIM4->SR &= ~TIM_SR_UIF;


	update();

	if(microSlotCounter < ownAddress * slotsPerMicroSlot) {
		// wait for own slot
		++microSlotCounter;
	} else if (microSlotCounter == ownAddress * slotsPerMicroSlot) {
		// start of own slot, send data

		outPacket.setDestination(broadcast_address);
		Data::sendPacket(outPacket, false);

		++microSlotCounter;
	}

}

}
