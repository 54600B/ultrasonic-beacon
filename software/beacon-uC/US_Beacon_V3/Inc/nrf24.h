/*
 * nrf24.h
 *
 *  Created on: Jun 6, 2018
 *      Author: rca
 */

#ifndef NRF24_H_
#define NRF24_H_

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Packet
{
	uint16_t d[7];
	uint8_t q[7];
	uint8_t id;
	int16_t x;
	int16_t y;
	uint8_t flags;
} __attribute__((packed));

bool nrf24_init(uint8_t ownAddress);

void nrf24_set_data(const struct Packet* packet);

//bool nrf24_is_packet_available();

//bool nrf24_get_packet(struct Packet* packet, uint8_t* sourceId);

void nrf24_get_packets(struct Packet packets[8]);

void nrf24_sync();

#ifdef __cplusplus
}
#endif

#endif /* NRF24_H_ */
