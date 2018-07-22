/*
 * hardware_version.h
 *
 *  Created on: Jun 6, 2018
 *      Author: rca
 */

#ifndef HARDWARE_VERSION_H_
#define HARDWARE_VERSION_H_

#include "us_beacon_base.h"

#ifdef __cplusplus
	constexpr
#else
	const
#endif
enum hardware_version_t pcb_hardware_version = V3;

#endif /* HARDWARE_VERSION_H_ */
