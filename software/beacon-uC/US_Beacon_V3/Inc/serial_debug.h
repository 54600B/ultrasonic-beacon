/*
 * serial_debug.h
 *
 *  Created on: 27.05.2018
 *      Author: ole
 */

#ifndef SERIAL_DEBUG_H_
#define SERIAL_DEBUG_H_

#include <stdint.h>
#include <string.h> // contains memcpy
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
	extern "C" {
#endif

void serial_debug_init(void);
char serial_debug_read(void);
void serial_debug_write(char ch);

#ifdef __cplusplus
	}
#endif

#endif /* SERIAL_DEBUG_H_ */
