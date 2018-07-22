/*
 * serial_rf.h
 *
 */

#ifndef SERIAL_RF_H_
#define SERIAL_RF_H_

#include <stdint.h>
#include <string.h> // contains memcpy
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
	extern "C" {
#endif

void serial_rf_init(void);
char serial_rf_read(void);
void serial_rf_write(char ch);

#ifdef __cplusplus
	}
#endif

#endif /* SERIAL_RF_H_ */
