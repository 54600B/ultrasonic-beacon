/*
 * delay.h
 *
 */

#ifndef DELAY_H_
#define DELAY_H_

#define F_CPU_MHZ 144

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void /*__attribute__((section(".fastcode")))*/
_delay_us(uint16_t us);

void /*__attribute__((section(".fastcode")))*/
_delay_ms(uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* DELAY_H_ */
