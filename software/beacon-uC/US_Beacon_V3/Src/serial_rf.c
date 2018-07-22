/*
 * serial_rf.c
 *
 *  Created on: Jun 10, 2018
 *      Author: rca
 */

#include "serial_rf.h"

#define RF_UART_BUFFER_LENGTH 512
static char debug_uart_buffer[RF_UART_BUFFER_LENGTH];
static volatile uint32_t nr_chars_stored = 0;

static volatile char *next_input_to_buffer = &debug_uart_buffer[0];
static volatile char *next_output_of_buffer = &debug_uart_buffer[0];

#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

void serial_rf_init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; /* Enable USART#2 clock             */
	USART2->BRR = __USART_BRR(72000000ul, 115200ul); /* 115200 baud @ 72MHz   */
	USART2->CR3 = 0x0000; /* no flow control                 */
	USART2->CR2 = 0x0000; /* 1 stop bit                      */
	USART2->CR1 = ((1ul << 2) | /* enable RX                       */
	(1ul << 3) | /* enable TX                       */
	(1ul << 13)) | /* enable USART                    */
	0; //USART_CR1_TXEIE ;
}

char serial_rf_read(void) {
	if (USART2->SR & 0x0020)
	    return (USART2->DR);
	return -1;
}

void serial_rf_write(char ch) {
	while (nr_chars_stored >= RF_UART_BUFFER_LENGTH - 1) {
	};
	*next_input_to_buffer = ch;
	nr_chars_stored += 1;
	next_input_to_buffer += 1;
	if (next_input_to_buffer >= &debug_uart_buffer[RF_UART_BUFFER_LENGTH]) {
		next_input_to_buffer -= RF_UART_BUFFER_LENGTH;
	}

	//if (nr_chars_stored <= 1)
	{
		USART2->CR1 |= USART_CR1_TXEIE;
	}
}

void USART2_IRQHandler(void) {
	if (next_output_of_buffer != next_input_to_buffer) {
		USART2->DR = *next_output_of_buffer & 0x00ff;
		next_output_of_buffer += 1;
		if (next_output_of_buffer
				>= &debug_uart_buffer[RF_UART_BUFFER_LENGTH]) {
			next_output_of_buffer -= RF_UART_BUFFER_LENGTH;
		}
		nr_chars_stored -= 1;

	} else {
		USART2->CR1 &= ~USART_CR1_TXEIE;
	}
}




