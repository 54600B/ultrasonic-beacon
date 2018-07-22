/*
 * serial_debug.c
 *
 *  Created on: 27.05.2018
 *      Author: ole
 */

#import "serial_debug.h"

#define DBG_UART_BUFFER_LENGTH 4096
char debug_uart_buffer[DBG_UART_BUFFER_LENGTH];
volatile uint32_t nr_chars_stored = 0;

volatile char *next_input_to_buffer = &debug_uart_buffer[0];
volatile char *next_output_of_buffer = &debug_uart_buffer[0];

#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

void serial_debug_init(void) {
	RCC->APB1ENR |= (1ul << 18); /* Enable USART#3 clock             */
	USART3->BRR = __USART_BRR(72000000ul, 115200ul); /* 115200 baud @ 72MHz   */
	USART3->CR3 = 0x0000; /* no flow control                 */
	USART3->CR2 = 0x0000; /* 1 stop bit                      */
	USART3->CR1 = ((1ul << 2) | /* enable RX                       */
	(1ul << 3) | /* enable TX                       */
	(1ul << 13)) | /* enable USART                    */
	0; //USART_CR1_TXEIE ;
}

char serial_debug_read(void) {
	if (USART3->SR & 0x0020)
	    return (USART3->DR);
	return -1;
}

void serial_debug_write(char ch) {

	while (nr_chars_stored >= DBG_UART_BUFFER_LENGTH - 1) {
	};
	*next_input_to_buffer = ch;

	NVIC_DisableIRQ(USART3_IRQn);
	nr_chars_stored += 1;
	next_input_to_buffer += 1;
	NVIC_EnableIRQ(USART3_IRQn);

	if (next_input_to_buffer >= &debug_uart_buffer[DBG_UART_BUFFER_LENGTH]) {
		next_input_to_buffer -= DBG_UART_BUFFER_LENGTH;
	}

	//if (nr_chars_stored <= 1)
	{
		USART3->CR1 |= USART_CR1_TXEIE;
	}
}

void USART3_IRQHandler(void) {
	if (next_output_of_buffer != next_input_to_buffer) {
		USART3->DR = *next_output_of_buffer & 0x00ff;
		next_output_of_buffer += 1;
		if (next_output_of_buffer
				>= &debug_uart_buffer[DBG_UART_BUFFER_LENGTH]) {
			next_output_of_buffer -= DBG_UART_BUFFER_LENGTH;
		}
		nr_chars_stored -= 1;

	} else {
		USART3->CR1 &= ~USART_CR1_TXEIE;
	}
}
