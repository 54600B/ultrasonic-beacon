#ifndef GPIO_H_
#define GPIO_H_

#include <stdio.h>
#include "stm32f4xx.h"                  // Device header

#ifdef __cplusplus
	extern "C" {
#endif

typedef enum {
	INPUT = 0,
	OUTPUT = 1,
	ALTERNATE_FUNCTION = 2,
	ANALOG = 3,
} Pin_function_t;
	
typedef enum{
  NO_AF = 0,
	AF0_SYS = 0,
	SYS = 0,
	AF1_TIM1_TIM2 = 1,
	TIM1_AF1 = 1,
	TIM2_AF1 = 1,
	AF2_TIM3_TIM4_TIM5 = 2,
	TIM3_AF2 = 2,
	TIM4_AF2 = 2,
	TIM5_AF2 = 2,
	AF3_TIM8_TIM9_TIM9_TIM10_TIM11 = 3,
	TIM8_AF3 = 3,
	TIM9_AF3 = 3,
	TIM10_AF3 = 3,
	TIM11_AF3 = 3,
	AF4_I2C = 4,
	I2C1_AF4 = 4,
	I2C2_AF4 = 4,
	I2C3_AF4 = 4,
	AF5_SPI1_SPI2_I2S2 = 5,
	SPI1_AF5 = 5,
	SPI2_AF5 = 5,
	I2S2_AF5 = 5,
	I2S2_EXT_AF5 = 5,
	AF6_SPI3_I2S3 = 6,
	SPI3_AF6 = 6,
	I2S3_AF6 = 6,
	AF7_USART1_USART2_USART3 = 7,
	USART1_AF7 = 7,
	USART2_AF7 = 7,
	USART3_AF7 = 7,
	AF8_USART4_USART5_USART6 = 8,
	USART4_AF8 = 8,
	USART5_AF8 = 8,
	USART6_AF8 = 8,
	AF9_CAN1_CAN2_TIM12_TIM13_TIM14 = 9,
	CAN1_AF9 = 9,
	CAN2_AF9 = 9,
	TIM12_AF9 = 9,
	TIM13_AF9 = 9,
	AF10_OTG = 10,
	OTG_AF10 = 10,
	AF11_ETH = 11,
	ETH_AF11 = 11,
	AF12_FSMC_SDIO_OTG = 12,
	FSMC_AF12 = 12,
	SDIO_AF12 = 12,
	AF13_DCMI = 13,
	DCMI_AF13 = 13,
	AF14 = 14,
	AF15_EVENTOUT = 15,
	EVENTOUT_AF15 = 15,
} Alternate_function_t;

typedef enum{
	_2MHZ = 0,
	_25MHZ = 1,
	_50MHZ = 2,
	_100MHZ = 3,
} Output_speed_t;

typedef enum{
	OPEN = 0,
	NO_PP = 0,
	PULL_UP = 1,
	PULL_DOWN = 2,
} Push_pull_t;

typedef struct{
	GPIO_TypeDef* port;
	uint8_t nr;
}Pin_t;

void configure_gpio( Pin_t pin, Pin_function_t pin_function, Alternate_function_t alternate_function, Output_speed_t speed, Push_pull_t push_pull );

void set(Pin_t pin);

void reset(Pin_t pin);

void toggle(Pin_t pin);

void led_on(Pin_t pin);

void led_off(Pin_t pin);


uint32_t read(Pin_t pin);

#ifdef __cplusplus
	}
#endif

#endif

