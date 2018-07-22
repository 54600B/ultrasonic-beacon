#include "gpio.h"


void configure_gpio( Pin_t pin, Pin_function_t pin_function, Alternate_function_t alternate_function, Output_speed_t speed, Push_pull_t push_pull )
{
	// Enable Portx AHB1 peripheral clock
	uint32_t port_address = (uint32_t)(pin.port);
	switch (port_address){
		case (uint32_t)GPIOA:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
						break;
		case (uint32_t)GPIOB:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
						break;
		case (uint32_t)GPIOC:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
						break;
		case (uint32_t)GPIOD:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
						break;
		case (uint32_t)GPIOE:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
						break;
		case (uint32_t)GPIOF:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
						break;
		case (uint32_t)GPIOG:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
						break;
		case (uint32_t)GPIOH:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
						break;
		case (uint32_t)GPIOI:
						RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
						break;
		default:
						break;
	}	
	
	// set pin function (input, output, alternate function or analog)
	pin.port->MODER &= ~(GPIO_MODER_MODE0_Msk << (2*pin.nr));
	pin.port->MODER |= pin_function << (2*pin.nr);
	
	// set alternate function
	if (pin_function == ALTERNATE_FUNCTION)
	{
		if (pin.nr <= 7)
		{
			pin.port->AFR[0] &= ~(GPIO_AFRL_AFSEL0_Msk << (4*pin.nr));
			pin.port->AFR[0] |= alternate_function << (4*pin.nr);
		}
		else if (pin.nr <= 15)
		{
			pin.port->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk << (4*(pin.nr-8)));
			pin.port->AFR[1] |= alternate_function << (4*(pin.nr-8));
		}
	}
	
	// set pin speed
	pin.port->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk << (2*pin.nr));
	pin.port->OSPEEDR |= speed << (2*pin.nr);
	
	// set pull up or pull down resistors
	pin.port->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk << (2*pin.nr));
	pin.port->PUPDR |= push_pull << (2*pin.nr);
}

void set(Pin_t pin)
{
	pin.port->BSRR = 1ul << pin.nr;
}

void reset(Pin_t pin)
{
	pin.port->BSRR = 1ul << (pin.nr+16);
}

void toggle(Pin_t pin)
{
	if(pin.port->ODR & (1u << pin.nr))
		reset(pin);
	else
		set(pin);
}

void led_on(Pin_t pin)
{
	reset(pin);
}

void led_off(Pin_t pin)
{
	set(pin);
}

uint32_t read(Pin_t pin)
{
  return (pin.port->IDR & (1ul << pin.nr)) != 0;
}
