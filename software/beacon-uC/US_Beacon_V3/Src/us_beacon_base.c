#include "us_beacon_base.h"
#include "gpio.h"

Pin_t amplification1_PA0 = { GPIOA, 0 }; // amplification set pin for microphone 1
Pin_t amplification2_PA1 = { GPIOA, 1 }; // amplification set pin for microphone 2
Pin_t amplification3_PA2 = { GPIOA, 2 }; // amplification set pin for microphone 3
Pin_t amplification4_PA3 = { GPIOA, 3 }; // amplification set pin for microphone 4

Pin_t dac1_PA4 = { GPIOA, 4 };   // dac1 output
Pin_t dac2_PA5 = { GPIOA, 5 };   // dac1 output

Pin_t led5_PC10 = { GPIOC, 10 };   // led5 output (ir signal quality indicator)
Pin_t led4_PC11 = { GPIOC, 11 };   // led4 output (processing time)
Pin_t led3_PC12 = { GPIOC, 12 };   // led3 output (oszi trigger output)
Pin_t led2_PD0 = { GPIOD, 0 }; // led4 output (ir modulation output for signal beacon->robot)
Pin_t led1_PD1 = { GPIOD, 1 };
Pin_t led_slave_beacon_sync = { GPIOA, 8 };
//Pin_t ir_sync_to_slave   = {GPIOA, 12};   //  (ir modulation output for signal master beacon->slave_beacon)

Pin_t mic1_adc10_PC0 = { GPIOC, 0 };   // microphone 1 adc input channel 10
Pin_t mic2_adc11_PC1 = { GPIOC, 1 };   // microphone 2 adc input channel 11
Pin_t mic3_adc12_PC2 = { GPIOC, 2 };   // microphone 3 adc input channel 12
Pin_t mic4_adc13_PC3 = { GPIOC, 3 };   // microphone 4 adc input channel 13
Pin_t slave_sync_adc3_PA3 = { GPIOA, 3 }; // synch input for fixed slave beacons
Pin_t vbat_adc14_PC4 = { GPIOC, 4 };   // battery monitor, input channel 14

Pin_t ir_sync_front_adc9_PB1 = { GPIOB, 1 }; // infrared time synchronization front, input channel 9
Pin_t ir_sync_back_adc8_PB0 = { GPIOB, 0 }; // infrared time synchronization back, input channel 8

Pin_t ir_sync_attenuator_front_PE13 = { GPIOE, 13 };
Pin_t ir_sync_attenuator_front_PE14 = { GPIOE, 14 };

Pin_t ir_lighthouse_PC9 = { GPIOC, 9 }; // enables ir leds for the backup-collision avoidance system, ir lighthouse on every opponent beacon

Pin_t uart3_tx_PD8 = { GPIOD, 8 };    // uart3 tx (debug uart)
Pin_t uart3_rx_PD9 = { GPIOD, 9 };    // uart3 rx (debug uart)

Pin_t uart2_tx_PD5 = { GPIOD, 5 };    // uart2 tx (rf uart)
Pin_t uart2_rx_PD6 = { GPIOD, 6 };    // uart2 rx (rf uart)
Pin_t uart5_rx_PD2 = { GPIOD, 2 };

Pin_t batt_low_led_PE4 = { GPIOE, 4 };    // battery low power led enable
Pin_t buzzer_PE5 = { GPIOE, 5 };    // gold_code_seed

Pin_t led_ui_red = { GPIOA, 9 };

Pin_t id_0_PE2 = { GPIOE, 2 };
Pin_t id_1_PE3 = { GPIOE, 3 };
Pin_t id_2_PE1 = { GPIOE, 1 };
Pin_t id_3_PE0 = { GPIOE, 0 };

Pin_t nrf_cs = {GPIOB, 12};
Pin_t nrf_sck = {GPIOB, 13};
Pin_t nrf_miso = {GPIOB, 14};
Pin_t nrf_mosi = {GPIOB, 15};

Pin_t nrf_int = {GPIOB,  10};
Pin_t nrf_ce  = {GPIOB,  11};

Pin_t ui_led_red_batt_low_PA9         = {GPIOA, 9};
Pin_t ui_led_yellow_ir_signal_ok_PA11 = {GPIOA, 11};
Pin_t ui_led_green_synchronized_PA10  = {GPIOA, 10};

void setup_gpio(enum hardware_version_t hardware_version) {
	configure_gpio(amplification1_PA0, OUTPUT, NO_AF, _25MHZ, NO_PP);
	configure_gpio(amplification2_PA1, OUTPUT, NO_AF, _25MHZ, NO_PP);
	configure_gpio(amplification3_PA2, OUTPUT, NO_AF, _25MHZ, NO_PP);
	configure_gpio(amplification4_PA3, OUTPUT, NO_AF, _25MHZ, NO_PP);

	if (hardware_version == V1) {
		/* remap pins of V2 to correspinding pins on V1 pcb hardware, the variable names do no longer represent the port and number! */
		led5_PC10.port = GPIOA;
		led5_PC10.nr = 8;
		led4_PC11.port = GPIOA;
		led4_PC11.nr = 9;
		led3_PC12.port = GPIOA;
		led3_PC12.nr = 10;
		nrf_int.port = GPIOA;
		nrf_int.nr = 11;
		nrf_ce.port = GPIOA;
		nrf_ce.nr = 12;
	}

	if ((hardware_version == V2) || (hardware_version == V3)) {
		nrf_int.port = GPIOD;
		nrf_int.nr = 0;
		nrf_ce.port = GPIOD;
		nrf_ce.nr = 1;
	}

	configure_gpio(led5_PC10, OUTPUT, NO_AF, _2MHZ, NO_PP);
	configure_gpio(led4_PC11, OUTPUT, NO_AF, _2MHZ, NO_PP);
	configure_gpio(led3_PC12, OUTPUT, NO_AF, _2MHZ, NO_PP);
	configure_gpio(led_ui_red, OUTPUT, NO_AF, _2MHZ, NO_PP);

	configure_gpio(led_slave_beacon_sync, OUTPUT, NO_AF, _2MHZ, NO_PP);

	configure_gpio(ir_lighthouse_PC9, OUTPUT, NO_AF, _2MHZ,
			NO_PP);

	configure_gpio(batt_low_led_PE4, OUTPUT, NO_AF, _2MHZ, NO_PP);
	configure_gpio(buzzer_PE5, OUTPUT, NO_AF, _2MHZ, NO_PP);

	configure_gpio(id_3_PE0, INPUT, NO_AF, _2MHZ, PULL_DOWN);
	configure_gpio(id_2_PE1, INPUT, NO_AF, _2MHZ, PULL_DOWN);
	configure_gpio(id_1_PE3, INPUT, NO_AF, _2MHZ, PULL_DOWN);
	configure_gpio(id_0_PE2, INPUT, NO_AF, _2MHZ, PULL_DOWN);

	configure_gpio(nrf_cs, OUTPUT, NO_AF, _25MHZ, NO_PP);
	configure_gpio(nrf_sck, ALTERNATE_FUNCTION, SPI2_AF5, _25MHZ, NO_PP);
	configure_gpio(nrf_miso, ALTERNATE_FUNCTION, SPI2_AF5, _25MHZ, NO_PP);
	configure_gpio(nrf_mosi, ALTERNATE_FUNCTION, SPI2_AF5, _25MHZ, NO_PP);
	configure_gpio(nrf_int, INPUT, NO_AF, _25MHZ, NO_PP);
	configure_gpio(nrf_ce, OUTPUT, NO_AF, _25MHZ, NO_PP);

	configure_gpio(ui_led_red_batt_low_PA9, OUTPUT, NO_AF, _2MHZ, NO_PP);
	configure_gpio(ui_led_yellow_ir_signal_ok_PA11, OUTPUT, NO_AF, _2MHZ,
			NO_PP);
	configure_gpio(ui_led_green_synchronized_PA10, OUTPUT, NO_AF, _2MHZ,
			NO_PP);

	configure_gpio(ir_sync_attenuator_front_PE13, OUTPUT, NO_AF, _2MHZ, NO_PP);
	configure_gpio(ir_sync_attenuator_front_PE14, OUTPUT, NO_AF, _2MHZ, NO_PP);
}

/*----------------------------------------------------------------------------
 * SystemCoreClockConfigure: configure SystemCoreClock using
 8 MHz HSE clock
 pll clock input = 1MHz
 AHB clock = 144MHz
 APB1 clock = 72MHz
 APB2 clock = 72MHz

 *----------------------------------------------------------------------------*/
void setup_system_core_clock() {
	RCC->CR |= RCC_CR_HSEON;                     				// Enable HSE
	while ((RCC->CR & RCC_CR_HSERDY) == 0)
		;                  // Wait for HSI Ready

	RCC->CFGR = RCC_CFGR_SW_HSE;                          // HSE is system clock
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSE)
		;  // Wait for HSE used as system clock

	FLASH->ACR = FLASH_ACR_PRFTEN;                     // Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_ICEN;                    // Instruction cache enable
	FLASH->ACR |= FLASH_ACR_DCEN;                           // Data cache enable
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                   // Flash 5 wait state

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // HCLK = SYSCLK (1,2,4,..512) (AHB prescaler)
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;              // APB1 = HCLK/4 (1,2,4,8,16)
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;              // APB2 = HCLK/2 (1,2,4,8,16)

	RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

	// PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P
	RCC->PLLCFGR = (8ul |                // PLL_M =  8 (2..63)
			(288ul << 6) |                // PLL_N = 384 (192..432)
			(0ul << 16) |       // PLL_P =   8 (0=DIV2, 1=DIV4, 2=DIV6, 3=DIV8)
			(RCC_PLLCFGR_PLLSRC_HSE) |                // PLL_SRC = HSE
			(6ul << 24));              // PLL_Q =   8 (2..15)

	RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
	while ((RCC->CR & RCC_CR_PLLRDY) == 0)
		__NOP();           // Wait till PLL is ready

	RCC->CFGR &= ~RCC_CFGR_SW;              // Select PLL as system clock source
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		;  // Wait till PLL is system clock src

	SystemCoreClockUpdate();

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(/*HAL_RCC_GetHCLKFreq() / 1000*/ 144 * 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void setup_adc() {
	configure_gpio(mic1_adc10_PC0, ANALOG, NO_AF, _25MHZ, NO_PP);
	configure_gpio(mic2_adc11_PC1, ANALOG, NO_AF, _25MHZ, NO_PP);
	configure_gpio(mic3_adc12_PC2, ANALOG, NO_AF, _25MHZ, NO_PP);
	configure_gpio(mic4_adc13_PC3, ANALOG, NO_AF, _25MHZ, NO_PP);
	configure_gpio(vbat_adc14_PC4, ANALOG, NO_AF, _25MHZ, NO_PP);
	//configure_gpio( slave_sync_adc3_PA3 , ANALOG, NO_AF, _25MHZ, NO_PP);

	configure_gpio(ir_sync_front_adc9_PB1, ANALOG, NO_AF, _25MHZ, NO_PP);
	configure_gpio(ir_sync_back_adc8_PB0, ANALOG, NO_AF, _25MHZ, NO_PP);

	// ADC1: ir front at ch 9 ir back at ch 8 ch14 
	// ADC2: mic1 at ch10 and mic2 at ch11
	// ADC3: mic3 at ch12 and mic4 at ch13

	RCC->APB2ENR |=
			RCC_APB2ENR_ADC1EN + RCC_APB2ENR_ADC2EN + RCC_APB2ENR_ADC3EN;
	ADC->CCR = 0ul * ADC_CCR_ADCPRE_0; // proescaler 2, ADC clock 36MHz from 72MHz APB2
	ADC1->CR1 = 0;
	ADC2->CR1 = 0;
	ADC3->CR1 = 0;
	ADC1->CR2 = ADC_CR2_ADON; // set ADC_CR2_SWSTART for manual conversion
	ADC2->CR2 = ADC_CR2_ADON;
	ADC3->CR2 = ADC_CR2_ADON;
	ADC1->SMPR1 = ADC_SMPR1_SMP14_0 * 3ul; //=56 clock cycles sample time = 1.6us
	ADC1->SMPR2 = ADC_SMPR2_SMP8_0 * 3ul + ADC_SMPR2_SMP9_0 * 3ul;
	ADC2->SMPR1 = ADC_SMPR1_SMP10_0 * 3ul + ADC_SMPR1_SMP11_0 * 3ul;
	ADC3->SMPR1 = ADC_SMPR1_SMP12_0 * 3ul + ADC_SMPR1_SMP13_0 * 3ul;
	ADC1->SQR1 = 0; // one conversion
	ADC2->SQR1 = 0;
	ADC3->SQR1 = 0;
	ADC1->SQR3 = ADC_SQR3_SQ1_0 * 9ul;
	ADC2->SQR3 = ADC_SQR3_SQ1_0 * 10ul;
	ADC3->SQR3 = ADC_SQR3_SQ1_0 * 12ul;
}

void setup_dac() {
	configure_gpio(dac1_PA4, ANALOG, NO_AF, _25MHZ, NO_PP);
	configure_gpio(dac2_PA5, ANALOG, NO_AF, _25MHZ, NO_PP);

	//printf("Configure DAC");

	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR = DAC_CR_EN1 + DAC_CR_EN2;

	//printf(" done\n");
}

void setup_uart() {
	configure_gpio(uart3_tx_PD8, ALTERNATE_FUNCTION, USART3_AF7, _25MHZ, NO_PP);
	configure_gpio(uart3_rx_PD9, ALTERNATE_FUNCTION, USART3_AF7, _25MHZ, NO_PP);

	configure_gpio(uart2_tx_PD5, ALTERNATE_FUNCTION, USART2_AF7, _25MHZ, NO_PP);
	//configure_gpio( uart2_rx_PD6, ALTERNATE_FUNCTION, USART2_AF7, _25MHZ, NO_PP);
	configure_gpio(uart5_rx_PD2, ALTERNATE_FUNCTION, USART5_AF8, _25MHZ, NO_PP);

	//SER_Initialize();
	serial_debug_init();
}

void setup_beacon_base_board(enum hardware_version_t hardware_version) {

	setup_system_core_clock();

	setup_uart();

	setup_gpio(hardware_version);
	reset(led1_PD1);
	reset(led2_PD0);
	reset(led3_PC12);
	reset(led4_PC11);
	reset(led5_PC10);

	set(led2_PD0);

	setup_adc();
	set(led3_PC12);

	setup_dac();
	set(led4_PC11);

	set(led_ui_red);
}

uint32_t get_board_id() {
	return read(id_0_PE2) + read(id_1_PE3) * 2ul + read(id_2_PE1) * 4ul;
}

uint32_t is_coop_enabled() {
	return read(id_3_PE0);
}
