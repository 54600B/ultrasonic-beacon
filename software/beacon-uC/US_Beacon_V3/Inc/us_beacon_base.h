#ifndef US_BEACON_BASE_H_
#define US_BEACON_BASE_H_

#include "gpio.h"
#include "serial_debug.h"

#ifdef __cplusplus
	extern "C" {
#endif

extern Pin_t amplification1_PA0;
extern Pin_t amplification2_PA1;
extern Pin_t amplification3_PA2;
extern Pin_t amplification4_PA3;

extern Pin_t dac1_PA4;
extern Pin_t dac2_PA5;

extern Pin_t led5_PC10;
extern Pin_t led4_PC11;
extern Pin_t led3_PC12;
extern Pin_t led2_PD0;
extern Pin_t led1_PD1;
extern Pin_t led_slave_beacon_sync;

extern Pin_t ir_lighthouse_PC9;

extern Pin_t mic1_adc10_PC0;
extern Pin_t mic2_adc11_PC1;
extern Pin_t mic3_adc12_PC2;
extern Pin_t mic4_adc13_PC3;
extern Pin_t slave_sync_adc3_PA3;
extern Pin_t vbat_adc14_PC4;

extern Pin_t ir_sync_adc9_PB1;
extern Pin_t ir_sync_adc8_PB0;
extern Pin_t ir_sync_attenuator_front_PE13;
extern Pin_t ir_sync_attenuator_front_PE14;

extern Pin_t uart2_tx_PD5;
extern Pin_t uart2_rx_PD6;
extern Pin_t uart5_rx_PD2;

extern Pin_t uart3_tx_PD8;
extern Pin_t uart3_rx_PD9;

extern Pin_t batt_low_led_PE4;
extern Pin_t buzzer_PE5;

extern Pin_t led_ui_red;

extern Pin_t id_0_PE2;
extern Pin_t id_1_PE3;
extern Pin_t id_2_PE1;
extern Pin_t id_3_PE0;

extern Pin_t nrf_cs;
extern Pin_t nrf_sck;
extern Pin_t nrf_miso;
extern Pin_t nrf_mosi;
extern Pin_t nrf_int;
extern Pin_t nrf_ce;

extern Pin_t ui_led_red_batt_low_PA9;
extern Pin_t ui_led_yellow_ir_signal_ok_PA11;
extern Pin_t ui_led_green_synchronized_PA10;

enum hardware_version_t{
	V1 = 1,
	V2 = 2,
	V3 = 3,
	V3_NRF = 4
};

void setup_gpio(enum hardware_version_t hardware_version);

void setup_system_core_clock(void);

void setup_adc(void);

void setup_dac(void);

void setup_uart(void);

void setup_beacon_base_board(enum hardware_version_t hardware_version);

uint32_t get_board_id(void);
uint32_t is_coop_enabled(void);

#ifdef __cplusplus
	}
#endif

#endif

