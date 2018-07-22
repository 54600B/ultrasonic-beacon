/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "us_beacon_base.h"

#define ARM_MATH_CM4 1
#include "arm_math.h"

#include "constants.h"
#include "location_estimation.h"
#include "hardware_version.h"
#include "delay.h"

#include "nrf24.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const float pi = 3.14159265359;
#define NR_OF_MICROPHONES 4
#define NR_OF_IR_CHANNELS 1
#define NR_OF_SIGNALS NR_OF_MICROPHONES + NR_OF_IR_CHANNELS
#define NR_OF_SAMPLES 2048
#define SAMPLERATE 160000.0f
#define SENDER_OVERSAMPLING 2 // DAC outputs values with the rate SAMPLERATE*SENDER_OVERSAMPLING
#define NR_OF_GOLD_CODES 3 // number of stored gold codes, only one is currently used

#define NR_OF_US_SENDERS  7
const float32_t US_SPAN = 3000;
const int32_t RF_SYNCHRONIZATION_OFFSET = 10;

#define AVG_MIC_DISTANCE_TO_BOARD_CENTER_MM 45

#define beacon_offset 50 // distance of speaker to the inside of the playground border
float beaconX[NR_OF_US_SENDERS] = { 0, 934, -934 }; // X-position of the fixed beacons in mm, coordinate origin is the center of the playing area with x-axis along the short border and y axis along the long border
float beaconY[NR_OF_US_SENDERS] = { 1562, -1562, -1562 }; // Y-position of the beacons in mm

float distance_to_beacon[NR_OF_US_SENDERS] = { 0, 0, 0, 0, 0, 0, 0 };

const float32_t BIN_WIDTH = ((float32_t) SAMPLERATE)
		/ ((float32_t) NR_OF_SAMPLES);

#define ECHO_DECAY_COUNT_SYNCHRONIZED 4 // nr of sending slots (=1 for no echo pause; =2 for one echo pause etc)

const uint32_t IR_QUALITY_STEP_SIZE = 1; // IR Quality estimator; reduction of the number of samples to evaluate while looking for the biggest IR silde lobe
const float32_t IR_QUALITY_GOOD_THRESHOLD = 2.0f; // if the quality of the ir correlation is greater than IR_QUALITY_GOOD_THRESHOLD then the current ir sample set is used to synchronize the local clock to the master beacon and the green LED lights up
float32_t ir_correlation_max = 0.0; //displayed in GUI

float32_t dist_correlation_3x128[3 * 128];

int16_t samples_ADC_A[NR_OF_SIGNALS][NR_OF_SAMPLES];
int16_t samples_ADC_B[NR_OF_SIGNALS][NR_OF_SAMPLES];
int16_t (*recording_samples_ADC_buf)[NR_OF_SAMPLES] = samples_ADC_A;
int16_t (*evaluating_samples_ADC_buf)[NR_OF_SAMPLES] = samples_ADC_B;

enum SampleBufferState {
	READY_FOR_EVALUATION, READY_FOR_SWAP
};
volatile enum SampleBufferState sampleBufferState = READY_FOR_SWAP;

const uint32_t DAC_MAX_AMPLITUDE = 3600; // upper limit of DAC is determined by the amplification of the audio power amplifier and the maximum audio amplifier output swing
const uint32_t DAC_MIN = 248; // lower limit of DAC is determined by the DACs unity gain amplifier output stage

enum Operation {
	OP_CONTROL_WORD = 0,
	OP_DAC_CHAN1 = 1,
	OP_DAC_CHAN2 = 2,
	OP_PRINT_WAVEFORM = 3,
};

enum ControlWord {
	CW_NRF_PACKET_CONTENT_DEBUG_OUTPUT_ENABLE = 1,
	CW_NRF_PACKET_CONTENT_DEBUG_OUTPUT_DISABLE = 2,
	CW_MICROPHONE_POSITION_DEBUG_OUTPUT_ENABLE = 3,
	CW_MICROPHONE_POSITION_DEBUG_OUTPUT_DISABLE = 4,
	CW_US_RANGING_MATRIX_DEBUG_OUTPUT_ENABLE = 5,
	CW_US_RANGING_MATRIX_DEBUG_OUTPUT_DISABLE = 6,
	CW_DEBUG_WAVEFORM_ENABLE = 45,
	CW_DEBUG_WAVEFORM_DISABLE = 46
};

enum NrfPacketContentDebugOutputState {
	NRF_PACKET_CONTENT_DEBUG_OUTPUT_DISABLED = 0,
	NRF_PACKET_CONTENT_DEBUG_OUTPUT_ENABLED  = 1
};
volatile enum NrfPacketContentDebugOutputState nrf_packet_content_debug_output = NRF_PACKET_CONTENT_DEBUG_OUTPUT_DISABLED;

enum MicrophonePositionDebugState {
	MICROPHONE_POSITION_DEBUG_OUTPUT_DISABLED = 0,
	MICROPHONE_POSITION_DEBUG_OUTPUT_ENABLED = 1
};
volatile enum MicrophonePositionDebugState microphone_position_debug_state = MICROPHONE_POSITION_DEBUG_OUTPUT_DISABLED;

enum UsRangingMatrixDebugOutputState {
	US_RANGING_MATRIX_DEBUG_OUTPUT_DISABLED = 0,
	US_RANGING_MATRIX_DEBUG_OUTPUT_ENABLED = 1
};
volatile enum UsRangingMatrixDebugOutputState us_ranging_matrix_debug_output_state = US_RANGING_MATRIX_DEBUG_OUTPUT_DISABLED;

enum WFDebugInteface {
	WF_DEBUG_OUTPUT_DISABLED = 0,
	WF_DEBUG_OUTPUT_ENABLED  = 1
};
volatile enum WFDebugInteface wf_debug_interface = WF_DEBUG_OUTPUT_DISABLED;

enum Waveform_Id {
	WF_NONE = 0,
	WF_MIC1 = 1,
	WF_MIC2 = 2,
	WF_MIC3 = 3,
	WF_MIC4 = 4,
	WF_MIC1_SPECTRUM = 5,
	WF_MIC2_SPECTRUM = 6,
	WF_MIC3_SPECTRUM = 7,
	WF_MIC4_SPECTRUM = 8,
	WF_Signal_Strengths = 9,
	WF_Master_Correlation = 10,
	WF_Slave1_Correlation = 11,
	WF_Slave2_Correlation = 12,
	WF_Angle_Correlation = 13,
	WF_IR = 14,
	WF_IR_Spectrum = 15,
	WF_IR_Correlation = 16,
	WF_IR_Correlation_Lower = 17,
	WF_IR_Correlation_Upper = 18,
	WF_NONE2 = 19,
	WF_MIC1_MASTER_CORRELATION_128 = 20,
	WF_MIC2_MASTER_CORRELATION_128 = 21,
	WF_MIC3_MASTER_CORRELATION_128 = 22,
	WF_MIC4_MASTER_CORRELATION_128 = 23,
	WF_MIC1_SLAVE1_CORRELATION_128 = 24,
	WF_MIC2_SLAVE1_CORRELATION_128 = 25,
	WF_MIC3_SLAVE1_CORRELATION_128 = 26,
	WF_MIC4_SLAVE1_CORRELATION_128 = 27,
	WF_MIC1_SLAVE2_CORRELATION_128 = 28,
	WF_MIC2_SLAVE2_CORRELATION_128 = 29,
	WF_MIC3_SLAVE2_CORRELATION_128 = 30,
	WF_MIC4_SLAVE2_CORRELATION_128 = 31,
	WF_MIC1_COOP3_CORRELATION_128 = 32,
	WF_MIC2_COOP3_CORRELATION_128 = 33,
	WF_MIC3_COOP3_CORRELATION_128 = 34,
	WF_MIC4_COOP3_CORRELATION_128 = 35,
	WF_MIC1_COOP4_CORRELATION_128 = 36,
	WF_MIC2_COOP4_CORRELATION_128 = 37,
	WF_MIC3_COOP4_CORRELATION_128 = 38,
	WF_MIC4_COOP4_CORRELATION_128 = 39,
	WF_MIC1_COOP5_CORRELATION_128 = 40,
	WF_MIC2_COOP5_CORRELATION_128 = 41,
	WF_MIC3_COOP5_CORRELATION_128 = 42,
	WF_MIC4_COOP5_CORRELATION_128 = 43,
	WF_MIC1_COOP6_CORRELATION_128 = 44,
	WF_MIC2_COOP6_CORRELATION_128 = 45,
	WF_MIC3_COOP6_CORRELATION_128 = 46,
	WF_MIC4_COOP6_CORRELATION_128 = 47,
	WF_DIST_CORRELATION_3x128 = 48,
	WF_SNAPSHOT = 49
};

enum Scaling {
	NO_SCALING, SCALING
};

enum DacChannel {
	DAC_CHAN_1 = 0, DAC_CHAN_2 = 1,
};

uint32_t dac_chan1_waveform_selection = WF_NONE;
uint32_t dac_chan2_waveform_selection = WF_NONE;
uint32_t debug_gui_waveform_selection = WF_NONE;

extern uint16_t dac_waveform[2][NR_OF_SAMPLES * SENDER_OVERSAMPLING];

uint16_t dac_waveform[2][NR_OF_SAMPLES * SENDER_OVERSAMPLING] __attribute__ ((section(".ccmram"))); //stores the dac output waveform, in normal operation this is used to generate the ultrasonic feed for the audio amplifier, for debug reasons intermediate results like correlation can be probed through the dac with an oscilloscope using the functions dac_debug_output()

enum SynchronizationState {
	SYNCHRONIZING, SYNCHRONIZED, MASTER_CLOCK
};

enum IrSignalQuality {
	IR_QUALITY_BAD = 0, IR_QUALITY_GOOD = 1,
};

volatile enum SynchronizationState synchronizationState = SYNCHRONIZING;
volatile enum IrSignalQuality irSignalQuality = IR_QUALITY_BAD;
volatile int32_t synchronization_offset = 0; //used to adjust the local sampling timing to synchronize the slave beacon or robot beacon to the master beacon
volatile float32_t accumulated_synchronization_offset = 0;
volatile float32_t nr_of_pulses_with_good_ir_signal = 0;
volatile float32_t nr_of_pulses_with_bad_ir_signal = 0;

float32_t mic_spectra[NR_OF_MICROPHONES][NR_OF_SAMPLES]; // stores the FFT result of all microphone channels { bin0re; bin0im; bin1re; bin1im; ...}
float32_t samples_f[NR_OF_SAMPLES] __attribute__ ((section(".ccmram"))); // [8192 Bytes] general purpose working array, may be used for calculations or temporary storage
//float32_t samples_i[NR_OF_SAMPLES] __attribute__ ((section(".ccmram"))); // [8192 Bytes] in-phase samples
//float32_t samples_q[NR_OF_SAMPLES] __attribute__ ((section(".ccmram"))); // [8192 Bytes] quadrature samples
float32_t spectrum[NR_OF_SAMPLES] __attribute__ ((section(".ccmram"))); //= { bin0re; bin0im; bin1re; bin1im; ... ; bin1023re; bin1023im}    binFrequency = SAMPLERATE * (binNr / NR_OF_SAMPLES) = binNr * BIN_WIDTH
float32_t spectral_product[NR_OF_SAMPLES] __attribute__ ((section(".ccmram")));


arm_rfft_fast_instance_f32 rfft; // configuration for the cmsis fft function using 2048 samples
arm_rfft_fast_instance_f32 rfft128; // configuration for the cmsis fft function using 128 samples

float32_t samples128_f[128] __attribute__ ((section(".ccmram")));
float32_t samples128_i[128] __attribute__ ((section(".ccmram")));
float32_t samples128_q[128] __attribute__ ((section(".ccmram")));
float32_t spectrum128[128] __attribute__ ((section(".ccmram")));
float32_t spectral_product128[128] __attribute__ ((section(".ccmram")));
float32_t spectra[14][128] __attribute__ ((section(".ccmram")));

float32_t fractional_synchronization_offset = 0.0f;

uint32_t battery_monitor_adc_value = 0;
volatile int32_t sender_counter = 0; // sample couter while actively sending or reveiving
volatile uint32_t echo_counter = 0; // counts the sample intervals to wait for the echo to decay
volatile uint32_t echo_decay_count = ECHO_DECAY_COUNT_SYNCHRONIZED; // nr of sample intervals to wait for the echo (silent sample intervals = echo_decay_count-1)

volatile float battery_voltage = 0.0;

enum BoardBehaviour {
	MASTER_SENDER, // only one beacon may be active as MASTER_SENDER, sending the US Signal 0 and IR gold signal 0
	SLAVE_SENDER_1, // only one beacon may be active as SLAVE_SENDER_1, synchronizing to MASTER SENDER gold code 0 and sending US Signal 1
	SLAVE_SENDER_2, // only one beacon may be active as SLAVE_SENDER_2, synchronizing to MASTER SENDER gold code 0 and sending US Signal 2
	ROBOT_BEACON // an infinite number of ROBOT_BEACONS can be active, synchronizing to MASTER SENDER and calculating their own position
};
enum BoardBehaviour boardBehaviour = SLAVE_SENDER_1; // change this constant to compile the code for different beacons on pcb version V1, in V2, V3 and V3_NRF the behaviour is set by the is switch

enum Signal {
	MIC1 = 0, MIC2 = 1, MIC3 = 2, MIC4 = 3, IR = 4
};

enum SynchronizationSource {
	TO_ROBOT = 0, //synchronize to gold code 0, currently used for all synchronization paths
	GOLD_CODE_1 = 1, //synchronize to gold code 1
	GOLD_CODE_2 = 2  //synchronize to gold code 2
};
uint32_t synchronizationSource = TO_ROBOT;

float32_t reduced_spectrum_distances[NR_OF_US_SENDERS][NR_OF_MICROPHONES];
float32_t reduced_spectrum_unambiguity[NR_OF_US_SENDERS][NR_OF_MICROPHONES];
float32_t reduced_spectrum_max_correlation[NR_OF_US_SENDERS][NR_OF_MICROPHONES];

int32_t nr_of_sync_steps = 2;
int32_t sync_step = 0;
int16_t sync_accumulator[NR_OF_SAMPLES];

int32_t synchonization_offset_debug_out;
uint32_t sync_error = 0;
float32_t quality;

float distances[NR_OF_US_SENDERS] = { 0, 0, 0, 0 }; // stores calculated distance to all four senders (3x fixed beacon + 1x angle signal)
float unambiguousity[NR_OF_US_SENDERS] = { 0, 0, 0, 0 };

float32_t botx = 0.0f;
float32_t boty = 0.0f;
float32_t alt_botx = 0.0f;
float32_t alt_boty = 0.0f;

uint32_t los_mics[NR_OF_US_SENDERS]; //line of sight microphone index for every us channel
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	setup_beacon_base_board(pcb_hardware_version);
	NVIC_SetPriority(USART3_IRQn, 7);
	NVIC_EnableIRQ(USART3_IRQn);

	NVIC_SetPriority(UART5_IRQn, 6);
	NVIC_EnableIRQ(UART5_IRQn);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	/* USER CODE BEGIN 2 */
	uint32_t coop_enabled = 0;
	uint32_t board_id = 0;
	if (pcb_hardware_version == V1){
		board_id = 1;
	}else{
		printf("RCA Eurobot Ultrasonic Beacon\n");
		board_id = get_board_id();
		printf("board_id = %d\n", (int) board_id);
		switch (board_id) {
		case 0:
			boardBehaviour = MASTER_SENDER;
			break;
		case 1:
			boardBehaviour = SLAVE_SENDER_1;
			break;
		case 2:
			boardBehaviour = SLAVE_SENDER_2;
			break;
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			boardBehaviour = ROBOT_BEACON;
			break;
		}
		coop_enabled = is_coop_enabled();
		printf("coop_enabled = %d\n", (int) coop_enabled);
	}

	//<set synchronization and dac behaviour>
	int i;
	switch (boardBehaviour) {
	case MASTER_SENDER:
		printf("boardBehaviour=MASTER_SENDER\n");
		synchronizationState = MASTER_CLOCK;
		for (i = 0; i < (NR_OF_SAMPLES * SENDER_OVERSAMPLING); i++) {
			dac_waveform[0][i] = sender[0][i];
			dac_waveform[1][i] = sender[3][i];
		}
		break;
	case SLAVE_SENDER_1:
		printf("boardBehaviour=SLAVE_SENDER_1\n");
		synchronizationState = SYNCHRONIZING;
		synchronizationSource = TO_ROBOT;
		uint32_t i;
		for (i = 0; i < (NR_OF_SAMPLES * SENDER_OVERSAMPLING); i++) {
			dac_waveform[0][i] = sender[1][i];
			dac_waveform[1][i] = sender[3][i];
		}
		break;
	case SLAVE_SENDER_2:
		printf("boardBehaviour=SLAVE_SENDER_2\n");
		synchronizationState = SYNCHRONIZING;
		synchronizationSource = TO_ROBOT;
		for (i = 0; i < (NR_OF_SAMPLES * SENDER_OVERSAMPLING); i++) {
			dac_waveform[0][i] = sender[2][i];
			dac_waveform[1][i] = sender[3][i];
		}
		break;
	case ROBOT_BEACON:
		printf("boardBehaviour=ROBOT_BEACON_1\n");
		synchronizationState = SYNCHRONIZING;
		synchronizationSource = TO_ROBOT;
		if (coop_enabled == 1) {
			for (i = 0; i < (NR_OF_SAMPLES * SENDER_OVERSAMPLING); i++) {
				dac_waveform[0][i] = sender[board_id][i];
				dac_waveform[1][i] = 0;
			}
		} else {
			for (i = 0; i < (NR_OF_SAMPLES * SENDER_OVERSAMPLING); i++) {
				dac_waveform[0][i] = 0;
				dac_waveform[1][i] = 0;
			}
		}
		break;
	}
	//</set synchronization and dac behaviour>

	if (pcb_hardware_version == V1) {
		printf("Hardware Version 1\n");
	}
	if (pcb_hardware_version == V2) {
		printf("Hardware Version 2\n");
	}
	if (pcb_hardware_version == V3) {
		printf("Hardware Version 3\n");
	}
	if (pcb_hardware_version == V3_NRF) {
		printf("Hardware Version 3 NRF\n");
	}

	printf("synchronizationSource=%i\n", (int) synchronizationSource);
	printf("boardBehaviour=%i\n", boardBehaviour);

	arm_rfft_fast_init_f32(&rfft, NR_OF_SAMPLES);
	arm_rfft_fast_init_f32(&rfft128, 128);

	//nrf_init();
	init_timers_TIM3_TIM5();

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


	if((pcb_hardware_version == V3) || (pcb_hardware_version == V3_NRF)) {
		nrf24_init(board_id);
	}

	//</test newton iterator>


	//dac_chan1_waveform_selection = WF_MIC1_MASTER_CORRELATION_128;
	//dac_chan2_waveform_selection = WF_MIC1;
	dac_chan1_waveform_selection = WF_NONE;
	dac_chan2_waveform_selection = WF_IR_Correlation;

	generate_spectra();
	//testConstants();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */
		if (sampleBufferState == READY_FOR_EVALUATION) {
			set(led4_PC11);

			if (boardBehaviour != MASTER_SENDER)
			{
				IR_synchronisation();
			}
			if (boardBehaviour == ROBOT_BEACON) {

				if (synchronizationState == SYNCHRONIZED)
				{
					US_ranging();

					PositionEstimation();

					if (microphone_position_debug_state == MICROPHONE_POSITION_DEBUG_OUTPUT_ENABLED)
					{
						microphonePositionEstimation();
					}

					Communication();
				}
				reset(led4_PC11);
			}

			if(pcb_hardware_version == V1){
				printf("ss=%i sigq=%i sc=%i frac=%f pg=%i \n",
						synchronizationState,
						irSignalQuality,
						sender_counter,
						fractional_synchronization_offset,
						(int32_t)nr_of_pulses_with_good_ir_signal);
			}

			sampleBufferState = READY_FOR_SWAP;
		}
		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	//HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	//HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	//HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

// 320kHz Interrupt
void TIM5_IRQHandler(void) {
	TIM5->SR &= ~TIM_SR_UIF;

	if (echo_counter == 0) {
		const int is_synchronized = synchronizationState == SYNCHRONIZED ||
				synchronizationState == MASTER_CLOCK;

		if((pcb_hardware_version == V3) || (pcb_hardware_version == V3_NRF)) {
			if (sender_counter == RF_SYNCHRONIZATION_OFFSET && is_synchronized) {
				nrf24_sync();
			}
		}

		// ADC1: ir   at ch 3 and vbat at ch14
		// ADC2: mic1 at ch10 and mic2 at ch11
		// ADC3: mic3 at ch12 and mic4 at ch13

		uint32_t adc_sample_index = sender_counter >> 1;

		if ((sender_counter & 1) == 0) {
			recording_samples_ADC_buf[IR][adc_sample_index] = ADC1->DR;
			recording_samples_ADC_buf[MIC1][adc_sample_index] = ADC2->DR;
			recording_samples_ADC_buf[MIC3][adc_sample_index] = ADC3->DR;

			ADC1->SQR3 = ADC_SQR3_SQ1_0 * 8ul;
			ADC2->SQR3 = ADC_SQR3_SQ1_0 * 11ul;
			ADC3->SQR3 = ADC_SQR3_SQ1_0 * 13ul;

		} else {
			recording_samples_ADC_buf[IR][adc_sample_index] += ADC1->DR;
			recording_samples_ADC_buf[MIC2][adc_sample_index] = ADC2->DR;
			recording_samples_ADC_buf[MIC4][adc_sample_index] = ADC3->DR;

			ADC1->SQR3 = ADC_SQR3_SQ1_0 * 9ul;
			ADC2->SQR3 = ADC_SQR3_SQ1_0 * 10ul;
			ADC3->SQR3 = ADC_SQR3_SQ1_0 * 12ul;

			if ((boardBehaviour == SLAVE_SENDER_1)
					|| (boardBehaviour == SLAVE_SENDER_2)) {
				ADC1->SQR3 = ADC_SQR3_SQ1_0 * 3ul;
			}
		}
		ADC1->CR2 |= ADC_CR2_SWSTART;
		ADC2->CR2 |= ADC_CR2_SWSTART;
		ADC3->CR2 |= ADC_CR2_SWSTART;

		DAC->DHR12RD = dac_waveform[0][sender_counter]
				+ ((uint32_t) (dac_waveform[1][sender_counter]) << 16);

		switch (boardBehaviour) {
		case MASTER_SENDER: {
			uint32_t gold_bit_nr = sender_counter >> 3;
			if (pcb_hardware_version == V1)
			{
				uint32_t bit = 1 - gold_seed[0][gold_bit_nr];
				if (bit) {
					set(buzzer_PE5); //used for gold_seed
				} else {
					reset(buzzer_PE5);
				}
			}else{
				uint32_t bit = gold_seed[0][gold_bit_nr];
				if (bit) {
					set(ir_lighthouse_PC9); //used for gold_seed
				} else {
					reset(ir_lighthouse_PC9);
				}
			}

			/*
			 bit = gold_seed[0][gold_bit_nr];
			 if (bit)
			 {
			 set(ir_sync_to_slave);
			 }else{
			 reset(ir_sync_to_slave);
			 }
			 */
		}
			break;

		case SLAVE_SENDER_1:
		case SLAVE_SENDER_2: {
			/*
			 slaves dont send because the isolation between gold codes is too bad, maybe also isolate channels in frequency domain

			 uint32_t gold_bit_nr = sender_counter >> 3;
			 uint8_t bit = gold_seed[0][gold_bit_nr];
			 if (bit)
			 {
			 set(ir_sync_to_slave);
			 }else{
			 reset(ir_sync_to_slave);
			 }
			 */
		}
			break;

		case ROBOT_BEACON:
			break;
		}
	} else {
		// measure battery voltage

		if (sender_counter == 1) {
			ADC1->SQR3 = ADC_SQR3_SQ1_0 * 14ul;
			ADC1->CR2 |= ADC_CR2_SWSTART;
		} else if (sender_counter == 2) {
			battery_monitor_adc_value = ADC1->DR;

			float adc_voltage = battery_monitor_adc_value * (1.0 / 4096.0)
					* 3.3;
			battery_voltage = adc_voltage * (20000.0 / 6800.0) * (5.23 / 3.88);
			if (battery_voltage < 5.5f) {
				set(batt_low_led_PE4);
				set(ui_led_red_batt_low_PA9);
			} else {
				reset(batt_low_led_PE4);
				reset(ui_led_red_batt_low_PA9);
			}
		}

	}

	sender_counter++;

	if (sender_counter >= NR_OF_SAMPLES * SENDER_OVERSAMPLING) {
		//reset(led3_PC12);  //LED3 is set (pin reset) after send period
		sender_counter = 0;
		echo_counter++;

		if (echo_counter == 1) {
			if (sampleBufferState == READY_FOR_SWAP) {
				int16_t (*temp)[NR_OF_SAMPLES] = recording_samples_ADC_buf;
				recording_samples_ADC_buf = evaluating_samples_ADC_buf;
				evaluating_samples_ADC_buf = temp;
				sampleBufferState = READY_FOR_EVALUATION;
			}
		}

		if (echo_counter == echo_decay_count - 1) {
			// synchronization to master beacon
			if ((synchronizationState == SYNCHRONIZING)
					|| (irSignalQuality == IR_QUALITY_GOOD)) {
				sender_counter = (int32_t)( (float32_t)synchronization_offset / (float32_t)nr_of_sync_steps +0.5f);

				//sender_counter = synchronization_offset *1;
				synchronization_offset = 0;
			}


			if (synchronizationState == SYNCHRONIZED) {
				if (irSignalQuality == IR_QUALITY_GOOD) {
					accumulated_synchronization_offset += sender_counter;
					nr_of_pulses_with_good_ir_signal += 1;
					fractional_synchronization_offset = accumulated_synchronization_offset / nr_of_pulses_with_good_ir_signal;
				} else {
					if (nr_of_pulses_with_good_ir_signal > 50) {

						int32_t now = fractional_synchronization_offset * nr_of_pulses_with_bad_ir_signal;
						int32_t next = fractional_synchronization_offset * (nr_of_pulses_with_bad_ir_signal + 1.0f);

						sender_counter = next - now;
						nr_of_pulses_with_bad_ir_signal += 1.0f;
					}
				}

			}


		}

		if (echo_counter >= echo_decay_count) {
			echo_counter = 0;

			// prepare for first sample

			ADC1->SQR3 = ADC_SQR3_SQ1_0 * 9ul;
			ADC2->SQR3 = ADC_SQR3_SQ1_0 * 10ul;
			ADC3->SQR3 = ADC_SQR3_SQ1_0 * 12ul;

			ADC1->CR2 |= ADC_CR2_SWSTART;
			ADC2->CR2 |= ADC_CR2_SWSTART;
			ADC3->CR2 |= ADC_CR2_SWSTART;

			//set(led3_PC12); // synchrinzation signal to trigger the scope
		}
	}
}

void dac_debug_output(float32_t* values, uint32_t nr_of_values,
		uint32_t dac_samples_per_value, enum DacChannel dac_channel,
		enum Scaling scaling_mode) {
	float32_t offset = 0;
	float32_t scaling_factor = 1;
	if (scaling_mode == SCALING) {
		float32_t maximum;
		uint32_t index_of_maximum;
		arm_max_f32(values, nr_of_values, &maximum, &index_of_maximum);

		float32_t minimum;
		uint32_t index_of_minimum;
		arm_min_f32(values, nr_of_values, &minimum, &index_of_minimum);

		offset = -minimum;
		scaling_factor = DAC_MAX_AMPLITUDE / (maximum - minimum);
	}

	uint32_t i;
	for (i = 0; i < nr_of_values; i++) {
		uint32_t s;
		for (s = 0; s < dac_samples_per_value; s++) {
			dac_waveform[dac_channel][i * dac_samples_per_value + s] =
					(uint32_t) (((values[i] + offset) * scaling_factor)
							+ DAC_MIN);
		}
	}
}

void remove_dc_from_spectrum(uint32_t waveform_id, float32_t* values) {
	if ((waveform_id == WF_IR_Spectrum) || (waveform_id == WF_MIC1_SPECTRUM)
			|| (waveform_id == WF_MIC2_SPECTRUM)
			|| (waveform_id == WF_MIC3_SPECTRUM)
			|| (waveform_id == WF_MIC4_SPECTRUM)) {
		uint32_t i;
		for (i = 0; i < 20; i++) {
			values[0] = 0.0;
		}
	}
}

void configurable_debug_output(uint32_t waveform_id, float32_t* values,
		uint32_t nr_of_values) {
	enum Scaling scaling_mode = SCALING;
	if ((waveform_id <= MIC4) || (waveform_id == WF_SNAPSHOT)) {
		//scaling_mode = NO_SCALING;
	}

	if (dac_chan1_waveform_selection == waveform_id) {
		remove_dc_from_spectrum(waveform_id, values);
		dac_debug_output(values, nr_of_values,
				(uint32_t) NR_OF_SAMPLES / nr_of_values, DAC_CHAN_1,
				scaling_mode);
	}

	if (dac_chan2_waveform_selection == waveform_id) {
		remove_dc_from_spectrum(waveform_id, values);
		dac_debug_output(values, nr_of_values,
				(uint32_t) NR_OF_SAMPLES / nr_of_values, DAC_CHAN_2,
				scaling_mode);
	}

	if (debug_gui_waveform_selection == waveform_id) {
		uint32_t i;
		for (i = 0; i < nr_of_values; i++) {
			printf("a%i[%i]=%f\n", waveform_id, i, values[i]);
		}
		printf("update plot\n");

		//debug_gui_waveform_selection = WF_NONE; // moved to end of main loop
	}

	if (debug_gui_waveform_selection == WF_SNAPSHOT) {
		if ((waveform_id == WF_MIC1) || (waveform_id == WF_MIC2)
				|| (waveform_id == WF_MIC3) || (waveform_id == WF_MIC4)) {
			uint32_t offset = 0;
			switch (waveform_id) {
			case WF_MIC1:
				offset = 2048 * 0;
				break;
			case WF_MIC2:
				offset = 2048 * 1;
				break;
			case WF_MIC3:
				offset = 2048 * 2;
				break;
			case WF_MIC4:
				offset = 2048 * 3;
				break;
			}

			uint32_t i;
			for (i = 0; i < nr_of_values; i++) {
				printf("a%i[%i]=%f\n", 37, i + offset, values[i]);
			}

			if (waveform_id == WF_MIC4) {
				printf("update plot\n");
			}
		}
	}

}

#define VARIANCE_HISTORY_LENGTH 20
int32_t var_history_buffer[VARIANCE_HISTORY_LENGTH];
uint32_t var_iterator = 0;

float32_t get_variance(int32_t latest_value) {
	var_history_buffer[var_iterator] = latest_value;
	var_iterator++;
	if (var_iterator >= VARIANCE_HISTORY_LENGTH) {
		var_iterator = 0;
	}
	uint32_t i;
	int32_t sum = 0;
	for (i = 0; i < VARIANCE_HISTORY_LENGTH; i++) {
		sum += var_history_buffer[i];
	}
	float32_t average = (float32_t) sum / (float32_t) (VARIANCE_HISTORY_LENGTH);
	float32_t variance_times_buffer_length = 0;
	for (i = 0; i < VARIANCE_HISTORY_LENGTH; i++) {
		int32_t error = average - var_history_buffer[i];
		variance_times_buffer_length += error * error;
	}
	return variance_times_buffer_length / (float32_t) VARIANCE_HISTORY_LENGTH;
}

void IR_synchronisation(void) {
	int i;
	if (pcb_hardware_version == V1) {
		for (i = 0; i < NR_OF_SAMPLES; i++) {
			evaluating_samples_ADC_buf[IR][i] =
					evaluating_samples_ADC_buf[MIC4][i];
		}
	}

	/*
	 for(i=0;i<NR_OF_SAMPLES;i++)
	 {
	 samples_f[i] = (float32_t)evaluating_samples_ADC_buf[IR][i];
	 }
	 */

	sync_step++;
	if (sync_step == 1) {
		for (i = 0; i < NR_OF_SAMPLES; i++) {
			sync_accumulator[i] = evaluating_samples_ADC_buf[IR][i];
		}
	} else {
		for (i = 0; i < NR_OF_SAMPLES; i++) {
			sync_accumulator[i] += evaluating_samples_ADC_buf[IR][i];
		}
	}

	if (sync_step >= nr_of_sync_steps) {
		for (i = 0; i < NR_OF_SAMPLES; i++) {
			samples_f[i] =  sync_accumulator[i];
		}
		sync_step = 0;

		samples_f[0] = samples_f[1];

		configurable_debug_output(WF_IR, samples_f, NR_OF_SAMPLES);

		arm_rfft_fast_f32(&rfft, samples_f, spectrum, 0); //1.162ms
		configurable_debug_output(WF_IR_Spectrum, spectrum, NR_OF_SAMPLES);

		// suppress ir beacon on 24.4 and 23.6kHz
		float center_frequency = 24000.0f;
		float block_span = 2000.0f;
		uint32_t lowerBin = (center_frequency - (block_span / 2.0f))
				/ BIN_WIDTH;
		uint32_t upperBin = (center_frequency + (block_span / 2.0f))
				/ BIN_WIDTH;
		uint32_t bin;
		for (bin = lowerBin; bin <= upperBin; bin++) {
			spectrum[(bin * 2) + 0] = 0.0f;
			spectrum[(bin * 2) + 1] = 0.0f;
		}

		//correlate
		arm_cmplx_mult_cmplx_f32(spectrum, (float32_t*) spectra_ir[0],
				spectral_product, NR_OF_SAMPLES / 2);
		arm_rfft_fast_f32(&rfft, spectral_product, samples_f, 1);

		for (i = 0; i < NR_OF_SAMPLES; i++) {
			if (samples_f[i] < 0.0f) {
				samples_f[i] = 0.0f;
			}
		}

		configurable_debug_output(WF_IR_Correlation, samples_f, NR_OF_SAMPLES);

		float32_t maximum;
		uint32_t index_of_maximum;
		arm_max_f32(samples_f, NR_OF_SAMPLES / 4, &maximum, &index_of_maximum);

		float32_t maximum_at_end;
		uint32_t index_of_maximum_at_end;
		arm_max_f32(&(samples_f[NR_OF_SAMPLES - NR_OF_SAMPLES / 4]),
		NR_OF_SAMPLES / 4, &maximum_at_end, &index_of_maximum_at_end);
		if (maximum_at_end > maximum) {
			maximum = maximum_at_end;
			index_of_maximum = index_of_maximum_at_end + NR_OF_SAMPLES
					- NR_OF_SAMPLES / 4;
		}

		ir_correlation_max = maximum; // displayed in gui

		// get the signal quality as maximum of main lobe to maximum of side lobe
		float32_t maximum_side_lobe = 0.0;
		int32_t offset = 10;
		int32_t c;
		for (c = 0; c < NR_OF_SAMPLES / 4; c += IR_QUALITY_STEP_SIZE) {
			if ((abs(c - (int32_t) index_of_maximum) > offset)
					&& (abs(
							c - ((int32_t) index_of_maximum - NR_OF_SAMPLES)
									> offset))) {
				if (maximum_side_lobe < fabs(samples_f[c])) {
					maximum_side_lobe = fabs(samples_f[c]);
				}
			}
		}
		quality = maximum / maximum_side_lobe;

		if ((quality >= IR_QUALITY_GOOD_THRESHOLD)) {
			if (pcb_hardware_version == V1){
				led_on(led5_PC10);
			}else{
				led_on(led_slave_beacon_sync);
			}
			irSignalQuality = IR_QUALITY_GOOD;
		} else {
			if (pcb_hardware_version == V1){
				led_off(led5_PC10);
			}else{
				led_off(led_slave_beacon_sync);
			}
			irSignalQuality = IR_QUALITY_BAD;
		}


		int32_t synchronization_offset_t = 0;

		if (irSignalQuality == IR_QUALITY_GOOD){

			if (index_of_maximum < (NR_OF_SAMPLES/4)){
				synchronization_offset_t = -1 * (int32_t) index_of_maximum;
			}

			if (index_of_maximum > (NR_OF_SAMPLES - NR_OF_SAMPLES/4)){
				synchronization_offset_t = NR_OF_SAMPLES - (int32_t) index_of_maximum;
			}

		}else{

			if (synchronizationState == SYNCHRONIZING){
				synchronization_offset_t = -NR_OF_SAMPLES/5; //if signal is bad, then shift timeslot by one fifth
			}
		}

		synchronization_offset = synchronization_offset_t;
		synchonization_offset_debug_out = synchronization_offset_t;



		/*
		if ((irSignalQuality == IR_QUALITY_GOOD) || (synchronizationState == SYNCHRONIZING)) {
			int32_t synchronization_offset_t = 0;
			{

				if ((quality >= 1.5f) && ((index_of_maximum < (NR_OF_SAMPLES/4)) || (index_of_maximum > (NR_OF_SAMPLES - NR_OF_SAMPLES/4)))) {
					if (index_of_maximum < (NR_OF_SAMPLES/2)) {
						synchronization_offset_t = -1 * (int32_t) index_of_maximum;
					} else {
						synchronization_offset_t = NR_OF_SAMPLES - (int32_t) index_of_maximum;
					}
				} else {

					synchronization_offset_t = -NR_OF_SAMPLES/5; //if signal is bad, then shift timeslot by one fifth

				}

				//if (index_of_maximum == index_of_maximum
				synchronization_offset = synchronization_offset_t;
				synchonization_offset_debug_out = synchronization_offset_t;
			}

			if ((synchronizationState == SYNCHRONIZED)
					&& ((index_of_maximum > 100) && (index_of_maximum < 1900))) {
				if (sync_error == 0) {
					// error after synchronization
					// put waveform on DAC_CHAN1 and correlation on DAC_CHAN2
					dac_chan2_waveform_selection = WF_IR_Correlation;
					//configurable_debug_output(WF_IR_Correlation, samples_f, NR_OF_SAMPLES);
					for (i = 0; i < NR_OF_SAMPLES; i++) {
						samples_f[i] =
								(float32_t) evaluating_samples_ADC_buf[IR][i];
					}
					//dac_chan1_waveform_selection = WF_IR;
					//configurable_debug_output(WF_IR, samples_f, NR_OF_SAMPLES);
					sync_error = 1;
					printf(
							"sync error i=%i q=%i%% IR on DAC1 and IR_Corr on DAC2\n",
							index_of_maximum, (uint32_t) (quality * 100.0f));
				}
			}

			if (synchronizationState == SYNCHRONIZING)
				printf("SYNCHRONIZING m=%i q=%i%% s_o=%i\n", index_of_maximum,
						(int32_t) (quality * 100.0f), synchronization_offset);
		} else {
			//printf("%i-%i q=%i\n", index_of_maximum, (uint32_t)maximum, (uint32_t)quality); //synchronized status update
		}
		*/

		float32_t variance_of_synchronization_offset = get_variance(synchronization_offset);
		if ((irSignalQuality == IR_QUALITY_GOOD) && (variance_of_synchronization_offset < 1.0f)
				&& (synchronizationState == SYNCHRONIZING)) {
			synchronizationState = SYNCHRONIZED;
			printf("SYNCHRONIZED\n");
			set(ui_led_green_synchronized_PA10);
		}


		//<ui LED>
		if (irSignalQuality == IR_QUALITY_GOOD) {
			if (pcb_hardware_version == V1){
				led_on(led_slave_beacon_sync);
			}else{
				set(ui_led_yellow_ir_signal_ok_PA11);
			}
		} else {
			if (pcb_hardware_version == V1){
				led_off(led_slave_beacon_sync);
			}else{
				reset(ui_led_yellow_ir_signal_ok_PA11);
			}
		}
		//</ui LED>
	}

	/*
	if (boardBehaviour != ROBOT_BEACON) {
		uint32_t mic;
		for (mic = MIC1; mic <= IR; mic++) {
			for (i = 0; i < NR_OF_SAMPLES; i++) {
				samples_f[i] = (float32_t) evaluating_samples_ADC_buf[mic][i];
			}
			configurable_debug_output(WF_MIC1 + mic, samples_f, NR_OF_SAMPLES);
		}
	}
	*/
}

/*
 * get_index_of_first_hill: find lowest local maximum which is bigger than maximum*echo_threshold_factor, return 0 if threshold is not reached
 */
uint32_t get_index_of_first_hill(float maximum, uint32_t index_of_maximum,
		float* samples_f, float threshold, float echo_threshold_factor,
		uint32_t start) {
	//find top of first hill which is bigger than (echo_threshold_factor * maximum)
	uint32_t index;
	if (maximum < threshold) {
		index = 0;
	} else {

		float32_t echo_threshold = maximum * echo_threshold_factor;

		index = start;
		while ((samples_f[index] < echo_threshold) && (index < index_of_maximum))
		//while (samples_f[index] < echo_threshold)
		{
			index++;
		}
		//climb hill
		//while ( (samples_f[index]<samples_f[index+1]) || (samples_f[index]<samples_f[index-1]) )
		while ((samples_f[index] < samples_f[index + 1])) {
			index++;
		}
	}
	return index;
}

/*
 function [p,y,a] = qint(ym1,y0,yp1)
 %QINT - quadratic interpolation of three adjacent samples
 %
 % [p,y,a] = qint(ym1,y0,yp1)
 %
 % returns the extremum location p, height y, and half-curvature a
 % of a parabolic fit through three points.
 % Parabola is given by y(x) = a*(x-p)^2+b,
 % where y(-1)=ym1, y(0)=y0, y(1)=yp1.

 p = (yp1 - ym1)/(2*(2*y0 - yp1 - ym1));
 y = y0 - 0.25*(ym1-yp1)*p;
 a = 0.5*(ym1 - 2*y0 + yp1);*/
float get_position_of_interpolated_peak(float32_t ym1, float32_t y0,
		float32_t yp1) {
	float32_t position = (yp1 - ym1) / (2.0f * (2.0f * y0 - yp1 - ym1));
	float32_t peak_value = y0 - 0.25f * (ym1 - yp1) * position;
	return position;
}

void testConstants(){

	dac_chan1_waveform_selection = WF_MIC1_SPECTRUM;
	dac_chan2_waveform_selection = WF_MIC1;

	uint32_t i;
	for (i = 0; i < NR_OF_SAMPLES; i++) {
		samples_f[i] = (float32_t) sender[0][i*2];
	}

	configurable_debug_output(WF_MIC1, samples_f, NR_OF_SAMPLES);

	uint32_t mic = 0;
	arm_rfft_fast_f32(&rfft, samples_f, &(mic_spectra[mic][0]), 0);
	for (i = 0; i < 20; i++) {
		mic_spectra[mic][i] = 0.0f;
	}

	configurable_debug_output(WF_MIC1_SPECTRUM, mic_spectra[mic], NR_OF_SAMPLES);


	//<correlation with 128 samples (64 complex bins) and frequency shifted signals>
	uint32_t lowerBin = lowest_spectral_bin[0];
	arm_cmplx_mult_cmplx_f32(
			(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
			(float32_t*) &(spectra[(2 * 0) + 1][0]),
			spectral_product128, 128 / 2);
	arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_q,
			1);

	arm_cmplx_mult_cmplx_f32(
			(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
			(float32_t*) &(spectra[(2 * 0) + 0][0]),
			spectral_product128, 128 / 2);
	arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_i,
			1);

	for (i = 0; i < 128; i++) {
		arm_sqrt_f32(
				samples128_i[i] * samples128_i[i]
						+ samples128_q[i] * samples128_q[i],
				&(samples128_f[i]));
	}

	configurable_debug_output(
				WF_MIC1_MASTER_CORRELATION_128,
				samples128_f, 128);
	//</correlation with 128 samples (64 complex bins) and frequency shifted signals>



	//<generate reference spectrum>
	/*
	float spectrum_128_i[128];
	for (i=0 ; i<128 ; i++){
		spectrum_128_i[i] = mic_spectra[mic][(lowerBin * 2)+i];
	}

	float spectrum_128_q[128];
	for (i=0 ; i<64 ; i++){
		spectrum_128_q[(i*2)+0] = spectrum_128_i[(i*2)+1] * -1.0;
		spectrum_128_q[(i*2)+1] = spectrum_128_i[(i*2)+0];
	}

	arm_cmplx_mult_cmplx_f32(
			(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
			(float32_t*) &(spectrum_128_i),
			spectral_product128, 128 / 2);
	arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_q,
			1);

	arm_cmplx_mult_cmplx_f32(
			(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
			(float32_t*) &(spectrum_128_q),
			spectral_product128, 128 / 2);
	arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_i,
			1);

	for (i = 0; i < 128; i++) {
		arm_sqrt_f32(
				samples128_i[i] * samples128_i[i]
						+ samples128_q[i] * samples128_q[i],
				&(samples128_f[i]));
	}
	configurable_debug_output(
		WF_MIC1_MASTER_CORRELATION_128,
		samples128_f, 128);
	*/
	//</generate reference spectrum>


	dac_chan1_waveform_selection = WF_NONE;
	dac_chan2_waveform_selection = WF_NONE;

}


/* generate the reference spectrum (var spectra) from the us sender waveforms in memory (var sender)*/
void generate_spectra(void){

	int32_t us_signal_nr;
	//for (us_signal_nr=0 ; us_signal_nr<NR_OF_US_SENDERS ; us_signal_nr++ ){

	for (us_signal_nr=NR_OF_US_SENDERS-1 ; us_signal_nr >= 0 ; us_signal_nr-- ){

		int32_t i;

		for (i = 0; i < NR_OF_SAMPLES; i++) {
			int32_t shifted_index = i-342;
			if (shifted_index < 0){
				shifted_index += NR_OF_SAMPLES;
			}

			samples_f[shifted_index] = (float32_t) sender[us_signal_nr][i*2];
		}

		arm_rfft_fast_f32(&rfft, samples_f, spectrum, 0);

		uint32_t lowerBin = lowest_spectral_bin[us_signal_nr];
		for (i=0 ; i<128 ; i++){
			spectra[2*us_signal_nr+0][i] = spectrum[(lowerBin * 2)+i];
		}

		for (i=0 ; i<64 ; i++){
			spectra[2*us_signal_nr+1][(i*2)+0] = spectra[2*us_signal_nr+0][(i*2)+1] * -1.0;
			spectra[2*us_signal_nr+1][(i*2)+1] = spectra[2*us_signal_nr+0][(i*2)+0];
		}


		for(i=0 ; i<20 ; i++)
		{
			spectra[2*us_signal_nr+0][(i*2)+0] = 0.0f;
			spectra[2*us_signal_nr+0][(i*2)+1] = 0.0f;
			spectra[2*us_signal_nr+1][(i*2)+0] = 0.0f;
			spectra[2*us_signal_nr+1][(i*2)+1] = 0.0f;

			spectra[2*us_signal_nr+0][((63-i)*2)+0] = 0.0f;
			spectra[2*us_signal_nr+0][((63-i)*2)+1] = 0.0f;
			spectra[2*us_signal_nr+1][((63-i)*2)+0] = 0.0f;
			spectra[2*us_signal_nr+1][((63-i)*2)+1] = 0.0f;
		}

	}

	uint32_t db_sig, db_bin;
	for (db_sig=0 ; db_sig<2*NR_OF_US_SENDERS ; db_sig++){
		for (db_bin=0 ; db_bin<128 ; db_bin++){
			//printf("spectra[%lu][%lu]=%f\n", db_sig, db_bin, spectra[db_sig][db_bin]);
		}
	}

}

void US_ranging(void) {
	// calculate fft of all microphone signals and store in mic_spectra
	uint32_t mic, i;
	for (mic = MIC1; mic <= MIC4; mic++) {
		for (i = 0; i < NR_OF_SAMPLES; i++) {
			samples_f[i] = (float32_t) evaluating_samples_ADC_buf[mic][i];
		}
		configurable_debug_output(WF_MIC1 + mic, samples_f, NR_OF_SAMPLES);

		arm_rfft_fast_f32(&rfft, samples_f, mic_spectra[mic], 0);
		for (i = 0; i < 20; i++) {
			mic_spectra[mic][i] = 0.0f;
		}

		//<magnitude spectrum>
		if (   ((dac_chan1_waveform_selection >= WF_MIC1_SPECTRUM)&&(dac_chan1_waveform_selection <= WF_MIC4_SPECTRUM))
			|| ((dac_chan2_waveform_selection >= WF_MIC1_SPECTRUM)&&(dac_chan2_waveform_selection <= WF_MIC4_SPECTRUM)) )
		{
			for (i = 0; i < NR_OF_SAMPLES/2; i++) {
				arm_sqrt_f32( mic_spectra[mic][2*i] * mic_spectra[mic][2*i]
								+ mic_spectra[mic][(2*i)+1] * mic_spectra[mic][(2*i)+1],
						&(samples_f[i]));
			}
			configurable_debug_output(WF_MIC1_SPECTRUM + mic, samples_f, NR_OF_SAMPLES/2);
		}
		//</magnitude spectrum>

		//configurable_debug_output(WF_MIC1_SPECTRUM + mic, mic_spectra[mic],
		//NR_OF_SAMPLES);
	}

	uint32_t sender;
	for (sender = 0; sender < NR_OF_US_SENDERS; sender++) {

		//<correlation with reduced spectrum>
		//if (sender == 0)
		{
			int mic;
			for (mic = 0; mic < NR_OF_MICROPHONES; mic++) {

				//<correlation with 128 samples (64 complex bins) and frequency shifted signals>
				uint32_t lowerBin = lowest_spectral_bin[sender];

				arm_cmplx_mult_cmplx_f32(
						(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
						(float32_t*) &(spectra[(2 * sender) + 1][0]),
						spectral_product128, 128 / 2);
				arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_q,
						1);

				arm_cmplx_mult_cmplx_f32(
						(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
						(float32_t*) &(spectra[(2 * sender) + 0][0]),
						spectral_product128, 128 / 2);

				arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_i,
						1);

				uint32_t i;
				for (i = 0; i < 128; i++) {
					arm_sqrt_f32(
							samples128_i[i] * samples128_i[i]
									+ samples128_q[i] * samples128_q[i],
							&(samples128_f[i]));
				}
				//</correlation with 128 samples (64 complex bins) and frequency shifted signals>

				configurable_debug_output(
						WF_MIC1_MASTER_CORRELATION_128 + mic + (sender * NR_OF_MICROPHONES),
						samples128_f, 128);

				float maximum;
				uint32_t index_of_maximum;
				arm_max_f32(samples128_f, 128, &maximum, &index_of_maximum);

				//filter echos
				index_of_maximum = get_index_of_first_hill(maximum,
						index_of_maximum, samples128_f, 5000.0f * 1000.0f, //threshold
						0.5f,         //echo_threshold_factor)
						0);       // start

				//<distance calculation>
				float32_t distance_mm = 0;
				if (index_of_maximum != 0) {
					//interpolate around maximum
					float32_t relativePeakPosition =
							get_position_of_interpolated_peak(
									samples128_f[index_of_maximum - 1],
									samples128_f[index_of_maximum],
									samples128_f[index_of_maximum + 1]);

					distance_mm = (((index_of_maximum + relativePeakPosition)
							* (330.0 / 160000.0) * 16 - 0.481 + 0.59 + 0.03)
							* 1000);
					distance_mm = (distance_mm - 810.0f) * 1.0472f;
				}
				reduced_spectrum_distances[sender][mic] = distance_mm;
				//</distance calculation>

				//<unambigiousity = magnitude of main lobe to magnitude of side lobe of correlation>
				uint32_t ignore_mask_half_span = 9;
				float correlation = samples128_f[index_of_maximum];

				int32_t mask_start = index_of_maximum - ignore_mask_half_span;
				int32_t mask_stop = index_of_maximum + ignore_mask_half_span;
				if (mask_start < 0) {
					mask_start = 0;
				};
				if (mask_stop > 127) {
					mask_stop = 127;
				};

				for (i = mask_start; i <= mask_stop; i++) {
					samples128_f[i] = 0.0;
				}
				float masked_maximum;
				uint32_t masked_maximum_index;
				arm_max_f32(samples128_f, 128, &masked_maximum,
						&masked_maximum_index);
				reduced_spectrum_unambiguity[sender][mic] = (correlation / masked_maximum) - 1.3f;

				if (reduced_spectrum_unambiguity[sender][mic] < 0.0){ reduced_spectrum_unambiguity[sender][mic]=0.0;}

				reduced_spectrum_max_correlation[sender][mic] = correlation;
				//</unambigiousity>
			}

			//<find best microphone> for beacon to robot distance measurement by selecting the microphone with the lowest valid distance
			int32_t los_mic = MIC1;
			/* based on distance
			for (mic = 1; mic < NR_OF_MICROPHONES; mic++) {
				if (reduced_spectrum_distances[sender][mic] > 0) {
					if ((reduced_spectrum_distances[sender][los_mic] > reduced_spectrum_distances[sender][mic])
							|| (reduced_spectrum_distances[sender][los_mic] == 0)) {
						los_mic = mic;
						los_mics[sender] = mic;
					}
				}
			}
			*/

			/* based on quality
			for (mic = 1; mic < NR_OF_MICROPHONES; mic++) {
				if (reduced_spectrum_unambiguity[sender][mic] > 2.0f) {
					if ((reduced_spectrum_distances[sender][los_mic] > reduced_spectrum_distances[sender][mic])
							|| (reduced_spectrum_unambiguity[sender][los_mic] < 2.0f)) {
						los_mic = mic;
						los_mics[sender] = mic;
					}
				}
			}
			*/

			/* based on correlation
			for (mic = 1; mic < NR_OF_MICROPHONES; mic++){
				if (reduced_spectrum_max_correlation[sender][mic] > reduced_spectrum_max_correlation[sender][los_mic])
				{
					los_mic = mic;
					los_mics[sender] = mic;
				}

			}
			*/

			for (mic=MIC2 ; mic < NR_OF_MICROPHONES; mic++){
				if(reduced_spectrum_unambiguity[sender][mic] > reduced_spectrum_unambiguity[sender][los_mic]){
					los_mic = mic;
				}
			}


			for (mic=MIC1 ; mic < NR_OF_MICROPHONES; mic++){
				if (  (reduced_spectrum_unambiguity[sender][mic] > 1.0f)
					&&(reduced_spectrum_distances[sender][mic] < reduced_spectrum_distances[sender][los_mic]))
				{
					los_mic = mic;
				}
			}
			los_mics[sender] = los_mic;
			//</find best microphone>

			/*
			unambiguousity[sender] = reduced_spectrum_unambiguity[sender][los_mic];

			unambiguousity[sender] *= reduced_spectrum_max_correlation[sender][los_mic] / (1000.0*8000000.0/128.0);
			distances[sender] = reduced_spectrum_distances[sender][los_mic]	/ 1000.0f +0.036; // convert from mm to m
			*/

		}
		//</correlation with reduced spectrum>

		//<debug output for correlation color map generation>
		if (debug_gui_waveform_selection == WF_DIST_CORRELATION_3x128) {
			uint32_t sender;
			for (sender = 0; sender < 3; sender++) {
				mic = los_mics[sender];

				uint32_t lowerBin = lowest_spectral_bin[sender];
				arm_cmplx_mult_cmplx_f32(
						(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
						(float32_t*) &(spectra[(2 * sender) + 1][lowerBin * 2]),
						spectral_product128, 128 / 2);
				arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_q,
						1);
				arm_cmplx_mult_cmplx_f32(
						(float32_t*) &(mic_spectra[mic][lowerBin * 2]),
						(float32_t*) &(spectra[(2 * sender) + 0][lowerBin * 2]),
						spectral_product128, 128 / 2);
				arm_rfft_fast_f32(&rfft128, spectral_product128, samples128_i,
						1);

				uint32_t i;
				for (i = 0; i < 128; i++) {
					arm_sqrt_f32(
							samples128_i[i] * samples128_i[i]
									+ samples128_q[i] * samples128_q[i],
							&(samples128_f[i]));
				}

				for (i = 0; i < 16; i++) {
					samples128_f[i] = 0.0f;
				}

				for (i = 0; i < 128; i++) {
					dist_correlation_3x128[sender * 128 + i] = samples128_f[i];
				}

			}
			configurable_debug_output(WF_DIST_CORRELATION_3x128,
					dist_correlation_3x128, 3 * 128);

			unambiguousity[2] = debug_gui_waveform_selection;
		}
		//</debug output for correlation color map generation>
	}


	if (us_ranging_matrix_debug_output_state == US_RANGING_MATRIX_DEBUG_OUTPUT_ENABLED)
	{
		int mic;
		for(mic=0; mic<NR_OF_MICROPHONES ; mic++)
		{
			int us_sender;
			for (us_sender=0 ; us_sender<NR_OF_US_SENDERS ; us_sender++)
			{
				printf("mic%i s=%i d=%i u=%i c=%i\n", mic+1, us_sender,
						(int)(reduced_spectrum_distances[us_sender][mic]*10.0f),
						(int)(reduced_spectrum_unambiguity[us_sender][mic]*10.0f),
						(int)(reduced_spectrum_max_correlation[us_sender][mic]));
			}
		}
	}

	//<add offset to cooperatice beacons to compensate for different acoustic structure>
	//int mic;
	for(mic=0; mic<NR_OF_MICROPHONES ; mic++)
	{
		int us_sender = 3;
		reduced_spectrum_distances[us_sender][mic] += 75;

		us_sender = 6;
		reduced_spectrum_distances[us_sender][mic] += 75;
	}
	//</add>
}

struct Packet packets[8];

float mic_pos_x[NR_OF_MICROPHONES];
float mic_pos_y[NR_OF_MICROPHONES];

void microphonePositionEstimation(){



	float tempX,tempY;

	int32_t mic;
	for(mic=0 ; mic<NR_OF_MICROPHONES ;  mic++){
		float distances[NR_OF_US_SENDERS];
		float quality[NR_OF_US_SENDERS];
		int32_t sender;
		for(sender=0 ; sender<NR_OF_US_SENDERS; sender++){
			distances[sender] = reduced_spectrum_distances[sender][mic];
			quality[sender] = reduced_spectrum_unambiguity[sender][mic] * reduced_spectrum_max_correlation[sender][mic] / (1000.0*8000000.0/128.0);
		}

		get_robot_position_newton_bisection_coop(&(mic_pos_x[mic]), &(mic_pos_y[mic]), //robot position
					&tempX, &tempY, distances, // distances to Beacon
					quality, //Signal Qualities
					beaconX, beaconY, //fixed beacon positions
					NR_OF_US_SENDERS,
					mic_pos_x[mic], mic_pos_y[mic]);

		printf("m%ix=%i m%iy=%i ", mic+1, (int32_t)(mic_pos_x[mic]*10.0f), mic+1, (int32_t)(mic_pos_y[mic]*10.0f));

	}
	printf("\n");
}

void print_robot_rf_packets(){
	int32_t id;
	for (id=3 ; id<3+4 ; id++)
	{
		printf("id=%d x=%d y=%d d[", id, packets[id].x, packets[id].y);
		int sender;
		for (sender=0 ; sender<NR_OF_US_SENDERS-1 ; sender++)
		{
			printf("%d;",packets[id].d[sender]);
		}
		printf("%d] q[", packets[id].d[NR_OF_US_SENDERS-1]);
		for (sender=0 ; sender<NR_OF_US_SENDERS-1 ; sender++)
		{
			printf("%d;",packets[id].q[sender]);
		}
		printf("%d] f=%d\n", packets[id].q[NR_OF_US_SENDERS-1], packets[id].flags);
	}
}

void PositionEstimation(void) {
	//get_coop_data(0, &beaconX[3], &beaconY[3], &unambiguousity[3]);

	nrf24_get_packets(packets);

	if (nrf_packet_content_debug_output == NRF_PACKET_CONTENT_DEBUG_OUTPUT_ENABLED){
		print_robot_rf_packets();
	}

	int32_t i;
	for (i=3 ; i<NR_OF_US_SENDERS ; i++){
		beaconX[i] = packets[i].x / 10.0f;
		beaconY[i] = packets[i].y / 10.0f;
	}


	for (i=0 ; i<NR_OF_US_SENDERS ; i++){
		if (unambiguousity[i] < 10){
			unambiguousity[i] = 0;
		}
	}

	float quality[NR_OF_US_SENDERS];
	float distances[NR_OF_US_SENDERS];
	for (i=0 ; i<NR_OF_US_SENDERS ; i++){

		float capped_unambiguity = reduced_spectrum_unambiguity[i][los_mics[i]];
		if (capped_unambiguity > 2.0){
			capped_unambiguity = 2.0;
		}

		float capped_distance = reduced_spectrum_distances[i][los_mics[i]];
		if (capped_distance > 4000){
			capped_distance = 4000;
		}

		float capped_correlation =reduced_spectrum_max_correlation[i][los_mics[i]];
		if (capped_correlation > 100000000)
		{
			capped_correlation = 100000000;
		}

		quality[i] = capped_unambiguity * capped_correlation * capped_distance;
		distances[i] = reduced_spectrum_distances[i][los_mics[i]] + AVG_MIC_DISTANCE_TO_BOARD_CENTER_MM;

		if (distances[i] == 0.0f) {
			quality[i] = 0.0f;
		}


	}

	//<disable coop by setting quality of signals 3 to 7 to zero>
	for (i=3 ; i<7 ; i++)
	{
		quality[i] = 0.0;
	}
	//</disable coop>

	get_robot_position_newton_bisection_coop(&botx, &boty, //robot position
			&alt_botx, &alt_boty, distances, // distances to Beacon
			quality, //Signal Qualities
			beaconX, beaconY, //fixed beacon positions
			NR_OF_US_SENDERS,
			botx, boty);
}

void Communication(void) {
	int i;
	for (i = 0; i < NR_OF_US_SENDERS; i++) {
		printf("d%i=%i u%i=%i ", i, (int) ((reduced_spectrum_distances[i][los_mics[i]] + AVG_MIC_DISTANCE_TO_BOARD_CENTER_MM) * 10.0f), (int) i,
				(int) (reduced_spectrum_unambiguity[i][los_mics[i]]*10.0f));
	}

	printf(" x=%i y=%i", (int16_t)(botx*10.0f), (int16_t)(boty*10.0f));
	printf(" ax=%i ay=%i", (int16_t)(alt_botx*10.0f), (int16_t)(alt_boty*10.f));
	printf(" cx=%i cy=%i", (int16_t) beaconX[3], (int16_t) beaconY[3]);
	printf(" IRQ=%i IR_o=%i", (int32_t) (quality * 100.0f),
			synchonization_offset_debug_out);
	printf(" IRC=%i OD=%i s=%i", (int32_t) ir_correlation_max,
			(int32_t) ((accumulated_synchronization_offset
					/ nr_of_pulses_with_good_ir_signal) * 1000.0f),
			(synchronizationState & 0x03) + ((irSignalQuality & 0x01) * 4));

	printf("\n");


	if((pcb_hardware_version == V3) || (pcb_hardware_version == V3_NRF)) {
		struct Packet packet = {
			.x = (int16_t)(botx*10.0f),
			.y = (int16_t)(boty*10.0f),
			.flags = 0,
			.id = get_board_id(),
			.d = {},
			.q = {}
		};

		for (i = 0; i < NR_OF_US_SENDERS; i++) {
			packet.d[i] = (uint16_t)(distances[i] * 10.0f);
			packet.q[i] = (uint16_t)(unambiguousity[i]);
		}

		reset(led4_PC11);
		nrf24_set_data(&packet);
	}

	//<matlab debug gui interface>
	debug_gui_waveform_selection = WF_NONE;
	char character = //SER_GetChar();
			serial_debug_read();
	if (character != 255) {

		printf( "debug uart %d\n", character);



		int operation = character >> 6;
		int payload = character & 0x3f;

		if (wf_debug_interface == WF_DEBUG_OUTPUT_ENABLED){
			int waveform = payload;

			if (operation == OP_DAC_CHAN1) {
				dac_chan1_waveform_selection = waveform;
				if (waveform == WF_NONE2) {
					for (i = 0; i < NR_OF_SAMPLES * SENDER_OVERSAMPLING; i++)
						dac_waveform[0][i] = 0.0;
				}
			}
			if (operation == OP_DAC_CHAN2) {
				dac_chan2_waveform_selection = waveform;
				if (waveform == WF_NONE2) {
					for (i = 0; i < NR_OF_SAMPLES * SENDER_OVERSAMPLING; i++)
						dac_waveform[1][i] = 0.0;
				}
			}

			if (operation == OP_PRINT_WAVEFORM) {
				// trigger an uart waveform output
				debug_gui_waveform_selection = waveform;
			}
		}

		if (operation == OP_CONTROL_WORD)
		{
			int control_word = payload;
			switch (control_word){
			case CW_DEBUG_WAVEFORM_ENABLE:
				wf_debug_interface = WF_DEBUG_OUTPUT_ENABLED;
				break;
			case CW_DEBUG_WAVEFORM_DISABLE:
				wf_debug_interface = WF_DEBUG_OUTPUT_DISABLED;
				break;

			case CW_MICROPHONE_POSITION_DEBUG_OUTPUT_ENABLE:
				microphone_position_debug_state = MICROPHONE_POSITION_DEBUG_OUTPUT_ENABLED;
				break;
			case CW_MICROPHONE_POSITION_DEBUG_OUTPUT_DISABLE:
				microphone_position_debug_state = MICROPHONE_POSITION_DEBUG_OUTPUT_DISABLED;
				break;

			case CW_NRF_PACKET_CONTENT_DEBUG_OUTPUT_ENABLE:
				nrf_packet_content_debug_output = NRF_PACKET_CONTENT_DEBUG_OUTPUT_ENABLED;
				break;
			case CW_NRF_PACKET_CONTENT_DEBUG_OUTPUT_DISABLE:
				nrf_packet_content_debug_output = NRF_PACKET_CONTENT_DEBUG_OUTPUT_DISABLED;
				break;

			case CW_US_RANGING_MATRIX_DEBUG_OUTPUT_ENABLE:
				us_ranging_matrix_debug_output_state = US_RANGING_MATRIX_DEBUG_OUTPUT_ENABLED;
				break;
			case CW_US_RANGING_MATRIX_DEBUG_OUTPUT_DISABLE:
				us_ranging_matrix_debug_output_state = US_RANGING_MATRIX_DEBUG_OUTPUT_DISABLED;
				break;

			default:
				printf("character %i is not a valid control word\n", operation);
			}
		}

	}
	//</matlab debug gui interface>
}

/* _write gets called by printf, here used to redirect printf to buffered debug uart */
int _write(int file, char *data, int len) {
	int i;
	for (i = 0; i < len; i++) {
		//SER_PutChar(data[i]);

		serial_debug_write(data[i]);
	}
	return len;
}

void testRFFT(void) {

}

void init_timers_TIM3_TIM5(void) {
	//setup timer 5 for 320kHz interrupt (adc and dac trigger)
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->PSC = 1;
	TIM5->ARR = 225;
	TIM5->DIER = TIM_DIER_UIE;
	TIM5->CR1 = TIM_CR1_CEN;

	/*
	//Timer3 is used to generate a PWM on PC9 which enables the LEDs of the IR-lighthouse
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CCER = TIM_CCER_CC4E;
	TIM3->CR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0
			| TIM_CCMR2_OC4PE;
	TIM3->CR1 |= TIM_CR1_ARPE;
	TIM3->PSC = 1;
	TIM3->ARR = ((72.0f * 1000 * 1000) / 1.0f) / (24.36f * 1000.0f); // set frequency; opponent 2 beacon = 24.36 kHz; opponent 1 beacon = 23.6 kHz
	TIM3->CCR4 = TIM3->ARR * 0.5f;                     // set duty-cycle
	TIM3->CR1 |= TIM_CR1_CEN;
	*/

	NVIC_SetPriority(TIM5_IRQn, 3);
	NVIC_EnableIRQ(TIM5_IRQn);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
