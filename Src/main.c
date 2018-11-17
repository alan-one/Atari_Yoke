/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
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
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define LED_BOUNDS 200
#define POLLING_DELAY 10
#define BOUND_BUFFER 100
#define LED_STARTUP_DELAY 50
#define NUMBER_LEDS 8
#define ADC_BUFFER 20
const uint16_t ledArrayPins[] = { L_TRIG_LED_Pin, R_TRIG_LED_Pin,
L_THUMB_LED_Pin, R_THUMB_LED_Pin, PITCH_LED_Pin, ROLL_LED_Pin,
STATUS_Pin, AUX_Pin };
GPIO_TypeDef * ledArrayPorts[] = { L_TRIG_LED_GPIO_Port, R_TRIG_LED_GPIO_Port,
L_THUMB_LED_GPIO_Port, R_THUMB_LED_GPIO_Port, PITCH_LED_GPIO_Port,
ROLL_LED_GPIO_Port, STATUS_GPIO_Port, AUX_GPIO_Port };

uint32_t ADC1ConvertedValues[ADC_BUFFER / 2];
volatile int32_t adcData1, adcData2;
volatile uint16_t adcCounter = 0;
volatile uint32_t yoke1 = 0;
volatile uint32_t yoke2 = 0;
volatile int ledBuffPos = 0;
uint8_t adcDataReady = 0;

typedef struct {
	uint8_t xbuff[2];
	uint8_t ybuff[2];
	uint8_t buttons;
} buffer;

struct yokeMaxMin {
	int32_t rollMax;
	int32_t rollMin;
	int32_t yawMax;
	int32_t yawMin;
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	int16_t lastX = 0;
	int16_t lastY = 0;

	buffer buff;
	buff.buttons = 0;
	//buff.buttons = 0;

	/*Setup initial yoke max and min ranges*/
	volatile struct yokeMaxMin bounds;
	bounds.rollMax = 2045;
	bounds.rollMin = 2050;
	bounds.yawMax = 2045;
	bounds.yawMin = 2050;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_USB_DEVICE_Init();

	/* USER CODE BEGIN 2 */

	//LED Startup sequence
	for (int i = 0; i < NUMBER_LEDS; i++) {
		HAL_GPIO_WritePin(ledArrayPorts[i], ledArrayPins[i], GPIO_PIN_RESET);
		HAL_Delay(LED_STARTUP_DELAY);
	}
	for (int i = NUMBER_LEDS; i > 0; i--) {
		HAL_GPIO_WritePin(ledArrayPorts[i - 1], ledArrayPins[i - 1],
				GPIO_PIN_SET);
		HAL_Delay(LED_STARTUP_DELAY);
	}

	for (int i = 0; i < NUMBER_LEDS; i++) {
		if (i == 0) {
			HAL_GPIO_WritePin(ledArrayPorts[i], ledArrayPins[i],
					GPIO_PIN_RESET);
		} else if (i == 1) {
			HAL_GPIO_WritePin(ledArrayPorts[i - 1], ledArrayPins[i - 1],
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(ledArrayPorts[i], ledArrayPins[i],
					GPIO_PIN_RESET);

		} else if (i == NUMBER_LEDS - 1) {
			HAL_GPIO_WritePin(ledArrayPorts[i - 1], ledArrayPins[i - 1],
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(ledArrayPorts[i], ledArrayPins[i], GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(ledArrayPorts[i - 1], ledArrayPins[i - 1],
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(ledArrayPorts[i], ledArrayPins[i],
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ledArrayPorts[i + 1], ledArrayPins[i + 1],
					GPIO_PIN_RESET);

		}
		HAL_Delay(LED_STARTUP_DELAY);
	}

	/*
	 for (int i = NUMBER_LEDS; i > 0; i--) {

	 if (i == 1) {
	 HAL_GPIO_WritePin(ledArrayPorts[i - 2], ledArrayPins[i - 2],
	 GPIO_PIN_SET);
	 } else {
	 HAL_GPIO_WritePin(ledArrayPorts[i - 1], ledArrayPins[i - 1],
	 GPIO_PIN_SET);
	 HAL_GPIO_WritePin(ledArrayPorts[i - 2], ledArrayPins[i - 2],
	 GPIO_PIN_SET);
	 }
	 HAL_Delay(LED_STARTUP_DELAY);
	 }
	 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (HAL_ADC_Start_DMA(&hadc, (uint32_t*) ADC1ConvertedValues,
		ADC_BUFFER) != HAL_OK) {
			return 0;
		}

		HAL_Delay(POLLING_DELAY);

		//S1_LEFT_TRIG_Pin
		if (HAL_GPIO_ReadPin(S1_LEFT_TRIG_GPIO_Port, S1_LEFT_TRIG_Pin) == 0) {
			buff.buttons |= 0b00000001;
			HAL_GPIO_WritePin(ledArrayPorts[0], ledArrayPins[0],
					GPIO_PIN_RESET);
		} else {
			buff.buttons &= ~(0b00000001);
			HAL_GPIO_WritePin(ledArrayPorts[0], ledArrayPins[0], GPIO_PIN_SET);
		}

		if (HAL_GPIO_ReadPin(S2_RIGHT_TRIG_GPIO_Port, S2_RIGHT_TRIG_Pin) == 0) {
			buff.buttons |= 0b00000010;
			HAL_GPIO_WritePin(ledArrayPorts[1], ledArrayPins[1],
					GPIO_PIN_RESET);
		} else {
			buff.buttons &= ~(0b00000010);
			HAL_GPIO_WritePin(ledArrayPorts[1], ledArrayPins[1], GPIO_PIN_SET);
		}
		if (HAL_GPIO_ReadPin(S3_LEFT_THUMB_GPIO_Port, S3_LEFT_THUMB_Pin) == 0) {
			buff.buttons |= 0b00000100;
			HAL_GPIO_WritePin(ledArrayPorts[2], ledArrayPins[2],
					GPIO_PIN_RESET);

		} else {
			buff.buttons &= ~(0b00000100);
			HAL_GPIO_WritePin(ledArrayPorts[2], ledArrayPins[2], GPIO_PIN_SET);
		}
		if (HAL_GPIO_ReadPin(S4_RIGHT_THUMB_GPIO_Port, S4_RIGHT_THUMB_Pin)
				== 0) {
			buff.buttons |= 0b00001000;
			HAL_GPIO_WritePin(ledArrayPorts[3], ledArrayPins[3],
					GPIO_PIN_RESET);

		} else {
			buff.buttons &= ~(0b00001000);
			HAL_GPIO_WritePin(ledArrayPorts[3], ledArrayPins[3], GPIO_PIN_SET);
		}

		/*check Roll boundaries and update if necessary*/
		if (yoke1 < bounds.rollMin) {
			bounds.rollMin = yoke1 - BOUND_BUFFER;
		} else if (yoke1 > bounds.rollMax) {
			bounds.rollMax = yoke1 + BOUND_BUFFER;
		}

		/*Check Yaw boundaries and update if necessary*/
		if (yoke2 < bounds.yawMin) {
			bounds.yawMin = yoke2 - BOUND_BUFFER;
		} else if (yoke2 > bounds.yawMax) {
			bounds.yawMax = yoke2 + BOUND_BUFFER;
		}

		/*Illuminate LED on boundary edges for roll and yaw*/
		if ((yoke1 < bounds.rollMin + LED_BOUNDS)
				|| (yoke1 > bounds.rollMax - LED_BOUNDS)) {
			HAL_GPIO_WritePin(ledArrayPorts[5], ledArrayPins[5],
					GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(ledArrayPorts[5], ledArrayPins[5], GPIO_PIN_SET);
		}
		if ((yoke2 < bounds.yawMin + LED_BOUNDS)
				|| (yoke2 > bounds.yawMax - LED_BOUNDS)) {
			HAL_GPIO_WritePin(ledArrayPorts[4], ledArrayPins[4],
					GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(ledArrayPorts[4], ledArrayPins[4], GPIO_PIN_SET);
		}

		//Split 12 bit data into two bytes
		//X-Axis
		int32_t x = ((yoke2 - bounds.yawMin) * 65536)
				/ (65536 - (65536 - bounds.yawMax));
		x -= 32767;

		buff.xbuff[0] = x & 0xFF;
		buff.xbuff[1] = x >> 8;
		//Y-Axis
		x = ((yoke1 - bounds.rollMin) * 65536)
				/ (65536 - (65536 - bounds.rollMax));
		x -= 32767;

		buff.ybuff[0] = x & 0xFF;
		buff.ybuff[1] = x >> 8;
		if(adcDataReady == 1){
			USBD_HID_SendReport(&hUsbDeviceFS, &buff, 5);
		}else{
			if (HAL_ADC_Start_DMA(&hadc, (uint32_t*) ADC1ConvertedValues,
					ADC_BUFFER) != HAL_OK) {
						return 0;
				}
			adcDataReady = 0;
		}
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PA0   ------> SharedAnalog_PA0
 PA1   ------> SharedAnalog_PA1
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			L_TRIG_LED_Pin | R_TRIG_LED_Pin | L_THUMB_LED_Pin | R_THUMB_LED_Pin
					| PITCH_LED_Pin | ROLL_LED_Pin | STATUS_Pin | AUX_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pins : PITCH_Pin ROLL_Pin */
	GPIO_InitStruct.Pin = PITCH_Pin | ROLL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : S1_LEFT_TRIG_Pin S2_RIGHT_TRIG_Pin S3_LEFT_THUMB_Pin S4_RIGHT_THUMB_Pin */
	GPIO_InitStruct.Pin = S1_LEFT_TRIG_Pin | S2_RIGHT_TRIG_Pin
			| S3_LEFT_THUMB_Pin | S4_RIGHT_THUMB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : L_TRIG_LED_Pin R_TRIG_LED_Pin L_THUMB_LED_Pin R_THUMB_LED_Pin
	 PITCH_LED_Pin ROLL_LED_Pin STATUS_Pin AUX_Pin */
	GPIO_InitStruct.Pin = L_TRIG_LED_Pin | R_TRIG_LED_Pin | L_THUMB_LED_Pin
			| R_THUMB_LED_Pin | PITCH_LED_Pin | ROLL_LED_Pin | STATUS_Pin
			| AUX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : DIP_4_Pin DIP_3_Pin DIP_2_Pin DIP_1_Pin */
	GPIO_InitStruct.Pin = DIP_4_Pin | DIP_3_Pin | DIP_2_Pin | DIP_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {

	//yoke1 = 0;
	//yoke2 = 0;
	for (int i = 0; i < ADC_BUFFER / 2; i++) {
		yoke1 += ADC1ConvertedValues[i] >> 16;
		yoke2 += ADC1ConvertedValues[i] & 0xFFFF;
	}

	yoke1 = yoke1 / ((ADC_BUFFER / 2) + 1);
	yoke2 = yoke2 / ((ADC_BUFFER / 2) + 1);
	HAL_ADC_Stop_DMA(&hadc);
	adcDataReady = 1;

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
