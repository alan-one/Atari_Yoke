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

#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "usbd_hid.h"

const uint16_t ledArrayPins[] =
{
    L_TRIG_LED_Pin, R_TRIG_LED_Pin,
    L_THUMB_LED_Pin, R_THUMB_LED_Pin,
	PITCH_LED_Pin, ROLL_LED_Pin,
    STATUS_Pin, AUX_Pin
};

GPIO_TypeDef *ledArrayPorts[] =
{
    L_TRIG_LED_GPIO_Port, R_TRIG_LED_GPIO_Port,
    L_THUMB_LED_GPIO_Port, R_THUMB_LED_GPIO_Port,
	PITCH_LED_GPIO_Port, ROLL_LED_GPIO_Port,
	STATUS_GPIO_Port, AUX_GPIO_Port
};

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

//Buffer that holds the USB data.
uint8_t buff[5] = {0, 0, 0, 0, 0};

//Buffer of ADC values.
uint32_t ADC1ConvertedValues[ADC_BUFFER / 2];

//Averaged buffer values.
uint32_t averageX = 0;
uint32_t averageY = 0;

//Run LED initialization if true.
uint8_t do_led_init = 1;

//LED initialization pattern.
uint8_t led_pattern[] =
{
    0x18, 0x24, 0x42, 0x81, 0xC3, 0xE7, 0xFF, 0x7E,
	0x3C, 0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18,
	0x30, 0x60, 0xC0
};

//Length of the array above.
int pat_len = 19;

int main(void) {

	//Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	//Configure the system clock.
	SystemClock_Config();

	//Initialize all configured peripherals.
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_USB_DEVICE_Init();

	//Start the DMA service and exit if there is a problem.
	if (HAL_ADC_Start_DMA(&hadc, (uint32_t*) ADC1ConvertedValues, ADC_BUFFER) != HAL_OK)
	{
		return 0;
	}

	while (1)
	{
		//Check it device was just powered up.
		if(do_led_init)
		{
			init_leds();
		}
		else
		{
			run_leds();
		}

		//Send out the USB data.
		USBD_HID_SendReport(&hUsbDeviceFS, buff, 5);
	}
}

//Startup LED pattern state machine.
void init_leds()
{
    static int this_index = 0;

    //Display LED array pattern.
    do_led_array(led_pattern[this_index++]);
	HAL_Delay(LED_STARTUP_DELAY);

	//If the end of the array has been reached, init_leds is finished.
	if (this_index >= pat_len)
	{
	    do_led_init = 0;
	}
}

//Turn on an array of LEDs
void do_led_array(uint8_t leds)
{
	int index = 0;

	//Shift through all the LED positions one at a time.
	for (uint8_t i = 1; i != 0; i <<= 1)
	{
        if (leds & i)
        {
        	HAL_GPIO_WritePin(ledArrayPorts[index], ledArrayPins[index], GPIO_PIN_RESET);
        }
        else
        {
        	HAL_GPIO_WritePin(ledArrayPorts[index], ledArrayPins[index], GPIO_PIN_SET);
        }

        //Move to next index in ledArrayPorts.
        index++;
	}
}

//Update the LEDS during normal use.
void run_leds()
{
	//Left trigger button.
	if (HAL_GPIO_ReadPin(S1_LEFT_TRIG_GPIO_Port, S1_LEFT_TRIG_Pin) == 0)
	{
		buff[BUTTONS] |= 0b00000001;
		HAL_GPIO_WritePin(ledArrayPorts[0], ledArrayPins[0], GPIO_PIN_RESET);
	}
	else
	{
		buff[BUTTONS] &= ~(0b00000001);
		HAL_GPIO_WritePin(ledArrayPorts[0], ledArrayPins[0], GPIO_PIN_SET);
	}

	//Right trigger button.
	if (HAL_GPIO_ReadPin(S2_RIGHT_TRIG_GPIO_Port, S2_RIGHT_TRIG_Pin) == 0)
	{

		buff[BUTTONS] |= 0b00000010;
		HAL_GPIO_WritePin(ledArrayPorts[1], ledArrayPins[1], GPIO_PIN_RESET);
	}
	else
	{
		buff[BUTTONS] &= ~(0b00000010);
		HAL_GPIO_WritePin(ledArrayPorts[1], ledArrayPins[1], GPIO_PIN_SET);
	}

	//Left thumb button.
	if (HAL_GPIO_ReadPin(S3_LEFT_THUMB_GPIO_Port, S3_LEFT_THUMB_Pin) == 0)
	{
		buff[BUTTONS] |= 0b00000100;
		HAL_GPIO_WritePin(ledArrayPorts[2], ledArrayPins[2], GPIO_PIN_RESET);
	}
	else
	{
		buff[BUTTONS] &= ~(0b00000100);
		HAL_GPIO_WritePin(ledArrayPorts[2], ledArrayPins[2], GPIO_PIN_SET);
	}

	//Right thumb button.
	if (HAL_GPIO_ReadPin(S4_RIGHT_THUMB_GPIO_Port, S4_RIGHT_THUMB_Pin) == 0)
	{
		buff[BUTTONS] |= 0b00001000;
		HAL_GPIO_WritePin(ledArrayPorts[3], ledArrayPins[3], GPIO_PIN_RESET);
	}
	else
	{
		buff[BUTTONS] &= ~(0b00001000);
		HAL_GPIO_WritePin(ledArrayPorts[3], ledArrayPins[3], GPIO_PIN_SET);
	}

	//Illuminate LEDs when at upper and lower 10% extents of the ADC values.
    if ((averageX > ADC_UPPER_BOUND) || (averageX < ADC_LOWER_BOUND))
	{
		HAL_GPIO_WritePin(ledArrayPorts[5], ledArrayPins[5], GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(ledArrayPorts[5], ledArrayPins[5], GPIO_PIN_SET);
	}

	if ((averageY > ADC_UPPER_BOUND) || (averageY < ADC_LOWER_BOUND))
	{
		HAL_GPIO_WritePin(ledArrayPorts[4], ledArrayPins[4], GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(ledArrayPorts[4], ledArrayPins[4], GPIO_PIN_SET);
	}

	HAL_GPIO_TogglePin(STATUS_GPIO_Port, STATUS_Pin);
}

//System Clock Configuration
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	//Initializes the CPU, AHB and APB busses clocks
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Initializes the CPU, AHB and APB busses clocks
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
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

void MX_ADC_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	//Configure the global features of the ADC:
	//Clock, Resolution, Data Alignment and number of conversions.
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.SamplingTimeCommon = ADC_SAMPLETIME_28CYCLES_5;

	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Configure for the selected ADC regular channel to be converted.
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	//Configure for the selected ADC regular channel to be converted.
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

void MX_DMA_Init(void)
{
	//DMA controller clock enable
	__HAL_RCC_DMA1_CLK_ENABLE();

	//DMA interrupt init - DMA1_Channel1_IRQn interrupt configuration
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	//GPIO Ports Clock Enable
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, L_TRIG_LED_Pin | R_TRIG_LED_Pin | L_THUMB_LED_Pin |
        R_THUMB_LED_Pin | PITCH_LED_Pin | ROLL_LED_Pin | STATUS_Pin | AUX_Pin, GPIO_PIN_SET);

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

	//Configure GPIO pins:
	//L_TRIG_LED_Pin, L_THUMB_LED_Pin,
	//R_TRIG_LED_Pin, R_THUMB_LED_Pin,
	//PITCH_LED_Pin,  ROLL_LED_Pin,
	//STATUS_Pin,     AUX_Pin.
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

//Get ADC samples, average them and put them in the USB buffer.
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	averageX = 0;

	//Average ADC samples for both X and Y to remove any noise.
	for (int i = 0; i < ADC_BUFFER / 2; i++)
	{
		averageX += (ADC1ConvertedValues[i] >> 16) & 0x0FFF;
		averageY += ADC1ConvertedValues[i] & 0x0FFF;
	}

	//Assign the ADC values to the USB output buffer.
	averageX /= (ADC_BUFFER / 2);
	averageY /= (ADC_BUFFER / 2);

	buff[X_ADC_LB] = averageX;
	buff[X_ADC_UB] = averageX >> 8;
	buff[Y_ADC_LB] = averageY;
	buff[Y_ADC_UB] = averageY >> 8;

	//Toggle the AUX LED if not in LED init mode.
	if(!do_led_init)
	{
	    HAL_GPIO_TogglePin(AUX_GPIO_Port, AUX_Pin);
	}
}

//This function is executed in case of error occurrence.
void _Error_Handler(char * file, int line)
{
	//User can add his own implementation to report the HAL error return state.
	while (1){}
}
