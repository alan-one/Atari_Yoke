/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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

#ifndef __MAIN_H
#define __MAIN_H

#include "stdint.h"

#define PITCH_Pin GPIO_PIN_0
#define PITCH_GPIO_Port GPIOA
#define ROLL_Pin GPIO_PIN_1
#define ROLL_GPIO_Port GPIOA
#define S1_LEFT_TRIG_Pin GPIO_PIN_2
#define S1_LEFT_TRIG_GPIO_Port GPIOA
#define S2_RIGHT_TRIG_Pin GPIO_PIN_3
#define S2_RIGHT_TRIG_GPIO_Port GPIOA
#define S3_LEFT_THUMB_Pin GPIO_PIN_4
#define S3_LEFT_THUMB_GPIO_Port GPIOA
#define S4_RIGHT_THUMB_Pin GPIO_PIN_5
#define S4_RIGHT_THUMB_GPIO_Port GPIOA
#define L_TRIG_LED_Pin GPIO_PIN_0
#define L_TRIG_LED_GPIO_Port GPIOB
#define R_TRIG_LED_Pin GPIO_PIN_1
#define R_TRIG_LED_GPIO_Port GPIOB
#define L_THUMB_LED_Pin GPIO_PIN_10
#define L_THUMB_LED_GPIO_Port GPIOB
#define R_THUMB_LED_Pin GPIO_PIN_11
#define R_THUMB_LED_GPIO_Port GPIOB
#define PITCH_LED_Pin GPIO_PIN_12
#define PITCH_LED_GPIO_Port GPIOB
#define ROLL_LED_Pin GPIO_PIN_13
#define ROLL_LED_GPIO_Port GPIOB
#define STATUS_Pin GPIO_PIN_14
#define STATUS_GPIO_Port GPIOB
#define AUX_Pin GPIO_PIN_15
#define AUX_GPIO_Port GPIOB
#define DIP_4_Pin GPIO_PIN_4
#define DIP_4_GPIO_Port GPIOB
#define DIP_3_Pin GPIO_PIN_5
#define DIP_3_GPIO_Port GPIOB
#define DIP_2_Pin GPIO_PIN_6
#define DIP_2_GPIO_Port GPIOB
#define DIP_1_Pin GPIO_PIN_7
#define DIP_1_GPIO_Port GPIOB

#define HSE_VALUE 8000000 //Set External Crystal Value

#define POLLING_DELAY 0
#define BOUND_BUFFER 100
#define LED_STARTUP_DELAY 50
#define NUMBER_LEDS 8
#define ADC_BUFFER 100

//Max counts for blinky LEDs. Controls blink rate.
#define STATUS_COUNT_MAX 100000
#define STATUS_COUNT_ON 2000
#define AUX_COUNT_MAX 100000
#define AUX_COUNT_ON 50000

//Max count for toggling modes.
#define MODE_COUNT_MAX 300000

//Value to add/subtract for dead spot compensation.
#define X_DEADSPOT_LO 410    //10% of max value.
#define X_DEADSPOT_HI 410    //10% of max value.
#define Y_DEADSPOT_LO 410    //10% of max value.
#define Y_DEADSPOT_HI 410    //10% of max value.

#define X_ADC_LB   0         //Buffer index for X-axis ADC values, lower byte.
#define X_ADC_UB   1         //Buffer index for X-axis ADC values, upper byte.
#define Y_ADC_LB   2         //Buffer index for Y-axis ADC values, lower byte.
#define Y_ADC_UB   3         //Buffer index for Y-axis ADC values, upper byte.
#define BUTTONS    4         //Buffer index for buttons.

#define ADC_UPPER_BOUND 3686  //90% of max value.
#define ADC_LOWER_BOUND 410   //10% of max value.

#define TRUE 1
#define FALSE 0

//High, middle and low values for X and Y ADC values.
#define X_HIGH 4095
#define Y_HIGH 4095
#define X_MIDDLE 2047
#define Y_MIDDLE 2047
#define X_LOW 0
#define Y_LOW 0

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC_Init(void);

void do_led_array(uint8_t); //Turn on an array of LEDs
void init_leds();           //Fancy initialization LED blinky blinky.
void run_leds();            //Update the LEDS during normal use.

void _Error_Handler(char *, int);
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif
