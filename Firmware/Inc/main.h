/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DEBUG 1

#if DEBUG
//PID Debug
	extern int16_t Input, Output, SetPoint, DebugSpeed;
	extern int16_t Et, LastErr, LastTwoErr;
	extern int16_t Voltage, Current;
#endif

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Direction_Pin GPIO_PIN_0
#define Direction_GPIO_Port GPIOA
#define Pitch_Pin GPIO_PIN_1
#define Pitch_GPIO_Port GPIOA
#define Yaw_Pin GPIO_PIN_2
#define Yaw_GPIO_Port GPIOA
#define Roll_Pin GPIO_PIN_3
#define Roll_GPIO_Port GPIOA
#define Current_Pin GPIO_PIN_4
#define Current_GPIO_Port GPIOA
#define Voltage_Pin GPIO_PIN_5
#define Voltage_GPIO_Port GPIOA
#define MotorLift_Pin GPIO_PIN_6
#define MotorLift_GPIO_Port GPIOA
#define MotorRight_Pin GPIO_PIN_7
#define MotorRight_GPIO_Port GPIOA
#define MotorSD_Pin GPIO_PIN_0
#define MotorSD_GPIO_Port GPIOB
#define HeadLight_Pin GPIO_PIN_1
#define HeadLight_GPIO_Port GPIOB
#define OutPower_Pin GPIO_PIN_2
#define OutPower_GPIO_Port GPIOB
#define Sign_Pin GPIO_PIN_10
#define Sign_GPIO_Port GPIOB
#define DBUS_Pin GPIO_PIN_11
#define DBUS_GPIO_Port GPIOB
#define SW4_Pin GPIO_PIN_12
#define SW4_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_13
#define SW3_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_14
#define SW2_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_15
#define SW1_GPIO_Port GPIOB
#define MpuInt_Pin GPIO_PIN_8
#define MpuInt_GPIO_Port GPIOA
#define UsartTxd_Pin GPIO_PIN_9
#define UsartTxd_GPIO_Port GPIOA
#define UsartRxd_Pin GPIO_PIN_10
#define UsartRxd_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_3
#define Buzzer_GPIO_Port GPIOB
#define WS2812B_Pin GPIO_PIN_5
#define WS2812B_GPIO_Port GPIOB
#define EncoderA_Pin GPIO_PIN_6
#define EncoderA_GPIO_Port GPIOB
#define EncoderB_Pin GPIO_PIN_7
#define EncoderB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
