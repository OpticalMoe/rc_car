/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
extern void Ws2812Extinguish(uint8_t start, uint8_t over);
extern void Ws2812ExtinguishAll(void);
extern void Ws2812bSet(int8_t Number, uint8_t Red, uint8_t Green, uint8_t Blue);
extern void Ws2812bSetOverlay(int8_t Number, uint8_t Red, uint8_t Green, uint8_t Blue);
extern void Ws2812bSetContinuous(int8_t start, uint8_t over, uint8_t Red, uint8_t Green, uint8_t Blue);
extern void Ws2812bSetOverlayContinuous(int8_t start, uint8_t over, uint8_t Red, uint8_t Green, uint8_t Blue);
extern void Ws2812bRefresh(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
