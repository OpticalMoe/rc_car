/**
  ******************************************************************************
  * File Name          : SPI.c
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
//WS2812B
uint16_t spiLightNumberMax = 50, spiLightNumber = 28;
uint8_t ws2812bResetDataLen = 60;
//9 * 50 + 60 + 1;(+1 防止最后一个字节发不完):50颗灯 * 9颜色数据长度/灯 + 60复位数据长度 + 1
uint8_t ws2812bData[451 + 60] = {0};				
uint8_t ws2812bResetData[100] = {0};

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration    
    PB3     ------> SPI1_SCK
    PB5     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = Buzzer_Pin|WS2812B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_SPI1_ENABLE();

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PB3     ------> SPI1_SCK
    PB5     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, Buzzer_Pin|WS2812B_Pin);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
//SPI - WS2812B
/**
  * @brief  Ws2812b 指定区间灭灯
  * @retval 从1开始计算Ws2812b位置
  */
void Ws2812Extinguish(uint8_t start, uint8_t over)
{
	uint8_t i, j;
	for(i = (start - 1); i <= (over - 1); i++)		//灯
	{
		for(j = 0; j < 3; j++)							//3 个颜色
		{		
			ws2812bData[ws2812bResetDataLen + i * 9 + j * 3 + 1] = 0x92;	//全为0的 0码
			ws2812bData[ws2812bResetDataLen + i * 9 + j * 3 + 2] = 0x49;
			ws2812bData[ws2812bResetDataLen + i * 9 + j * 3 + 3] = 0x24;
		}
	}
}

/**
  * @brief  Ws2812b 所有灯 灭
  * @retval 从1开始计算Ws2812b位置
  */
void Ws2812ExtinguishAll(void)
{
	Ws2812Extinguish(1, spiLightNumber + 1);
}

/**
  * @brief  设定Ws2812b颜色，不叠加
  * @retval 从1开始计算Ws2812b位置
  */
void Ws2812bSet(int8_t Number, uint8_t Red, uint8_t Green, uint8_t Blue)
{
	uint8_t color;
	Number -= 1;		//由从1开始计数 -> 从0开始计数
	//清空当前颜色
	for(color = 0; color < 3; color++)							//3 个颜色
	{		
		ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + 1] = 0x92;	//全为0的 0码
		ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + 2] = 0x49;
		ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + 3] = 0x24;
	}
	Ws2812bSetOverlay(Number + 1, Red, Green, Blue);
}

/**
  * @brief  设定Ws2812b颜色，叠加
  * @retval 从1开始计算Ws2812b位置
  */
void Ws2812bSetOverlay(int8_t Number, uint8_t Red, uint8_t Green, uint8_t Blue)
{
	uint8_t color, bit;
	uint8_t Color[3] = {Green, Red, Blue};
	if(Number < 1) return;		//小于零，无意义，退出
	Number -= 1;		//由从1开始计数 -> 从0开始计数

	for(color = 0; color < 3; color++)			//三个颜色
	{
		bit = 1;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x40;
			Color[color] <<= 1;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x08;
			Color[color] <<= 1;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x01;
			Color[color] <<= 1;
		
		bit = 2;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x20;
			Color[color] <<= 1;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x04;
			Color[color] <<= 1;
		
		bit = 3;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x80;
			Color[color] <<= 1;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x10;
			Color[color] <<= 1;
		if(Color[color] & 0x80)
			ws2812bData[ws2812bResetDataLen + Number * 9 + color * 3 + bit] |= 0x20;		
	}
}

/**
  * @brief  设定指定区间Ws2812b颜色，不叠加
  * @retval 从1开始计算Ws2812b位置
  */
void Ws2812bSetContinuous(int8_t start, uint8_t over, uint8_t Red, uint8_t Green, uint8_t Blue)
{
	uint8_t i;
	for(i = start; i <= over; i++)
		Ws2812bSet(i, Red, Green, Blue);
}

/**
  * @brief  设定指定区间Ws2812b颜色，叠加
  * @retval 从1开始计算Ws2812b位置
  */
void Ws2812bSetOverlayContinuous(int8_t start, uint8_t over, uint8_t Red, uint8_t Green, uint8_t Blue)
{
	uint8_t i;
	for(i = start; i <= over; i++)
		Ws2812bSetOverlay(i, Red, Green, Blue);
}

/**
  * @brief  刷新Ws2812b灯带
  * @retval None
  */
void Ws2812bRefresh(void)
{		
//	HAL_SPI_Transmit_IT(&hspi1, ws2812bResetData, 100);						//537us低电平 复位
	HAL_SPI_Transmit_IT(&hspi1, ws2812bData, ws2812bResetDataLen + spiLightNumber * 9 + 1);		//灯带数据
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
