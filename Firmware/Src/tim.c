/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "iwdg.h"

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);

}
/* TIM4 init function */
void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_encoderHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration    
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2 
    */
    GPIO_InitStruct.Pin = EncoderA_Pin|EncoderB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration    
    PA0-WKUP     ------> TIM2_CH1
    PA1     ------> TIM2_CH2
    PA2     ------> TIM2_CH3
    PA3     ------> TIM2_CH4 
    */
    GPIO_InitStruct.Pin = Direction_Pin|Pitch_Pin|Yaw_Pin|Roll_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3 
    */
    GPIO_InitStruct.Pin = MotorLift_Pin|MotorRight_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MotorSD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MotorSD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  if(tim_encoderHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
  
    /**TIM4 GPIO Configuration    
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2 
    */
    HAL_GPIO_DeInit(GPIOB, EncoderA_Pin|EncoderB_Pin);

  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  电机自检
  * @param  None
  * @retval None
  */
uint16_t MotorSelfCheck(void)
{
	uint16_t checkResult = 0;
	uint16_t checkEncoder = 0;
	uint16_t checkOneTime = 20;
	
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	
	//编码器
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);		
	__HAL_TIM_SET_COUNTER(&htim4, 30000);
	
	//第一轮， 正转
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 300);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_Delay(checkOneTime);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	
	checkEncoder = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim4)) - 30000;
	if(checkEncoder > 10)
		checkResult++;
	
	//第二轮， 反转
	HAL_Delay(100);
	__HAL_TIM_SET_COUNTER(&htim4, 30000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 300);
	HAL_Delay(checkOneTime);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	
	checkEncoder = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim4)) - 30000;
	if(checkEncoder > 10)
		checkResult++;
		
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	return checkResult;
}

/**
  * @brief  电机上电奏乐， 自检成功
  * @param  None
  * @retval None
  */
void MotorMusicSuccess(void)
{	
	float musicPeriod = 0;				//发声频率
	uint32_t musicFrequency = 0;	//定时器装载值
	float musicVolume = 0.5;			//音量
	float musicOffset = 0.87;			//偏移，解决奏乐时电机转动
	
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	
	//安全起见，先停止PWM
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	
	//初始化MotorRight极性为反向
	TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
	
	//C1Up:525
	musicPeriod = 519;
	musicFrequency = (uint16_t)((8000000 / musicPeriod) + 0.5 - 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, musicFrequency);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, musicFrequency * musicVolume);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	
	//A4:589
	musicPeriod = 586;
	musicFrequency = (uint16_t)((8000000 / musicPeriod) + 0.5 - 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, musicFrequency);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, musicFrequency * musicVolume);
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	
	//G1Up:786
	musicPeriod = 780;
	musicFrequency = (uint16_t)((8000000 / musicPeriod) + 0.5 - 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, musicFrequency);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, musicFrequency * musicVolume);
	HAL_Delay(300);
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}

/**
  * @brief  电机上电奏乐， 自检出错
  * @param  None
  * @retval None
  */
void MotorMusicError(void)
{	
	float musicPeriod = 0;				//发声频率
	uint32_t musicFrequency = 0;	//定时器装载值
	float musicVolume = 0.5;			//音量
	float musicOffset = 0.87;		//偏移，解决奏乐时电机转动
	
	//安全起见，先停止PWM
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	
	//初始化MotorRight极性为反向
	TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
	
	//C1Up:525
	musicPeriod = 786;
	musicFrequency = (uint16_t)((8000000 / musicPeriod) + 0.5 - 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, musicFrequency);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, musicFrequency * musicVolume);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_Delay(250);
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	HAL_Delay(250);
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	
		//G1Up:786
	musicPeriod = 530;
	musicFrequency = (uint16_t)((8000000 / musicPeriod) + 0.5 - 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, musicFrequency);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, musicFrequency * musicVolume * 0.5 * musicOffset);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, musicFrequency * musicVolume);
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	HAL_Delay(200);
	HAL_IWDG_Refresh(&hiwdg);			//喂狗， 500ms内
	
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}

/**
  * @brief  电机方向引脚 重新初始化为普通GPIO
  * @param  None
  * @retval None
  */
void MotorGpioReInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MotorLift_GPIO_Port, MotorLift_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorRight_GPIO_Port, MotorRight_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MotorLift_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MotorLift_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = MotorRight_Pin;
  HAL_GPIO_Init(MotorRight_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
