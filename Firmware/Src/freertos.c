/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId runawayMonitorHandle;
osThreadId motorPIDHandle;
osThreadId remoteDBUSHandle;
osThreadId remoteUARTHandle;
osThreadId lightSPIHandle;
osThreadId gimbalHandle;
osThreadId featureSelectHandle;
osThreadId statusReturnHandle;
osThreadId voltageMonitorHandle;
osMessageQId carSetSpeedHandle;
osMessageQId carSetDirectionHandle;
osMessageQId carLightHandle;
osMessageQId carBuzzerHandle;
osMessageQId carControlModeHandle;
osMessageQId carControlGearHandle;
osMessageQId carRunawayHandle;
osMessageQId remoteSpeedHandle;
osMessageQId remoteDirectionHandle;
osMessageQId carSpiLightHandle;
osMessageQId carBreakHandle;
osMessageQId carVoltageHandle;
osMessageQId carCurrentHandle;
osMessageQId carRealSpeedHandle;
osSemaphoreId succRemoteUARTHandle;
osSemaphoreId succRemoteDBUSHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartRunawayMonitor(void const * argument);
void StartMotorPID(void const * argument);
void StartRemoteDBUS(void const * argument);
void StartRemoteUART(void const * argument);
void StartLightSPI(void const * argument);
void StartGimbal(void const * argument);
void StartFeatureSelect(void const * argument);
void StartStatusReturn(void const * argument);
void StartVoltageMonitor(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of succRemoteUART */
  osSemaphoreDef(succRemoteUART);
  succRemoteUARTHandle = osSemaphoreCreate(osSemaphore(succRemoteUART), 1);

  /* definition and creation of succRemoteDBUS */
  osSemaphoreDef(succRemoteDBUS);
  succRemoteDBUSHandle = osSemaphoreCreate(osSemaphore(succRemoteDBUS), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of carSetSpeed */
  osMessageQDef(carSetSpeed, 1, float);
  carSetSpeedHandle = osMessageCreate(osMessageQ(carSetSpeed), NULL);

  /* definition and creation of carSetDirection */
  osMessageQDef(carSetDirection, 1, float);
  carSetDirectionHandle = osMessageCreate(osMessageQ(carSetDirection), NULL);

  /* definition and creation of carLight */
  osMessageQDef(carLight, 1, uint16_t);
  carLightHandle = osMessageCreate(osMessageQ(carLight), NULL);

  /* definition and creation of carBuzzer */
  osMessageQDef(carBuzzer, 1, uint16_t);
  carBuzzerHandle = osMessageCreate(osMessageQ(carBuzzer), NULL);

  /* definition and creation of carControlMode */
  osMessageQDef(carControlMode, 1, uint16_t);
  carControlModeHandle = osMessageCreate(osMessageQ(carControlMode), NULL);

  /* definition and creation of carControlGear */
  osMessageQDef(carControlGear, 1, uint16_t);
  carControlGearHandle = osMessageCreate(osMessageQ(carControlGear), NULL);

  /* definition and creation of carRunaway */
  osMessageQDef(carRunaway, 1, uint16_t);
  carRunawayHandle = osMessageCreate(osMessageQ(carRunaway), NULL);

  /* definition and creation of remoteSpeed */
  osMessageQDef(remoteSpeed, 1, float);
  remoteSpeedHandle = osMessageCreate(osMessageQ(remoteSpeed), NULL);

  /* definition and creation of remoteDirection */
  osMessageQDef(remoteDirection, 1, float);
  remoteDirectionHandle = osMessageCreate(osMessageQ(remoteDirection), NULL);

  /* definition and creation of carSpiLight */
  osMessageQDef(carSpiLight, 1, uint16_t);
  carSpiLightHandle = osMessageCreate(osMessageQ(carSpiLight), NULL);

  /* definition and creation of carBreak */
  osMessageQDef(carBreak, 1, uint8_t);
  carBreakHandle = osMessageCreate(osMessageQ(carBreak), NULL);

  /* definition and creation of carVoltage */
  osMessageQDef(carVoltage, 1, float);
  carVoltageHandle = osMessageCreate(osMessageQ(carVoltage), NULL);

  /* definition and creation of carCurrent */
  osMessageQDef(carCurrent, 1, float);
  carCurrentHandle = osMessageCreate(osMessageQ(carCurrent), NULL);

  /* definition and creation of carRealSpeed */
  osMessageQDef(carRealSpeed, 1, float);
  carRealSpeedHandle = osMessageCreate(osMessageQ(carRealSpeed), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of runawayMonitor */
  osThreadDef(runawayMonitor, StartRunawayMonitor, osPriorityAboveNormal, 0, 128);
  runawayMonitorHandle = osThreadCreate(osThread(runawayMonitor), NULL);

  /* definition and creation of motorPID */
  osThreadDef(motorPID, StartMotorPID, osPriorityHigh, 0, 128);
  motorPIDHandle = osThreadCreate(osThread(motorPID), NULL);

  /* definition and creation of remoteDBUS */
  osThreadDef(remoteDBUS, StartRemoteDBUS, osPriorityNormal, 0, 128);
  remoteDBUSHandle = osThreadCreate(osThread(remoteDBUS), NULL);

  /* definition and creation of remoteUART */
  osThreadDef(remoteUART, StartRemoteUART, osPriorityNormal, 0, 128);
  remoteUARTHandle = osThreadCreate(osThread(remoteUART), NULL);

  /* definition and creation of lightSPI */
  osThreadDef(lightSPI, StartLightSPI, osPriorityLow, 0, 128);
  lightSPIHandle = osThreadCreate(osThread(lightSPI), NULL);

  /* definition and creation of gimbal */
  osThreadDef(gimbal, StartGimbal, osPriorityIdle, 0, 128);
  gimbalHandle = osThreadCreate(osThread(gimbal), NULL);

  /* definition and creation of featureSelect */
  osThreadDef(featureSelect, StartFeatureSelect, osPriorityLow, 0, 128);
  featureSelectHandle = osThreadCreate(osThread(featureSelect), NULL);

  /* definition and creation of statusReturn */
  osThreadDef(statusReturn, StartStatusReturn, osPriorityLow, 0, 128);
  statusReturnHandle = osThreadCreate(osThread(statusReturn), NULL);

  /* definition and creation of voltageMonitor */
  osThreadDef(voltageMonitor, StartVoltageMonitor, osPriorityLow, 0, 128);
  voltageMonitorHandle = osThreadCreate(osThread(voltageMonitor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	//喂狗
	HAL_IWDG_Refresh(&hiwdg);
	
	//挂起任务		
	vTaskSuspend(remoteDBUSHandle);			//挂起DBUS遥控任务
	vTaskSuspend(remoteUARTHandle);			//挂起UART遥控任务
	vTaskSuspend(lightSPIHandle);				//挂起Ws2812b灯带任务
	vTaskSuspend(gimbalHandle);					//挂起云台任务
			
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartRunawayMonitor */
/**
  * @brief  Function implementing the runawayMonitor thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartRunawayMonitor */
void StartRunawayMonitor(void const * argument)
{
  /* USER CODE BEGIN StartRunawayMonitor */
	//失控保护
	//超时 500ms + 10ms(冗余) 未收到有效数据，则执行失控保护
	//失控保护动作：速度 0m/s，方向回中，触发失控 标志
  float IWDG_CarSetSpeed = 0, IWDG_CarSetDirection = 0;
	uint16_t IWDG_CarDirectionZero = 1500;
	uint16_t IWDG_CarRunaway = 1;
	
	//舵机 方向
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);						//Direction
	
	//上电回中
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, IWDG_CarDirectionZero - IWDG_CarSetDirection);
	
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(Sign_GPIO_Port, Sign_Pin);
		//失控保护
		if(xQueueReceive(remoteSpeedHandle, &IWDG_CarSetSpeed, 510) == pdFALSE)
		{
			IWDG_CarSetSpeed = 0;
			IWDG_CarSetDirection = 0;
			IWDG_CarRunaway = 1;
		}
		else
			if(xQueueReceive(remoteDirectionHandle, &IWDG_CarSetDirection, 10) == pdFALSE)
			{
				IWDG_CarSetSpeed = 0;
				IWDG_CarSetDirection = 0;
				IWDG_CarRunaway = 1;
			}
			else
				IWDG_CarRunaway = 0;
		
		//方向 限幅 & 输出
		IWDG_CarSetDirection = IWDG_CarSetDirection < -330 ? -330 : IWDG_CarSetDirection;
		IWDG_CarSetDirection = IWDG_CarSetDirection > 330 ? 330 : IWDG_CarSetDirection;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, IWDG_CarDirectionZero - IWDG_CarSetDirection);
		
		//车速 限幅
		IWDG_CarSetSpeed = IWDG_CarSetSpeed < -5.0 ? -5.0 : IWDG_CarSetSpeed;
		IWDG_CarSetSpeed = IWDG_CarSetSpeed > 5.0 ? 5.0 : IWDG_CarSetSpeed;
		
		//覆写 消息队列
		xQueueOverwrite(carSetSpeedHandle, &IWDG_CarSetSpeed);
		xQueueOverwrite(carSetDirectionHandle, &IWDG_CarSetDirection);
		xQueueOverwrite(carRunawayHandle, &IWDG_CarRunaway);
  }
  /* USER CODE END StartRunawayMonitor */
}

/* USER CODE BEGIN Header_StartMotorPID */
/**
* @brief Function implementing the motorPID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorPID */
void StartMotorPID(void const * argument)
{
  /* USER CODE BEGIN StartMotorPID */
	//PID 运算
	//增量式 PID，周期 20ms
	int16_t pidInput = 0, pidSetPoint = 0;
	int16_t pidEt = 0, pidLastErr = 0, pidLastTwoErr = 0;
	double pidOutput = 0.0;
	float pidKp = 0, pidKi = 0, pidKd  = 0;
	uint16_t pidMotorOutput = 0;
	float pidCarRealSpeed = 0;
	
	//刹车
	int16_t pidSetPointLast = 0;
	uint8_t pidCarBreak = 0;
	uint8_t pidCarBreakRemove = 0;
	uint32_t pidCarStopTime = 0;
	
	//车运行数据
	float pidCarSetSpeed = 0;
	
	//精准延时
	TickType_t pidLastTick = xTaskGetTickCount();
	
	//电机自检
	if(MotorSelfCheck())
		MotorMusicSuccess();
	else 
		MotorMusicError();
	//重新初始化TIM3
	MX_TIM3_Init();			
	//重新初始化电机方向引脚为 普通GPIO
	MotorGpioReInit();
	//编码器
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	//安全起见，关输出
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);	//低
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);	//低
	
	//设定PID参数 s=7,Ts=5; 
	//Kp = 5.6 = 0.8 * s, Ki = 1.5 = 0.3 * Ts, Kd = 0.5 = 0.1 * Ts;
	pidKp = 4.0,	pidKi = 0.4, pidKd  = 0.3;
	
	//H桥电机
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);						//MotorSD
	//编码器 初值
	__HAL_TIM_SET_COUNTER(&htim4, 30000);
	
	//脉冲值 * 0.03 = 速度(m/s)
  /* Infinite loop */
  for(;;)
  {
		//喂狗
		HAL_IWDG_Refresh(&hiwdg);
		
		//获取 车速
		xQueuePeek(carSetSpeedHandle, &pidCarSetSpeed, 0);
		pidSetPoint = (int16_t)(pidCarSetSpeed * 33.33);	//m/s - > 脉冲值
		
		//设定值 限幅
		pidSetPoint = pidSetPoint < -167 ? -167 : pidSetPoint;
		pidSetPoint = pidSetPoint > 167 ? 167 : pidSetPoint;
		
		//获取编码器计数值
		pidInput = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim4)) - 30000;
		__HAL_TIM_SET_COUNTER(&htim4, 30000);
		
		//PID
		pidEt = pidSetPoint - pidInput;
		pidOutput += pidKp * (pidEt - pidLastErr) + pidKi * pidEt + pidKd * (pidEt - 2 * pidLastErr + pidLastTwoErr);
		//PID限幅
		pidOutput = pidOutput > 265 ? 265 : pidOutput;
		pidOutput = pidOutput < -265 ? -265 : pidOutput;
		
		//停车时，尝试减小输出，尝试1次
		if(!pidSetPoint)
		{
			if(pidSetPointLast)										//刚下发 停车 指令
				pidCarStopTime = 0, pidCarBreakRemove = 0;
			else
				pidCarStopTime++;
			if(pidCarStopTime > 50)								//1s 后
			{
				if((!pidInput) && (pidLastErr))			//车 运动刚停下
					pidCarBreakRemove++;
				if(pidCarBreakRemove == 0)					
					pidOutput += pidOutput < 0 ? 0.072 : -0.072;
			}
		}
		pidSetPointLast = pidSetPoint;
		
		//输出曲线。类似对数曲线。
		//分界点 25:256.7; 0.002453; -0.09653;
		//分界点 18:266.2; 0.002425; -0.125;
		//分界点 15:266.2; 0.002425; -0.155; use
		//可快速从驻车状态退出，且低速时刹车距离变短。
		//f(x) = 256.7[227.6, 285.8] * exp(0.002453[0.001927, 0.002979] * x) 
		//				+ (-256.7[-297.8, -215.6]) * exp(-0.09653[-0.1635, -0.02957] * x);
		pidMotorOutput = 266.2 * (exp(0.002425 * (pidOutput < 0 ? - pidOutput : pidOutput)) 
											- exp(-0.155 * (pidOutput < 0 ? - pidOutput : pidOutput))) + 0.5;
		//输出 限幅
		pidMotorOutput = pidMotorOutput > 499 ? 499 : pidMotorOutput;
		
		//设定方向	
		HAL_GPIO_WritePin(MotorLift_GPIO_Port, MotorLift_Pin, pidOutput < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MotorRight_GPIO_Port, MotorRight_Pin, pidOutput < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
				
		//电机 输出
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pidMotorOutput);
		
		//覆写消息
		pidCarBreak = (pidInput * pidOutput <= 0 ? 1 : 0);
		xQueueOverwrite(carBreakHandle, &pidCarBreak);
		pidCarRealSpeed = (pidInput * 33.33);
		xQueueOverwrite(carRealSpeedHandle, &pidCarRealSpeed);
		
		#if DEBUG	
			//拷贝 Debug 数据
			SetPoint = pidSetPoint, Input = pidInput, Output = pidOutput;
			Et = pidMotorOutput, LastErr = pidCarBreakRemove, LastTwoErr = pidLastTwoErr;
		#endif
		
		pidLastTwoErr = pidLastErr;
		pidLastErr = pidEt;
    osDelayUntil(&pidLastTick, 20);
  }
  /* USER CODE END StartMotorPID */
}

/* USER CODE BEGIN Header_StartRemoteDBUS */
/**
* @brief Function implementing the remoteDBUS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteDBUS */
void StartRemoteDBUS(void const * argument)
{
  /* USER CODE BEGIN StartRemoteDBUS */
    /*
    遥控器为 DJI DT7&DR16 Robomaster特殊固件
    官方尚未公开售卖&协议资料，所以这里删除了解码部分；您可根据自己的遥控器编写对应的解码；
    或者您可以不使用遥控器控制，只使用 USART1 控制车辆，可不修改该任务代码；
    使用USART3+DMA空闲中断的方式接收，空闲时触发中断，并向信号量写入数据；
    这里等待信号量，并解码。解码后写入速度、方向、各控制器件等队列，供其他任务使用；
    */
	//D-BUS 
	//通道1：速度；通道2：方向（右手系）；
	//左拨杆：上：UART；中：低速（MAX 2.5m/s)；下：高速（MAX 5m/s）；
	//右拨杆：上：切换大灯状态（上升沿触发）；中：无动作；下：打开蜂鸣器（电平触发）
	//通道0&3，拨轮，键鼠：预留。
	uint8_t DBUS_RxBuffer[18] = {0};	//接收数据缓存区
	int16_t dbusRemoteSpeed = 0;		//速度
	int16_t dbusRemoteAngle = 0;		//角度
	uint8_t dbusRemoteSwitchLift = 0;		//遥控左开关状态
	uint8_t dbusRemoteSwitchRight = 0, dbusRemoteSwitchRightLast;	//遥控右开关状态
	uint8_t dbusErrorCount = 0;			//遥控数据错误计数
	
	float dbusRemoteSetSpeed = 0, dbusRemoteSetDirection = 0;
	uint16_t dbusCarControlMode = 0, dbusCarControlGear = 0;
	uint16_t dbusCarLight = 0, dbusCarBuzzer = 0;

	//开中断 会进入一次
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);					//DBUS 空闲中断
	HAL_UART_Receive_DMA(&huart3, DBUS_RxBuffer, 18);			//DMA
	
	//初始化遥控器 各通道 为 中位；【这里需根据您的遥控器设置为 中位 对应的参数】
	DBUS_RxBuffer[0] = 0x00;
	DBUS_RxBuffer[1] = 0x00;
	DBUS_RxBuffer[2] = 0x00;
	DBUS_RxBuffer[3] = 0x00;
	DBUS_RxBuffer[4] = 0x00;
	DBUS_RxBuffer[5] = 0x00;
	
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(succRemoteDBUSHandle, osWaitForever);
		
		//处理数据【需根据您的遥控器修改，这里只是演示，无法运行；】
		//遥控器面板 0~3通道
//		CH0 = DBUS_RxBuffer[0];
		dbusRemoteSpeed = DBUS_RxBuffer[1];
		dbusRemoteAngle = DBUS_RxBuffer[2];
//		CH3 = DBUS_RxBuffer[3];
      
		dbusRemoteSwitchLift = (DBUS_RxBuffer[4] & 0x0003;		//左拨杆 档位:上1，下2，中3
		dbusRemoteSwitchRight = DBUS_RxBuffer[5] & 0x0003;		//右拨杆 档位:上1，下2，中3
//		TrackWheel = DBUS_RxBuffer[16];		//拨轮
		
		//判断遥控数据是否正确
		dbusErrorCount = 0;
		dbusErrorCount += dbusRemoteSpeed > 660 ? 1 : 0;
		dbusErrorCount += dbusRemoteSpeed < -660 ? 1 : 0;
		dbusErrorCount += dbusRemoteAngle > 660 ? 1 : 0;
		dbusErrorCount += dbusRemoteAngle < -660 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchLift > 3 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchLift == 0 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchRight > 3 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchRight == 0 ? 1 : 0;
		if(dbusErrorCount)	//数据存在错误，跳过本次循环
			continue;
		
        // 【以下部分不用修改，可直接使用】
		//左拨杆
		if(dbusRemoteSwitchLift != 1)									//非上 -> 遥控器控制
		{
			dbusCarControlMode = 1;
			if((dbusRemoteSpeed < 10) && (dbusRemoteSpeed > -10)) dbusRemoteSpeed = 0;		//摇杆死区
			if(dbusRemoteSwitchLift == 3)	//中 -> 低速
			{
				dbusCarControlGear = 1;
				dbusRemoteSpeed *= 0.5;
			}
			else
				dbusCarControlGear = 2;			//下 -> 高速
			dbusRemoteSetSpeed = (float)dbusRemoteSpeed * 0.00758;	//摇杆值 转 速度m/s(0.00758 = 1 / (660 / 5m/s))
			dbusRemoteSetDirection = (float)dbusRemoteAngle * 0.5;
			
			//覆写消息队列
			xQueueOverwrite(remoteSpeedHandle, &dbusRemoteSetSpeed);
			xQueueOverwrite(remoteDirectionHandle, &dbusRemoteSetDirection);			
			xQueueOverwrite(carControlGearHandle, &dbusCarControlGear);
		}
		else			//上 -> 串口控制
		{
			dbusCarControlMode = 2;
		}
		xQueueOverwrite(carControlModeHandle, &dbusCarControlMode);
		
		//右拨杆 -> 大灯 & 喇叭
		if(dbusRemoteSwitchRight != dbusRemoteSwitchRightLast)
		{
			if(dbusRemoteSwitchRight == 1)			//上 -> 大灯
			{
				xQueuePeek(carLightHandle, &dbusCarLight, 7);
				dbusCarLight = (dbusCarLight != 0) ? 0 : 1;
				xQueueOverwrite(carLightHandle, &dbusCarLight);
			}
			else
				if(dbusRemoteSwitchRight == 2)				//下 -> 喇叭
				{
					xQueuePeek(carBuzzerHandle, &dbusCarBuzzer, 7);
					dbusCarBuzzer = (dbusCarBuzzer != 0) ? 0 : 1;
					xQueueOverwrite(carBuzzerHandle, &dbusCarBuzzer);
				}
		}
		dbusRemoteSwitchRightLast = dbusRemoteSwitchRight;
	}
  /* USER CODE END StartRemoteDBUS */
}

/* USER CODE BEGIN Header_StartRemoteUART */
/**
* @brief Function implementing the remoteUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteUART */
void StartRemoteUART(void const * argument)
{
  /* USER CODE BEGIN StartRemoteUART */
	//UART
	//115200bps，8位数据，1位停止，无校验；
	//16/帧
	//0&1&2：帧头0x4D 0x6F 0x65
	//3&4 ：速度（m/s）* 100 + 1000 [大端模式]
	//5&6 ：方向（舵机脉宽） + 1000；（中位 0 + 1000）[大端模式]
	//7：大灯；8：蜂鸣器；9：WS2812B彩灯
	//10 - 15：预留
	uint8_t UARTRxBuffer[16] = {0};
	float uartRemoteSpeed = 0, uartRemoteDirection = 0;
	uint16_t uartCarControlMode = 0;
	uint16_t uartCarControlGear = 0;
	uint16_t uartCarLight = 0, uartCarBuzzer = 0,uartCarSpiLight = 0;
	
	//开中断 会进入一次
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);					// 空闲中断
	HAL_UART_Receive_DMA(&huart1, UARTRxBuffer, 16);			//DMA
	
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(succRemoteUARTHandle, osWaitForever);
		
		xQueuePeek(carControlModeHandle, &uartCarControlMode, 1);
		if(uartCarControlMode == 2)		//允许串口控制
		{
			if((UARTRxBuffer[0] == 0x4D) && (UARTRxBuffer[1] == 0x6F) 
					&& (UARTRxBuffer[2] == 0x65))														//校验帧头
			{
				uartRemoteSpeed = ((((UARTRxBuffer[3] << 8) & 0xFF00) | UARTRxBuffer[4]) - 1000) / 100;
				uartRemoteDirection = ((((UARTRxBuffer[5] << 8) & 0xFF00) | UARTRxBuffer[6]) - 1000);
				uartCarControlGear = 2;     // 默认高速档位 max 5m/s
				//覆写 消息队列
				if((uartRemoteSpeed < 0 ? -uartRemoteSpeed : uartRemoteSpeed) <= 5.0)
					xQueueOverwrite(remoteSpeedHandle, &uartRemoteSpeed);
				if((uartRemoteDirection < 0 ? -uartRemoteDirection : uartRemoteDirection) <= 330.0)
					xQueueOverwrite(remoteDirectionHandle, &uartRemoteDirection);
				xQueueOverwrite(carControlGearHandle, &uartCarControlGear);
				
				uartCarLight = (UARTRxBuffer[7] != 0) ? 0x0001 : 0x0000;
				uartCarBuzzer = (UARTRxBuffer[8] != 0) ? 0x0001 : 0x0000;
				uartCarSpiLight = (UARTRxBuffer[9] != 0) ? 0x0001 : 0x0000;
				xQueueOverwrite(carLightHandle, &uartCarLight);
				xQueueOverwrite(carBuzzerHandle, &uartCarBuzzer);
				xQueueOverwrite(carSpiLightHandle, &uartCarSpiLight);
			}
		}
  }
  /* USER CODE END StartRemoteUART */
}

/* USER CODE BEGIN Header_StartLightSPI */
/**
* @brief Function implementing the lightSPI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightSPI */
void StartLightSPI(void const * argument)
{
  /* USER CODE BEGIN StartLightSPI */
	//WS2812B彩灯
	//SPI模拟单线归零码
	//DBUS控制：低速 -> 绿灯；高速 -> 蓝灯；
	//UART控制：黄灯
	//失控    ：红灯
	//尾灯：刹车 高亮；行车 低亮
	uint8_t i;
	uint8_t spiValue[6] = {0x00};							//亮度运算值
	
	float spiCarSetSpeed = 0;
	uint16_t spiCarControlMode = 0, spiCarControlGear = 0;
	uint16_t spiCarRunaway = 0,spiCarSpiLight = 0;
	uint8_t spiCarBreak = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xQueuePeek(carSpiLightHandle, &spiCarSpiLight, 1);
		if(spiCarSpiLight == 0)		//失能 灯带
			Ws2812ExtinguishAll();	//清空灯带
		else
		{
			Ws2812ExtinguishAll();	//清空灯带
			xQueuePeek(carRunawayHandle, &spiCarRunaway, 1);
			
			if(spiCarRunaway == 1)					//失控 -> 红灯
				Ws2812bSetContinuous(1, 12, 50, 0, 0);
			else			//未失控
			{
				//获取消息队列
				xQueuePeek(carControlModeHandle, &spiCarControlMode, 1);
				xQueuePeek(carControlGearHandle, &spiCarControlGear, 1);
				xQueuePeek(carSetSpeedHandle, &spiCarSetSpeed, 1);
				//取绝对值
				spiCarSetSpeed = spiCarSetSpeed < 0 ? -spiCarSetSpeed : spiCarSetSpeed;	
				//低速时 翻倍;(spiCarControlGear = 1：低速，2：高速)
				spiCarSetSpeed *= (3 - spiCarControlGear);
				//清空spiValue
				for(i = 0; i < 6; i++)
					spiValue[i] = 0;
				//设置灯带数据
				spiValue[0] = 255;
				for(i = 1; i < (uint8_t)(spiCarSetSpeed + 1); i++)
					spiValue[i] = 255;
				spiValue[i] = (uint8_t)((spiCarSetSpeed - (i - 1)) * 258);
				//选择显示颜色
				for(i = 1; i <= 6; i++)
				{
					if(spiCarControlMode == 2)				//串口控制 -> 黄
					{
						Ws2812bSet(i, spiValue[i - 1], spiValue[i - 1], 0);		
						Ws2812bSet(13 - i, spiValue[i - 1], spiValue[i - 1], 0);
					}
					else
						if(spiCarControlGear == 2)			//高速
						{
							Ws2812bSet(i, 0, 0, spiValue[i - 1]);		
							Ws2812bSet(13 - i, 0, 0, spiValue[i - 1]);
						}
						else
							if(spiCarControlGear == 1)		//低速
							{
								Ws2812bSet(i, 0, spiValue[i - 1], 0);		
								Ws2812bSet(13 - i, 0, spiValue[i - 1], 0);
							}
				}
			}
			//尾灯
			xQueuePeek(carBreakHandle, &spiCarBreak, 1);
			if(spiCarBreak) Ws2812bSetContinuous(13, 16 + 7, 255, 0, 0);
			else Ws2812bSetContinuous(13, 16 + 7, 25, 0, 0);
		}
		Ws2812bRefresh();
		osDelay(50);
  }
  /* USER CODE END StartLightSPI */
}

/* USER CODE BEGIN Header_StartGimbal */
/**
* @brief Function implementing the gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbal */
void StartGimbal(void const * argument)
{
  /* USER CODE BEGIN StartGimbal */
	//云台 【未完成】
	//三轴，无自稳
	uint16_t Count = 800;
	
	//舵机
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);						//Pitch
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);						//Yaw
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);						//Roll
	
  /* Infinite loop */
  for(;;)
  {
      // 测试舵机 无意义
		if((Count += 100) > 2000) Count = 1000;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Count);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Count);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, Count);
    osDelay(1000);
  }
  /* USER CODE END StartGimbal */
}

/* USER CODE BEGIN Header_StartFeatureSelect */
/**
* @brief Function implementing the featureSelect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFeatureSelect */
void StartFeatureSelect(void const * argument)
{
  /* USER CODE BEGIN StartFeatureSelect */
	//拨码开关 功能选择
	//500ms 判断一次，根据状态 挂起/恢复 相应任务；
	//1：UART使能
	//2：DBUS使能
	//3：云台使能
	//4：WS2812B使能
	uint8_t LastStateSW1, LastStateSW2, LastStateSW3, LastStateSW4;
	uint8_t NowStateSW1, NowStateSW2, NowStateSW3, NowStateSW4;
	uint16_t selectCarSpiLight = 0;
	uint16_t selectCarControlMode = 0;
		
	//和现实世界情况相反，创造第一次运行条件。
	LastStateSW1 = !HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
	LastStateSW2 = !HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
	LastStateSW3 = !HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
	LastStateSW4 = !HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin);
	
  /* Infinite loop */
  for(;;)
  {
		NowStateSW1 = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
		NowStateSW2 = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
		NowStateSW3 = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
		NowStateSW4 = HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin);
		
		//UART控制
		if(NowStateSW1 != LastStateSW1)
		{
			HAL_GPIO_WritePin(OutPower_GPIO_Port, OutPower_Pin, NowStateSW1);
			if(NowStateSW1 == GPIO_PIN_RESET)
			{
				vTaskResume(remoteUARTHandle);
			}
			else
			{	
				vTaskSuspend(remoteUARTHandle);
			}
		}
		
		//DBUS控制 【需您修改解码部分，若不想写，可不使用DBUS控制】
		if(NowStateSW2 != LastStateSW2)
		{
			if(NowStateSW2 == GPIO_PIN_RESET)
			{
				vTaskResume(remoteDBUSHandle);
			}
			else
			{
				vTaskSuspend(remoteDBUSHandle);
			}
		}
		
		//若只开 UART，则允许UART控制
		if((NowStateSW2 == GPIO_PIN_SET) && (NowStateSW1 == GPIO_PIN_RESET))
		{
				selectCarControlMode = 2;
				xQueueOverwrite(carControlModeHandle, &selectCarControlMode);
		}
		
		//云台 【任务代码未完成，无意义】
		if(NowStateSW3 != LastStateSW3)
		{
			if(NowStateSW3 == GPIO_PIN_RESET)
			{
				vTaskResume(gimbalHandle);
			}
			else
			{
				vTaskSuspend(gimbalHandle);
			}
		}
		
		//Ws2812b灯带
		if(NowStateSW4 != LastStateSW4)
		{
			if(NowStateSW4 == GPIO_PIN_RESET)
			{
				vTaskResume(lightSPIHandle);
				selectCarSpiLight = 1;
			}
			else
			{
				vTaskSuspend(lightSPIHandle);
				selectCarSpiLight = 0;
				Ws2812ExtinguishAll();
				Ws2812bRefresh();
			}
			xQueueOverwrite(carSpiLightHandle, &selectCarSpiLight);
		}
		
		LastStateSW1 = NowStateSW1;
		LastStateSW2 = NowStateSW2;
		LastStateSW3 = NowStateSW3;
		LastStateSW4 = NowStateSW4;
		
    osDelay(500);
  }
  /* USER CODE END StartFeatureSelect */
}

/* USER CODE BEGIN Header_StartStatusReturn */
/**
* @brief Function implementing the statusReturn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatusReturn */
void StartStatusReturn(void const * argument)
{
  /* USER CODE BEGIN StartStatusReturn */
	//状态回传
	//通过串口回传车运行参数，4Hz，间隔250ms；
	//0&1&2：帧头：0x4D 0x6F 0x65
	//3&4  ：设定速度m/s * 100 + 1000；大端模式
	//5&6  ：实际速度m/s * 100 + 1000；大端模式
	//7&8  ：设定方向（舵机偏离中点脉宽） + 1000；大端模式
	//9&10 ：电压 * 10；电流 * 10；
	//12   ：车设备状态（按位设置）
	//13 - 15：预留
	uint8_t UARTTxBuffer[16] = {0};
	
	uint16_t statusCarSetSpeed = 0, statusCarSetDirection = 0;
	uint16_t statusCarControlMode = 0, statusCarControlGear = 0;
	uint16_t statusCarRealSpeed = 0;
	uint16_t statusCarRunaway = 0;
	uint16_t statusCarLight = 0, statusCarBuzzer = 0;
	uint16_t statusCarSpiLight = 0;
	float statusCarVoltage = 0, statusCarCurrent = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xQueuePeek(carSetSpeedHandle, &statusCarSetSpeed, 1);
		xQueuePeek(carSetDirectionHandle, &statusCarSetDirection, 1);
		xQueuePeek(carControlModeHandle, &statusCarControlMode, 1);
		xQueuePeek(carControlGearHandle, &statusCarControlGear, 1);
		xQueuePeek(carRunawayHandle, &statusCarRunaway, 1);
		xQueuePeek(carRealSpeedHandle, &statusCarRealSpeed, 1);
		
		xQueuePeek(carLightHandle, &statusCarLight, 1);
		xQueuePeek(carBuzzerHandle, &statusCarBuzzer, 1);
		xQueuePeek(carSpiLightHandle, &statusCarSpiLight, 1);
		xQueuePeek(carVoltageHandle, &statusCarVoltage, 1);
		xQueuePeek(carCurrentHandle, &statusCarCurrent, 1);
		
		UARTTxBuffer[0] = 0x4D;
		UARTTxBuffer[1] = 0x6F;
		UARTTxBuffer[2] = 0x65;
		
		//设定速度
		UARTTxBuffer[3] = (statusCarSetSpeed * 100 + 1000) >> 8;
		UARTTxBuffer[4] = (statusCarSetSpeed * 100 + 1000);
		
		//实际速度		
		UARTTxBuffer[5] = (statusCarRealSpeed * 100 + 1000) >> 8;	
		UARTTxBuffer[6] = (statusCarRealSpeed * 100 + 1000);
		
		//设定角度
		UARTTxBuffer[7] = (statusCarSetDirection + 1000) >> 8;
		UARTTxBuffer[8] = (statusCarSetDirection + 1000);
		
		UARTTxBuffer[9] = (uint8_t)(statusCarVoltage * 10);		//电压
		UARTTxBuffer[10] = (uint8_t)(statusCarCurrent * 10);		//电流（预留）
		
		UARTTxBuffer[11] = 0;
		UARTTxBuffer[12] = ((statusCarSpiLight & 0x0001) << 6) | ((statusCarBuzzer & 0x0001) << 5) 
											| ((statusCarLight & 0x0001) << 4) 
											| (((statusCarControlGear & 0x0003) - 1) << 3) 
											| ((statusCarControlMode == 2 ? 1 : 0) << 2)
											| ((statusCarControlMode == 1 ? 1 : 0) << 1)
											| ((statusCarRunaway & 0x0001));
											
		//预留
		UARTTxBuffer[13] = 0;
		UARTTxBuffer[14] = 0;	
		UARTTxBuffer[15] = 0;
		
		HAL_UART_Transmit_DMA(&huart1, UARTTxBuffer, 16);
			
    osDelay(250);
  }
  /* USER CODE END StartStatusReturn */
}

/* USER CODE BEGIN Header_StartVoltageMonitor */
/**
* @brief Function implementing the voltageMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVoltageMonitor */
void StartVoltageMonitor(void const * argument)
{
  /* USER CODE BEGIN StartVoltageMonitor */
	//电压监测
	//避免电池过放，监测频率：正常 4Hz；低电压 10Hz
	//报警电压：3.5 * 3 = 10.5V；超时 1s 报警
	//报警动作：蜂鸣器 “滴” 10Hz；
	volatile uint16_t voltageAdcValue[2] = {0};		//ADC值
	float voltageVoltage = 0.0;			//实际电压值
	float voltageLow = 10.5, voltageSeriousLow = 10.0;				//报警电压
	uint32_t voltageLowTime = 0, voltageSeriousLowTime = 0;		//低电压计数
	float voltageCurrent = 0.0;			//电流值
	uint32_t voltageCurrentCalibration = 0;
	
	//车 外设
	uint16_t voltageCarHeadLight = 0, voltageCarBuzzer = 0;
	
	//校准ADC
	HAL_ADCEx_Calibration_Start(&hadc1);
	osDelay(2);
	
	//开DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)voltageAdcValue, 2);
	
//	//校准 电流ADC值
//	for(uint8_t i = 0; i < 100; i++)
//		voltageCurrentCalibration += voltageAdcValue[0], osDelay(1);
//	voltageCurrentCalibration = (uint32_t)((float)voltageCurrentCalibration / 100 + 0.5);
	
	//外设初始化
	SectionPeripheralReInit();
	
	//自检
	HAL_GPIO_TogglePin(HeadLight_GPIO_Port, HeadLight_Pin);
	HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
	osDelay(50);
	HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
	osDelay(50);
	HAL_GPIO_TogglePin(HeadLight_GPIO_Port, HeadLight_Pin);
	osDelay(100);
	HAL_GPIO_TogglePin(HeadLight_GPIO_Port, HeadLight_Pin);
	osDelay(100);
	HAL_GPIO_TogglePin(HeadLight_GPIO_Port, HeadLight_Pin);
	
  /* Infinite loop */
  for(;;)
  {
		//大灯
		xQueuePeek(carLightHandle, &voltageCarHeadLight, 1);
		HAL_GPIO_WritePin(HeadLight_GPIO_Port, HeadLight_Pin, voltageCarHeadLight);
		
		//运算实际电压值，并覆写消息
		voltageVoltage = voltageAdcValue[1] * 0.00349 + 0.005;
		xQueueOverwrite(carVoltageHandle, &voltageVoltage);
		
		//运算实际电流值，并覆写消息
		voltageCurrent = (voltageAdcValue[0]) * 0.00533 + 0.005;
		xQueueOverwrite(carCurrentHandle, &voltageCurrent);
		
		//严重低电量？
		if(voltageVoltage > voltageSeriousLow)
			voltageSeriousLowTime = xTaskGetTickCount();
		
		//低电压？
		if(voltageVoltage > voltageLow)
			voltageLowTime = xTaskGetTickCount();
		
		//报警？
		if((xTaskGetTickCount() - voltageSeriousLowTime) >= 1000)
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, (xTaskGetTickCount() % 500 < 100) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		else
			if((xTaskGetTickCount() - voltageLowTime) >= 1000)
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, (xTaskGetTickCount() % 1000 < 50) ? GPIO_PIN_SET : GPIO_PIN_RESET);
			else
			{
				xQueuePeek(carBuzzerHandle, &voltageCarBuzzer, 1);
				HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, voltageCarBuzzer);
			}
		
		//拷贝 Debug 数据
		Voltage = (int16_t)(voltageVoltage * 100);
//		Current = (int16_t)(voltageCurrent * 100);
		Current = voltageAdcValue[0];
			
		osDelay(50);
  }
  /* USER CODE END StartVoltageMonitor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
