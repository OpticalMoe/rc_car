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
	//ι��
	HAL_IWDG_Refresh(&hiwdg);
	
	//��������		
	vTaskSuspend(remoteDBUSHandle);			//����DBUSң������
	vTaskSuspend(remoteUARTHandle);			//����UARTң������
	vTaskSuspend(lightSPIHandle);				//����Ws2812b�ƴ�����
	vTaskSuspend(gimbalHandle);					//������̨����
			
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
	//ʧ�ر���
	//��ʱ 500ms + 10ms(����) δ�յ���Ч���ݣ���ִ��ʧ�ر���
	//ʧ�ر����������ٶ� 0m/s��������У�����ʧ�� ��־
  float IWDG_CarSetSpeed = 0, IWDG_CarSetDirection = 0;
	uint16_t IWDG_CarDirectionZero = 1500;
	uint16_t IWDG_CarRunaway = 1;
	
	//��� ����
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);						//Direction
	
	//�ϵ����
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, IWDG_CarDirectionZero - IWDG_CarSetDirection);
	
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(Sign_GPIO_Port, Sign_Pin);
		//ʧ�ر���
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
		
		//���� �޷� & ���
		IWDG_CarSetDirection = IWDG_CarSetDirection < -330 ? -330 : IWDG_CarSetDirection;
		IWDG_CarSetDirection = IWDG_CarSetDirection > 330 ? 330 : IWDG_CarSetDirection;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, IWDG_CarDirectionZero - IWDG_CarSetDirection);
		
		//���� �޷�
		IWDG_CarSetSpeed = IWDG_CarSetSpeed < -5.0 ? -5.0 : IWDG_CarSetSpeed;
		IWDG_CarSetSpeed = IWDG_CarSetSpeed > 5.0 ? 5.0 : IWDG_CarSetSpeed;
		
		//��д ��Ϣ����
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
	//PID ����
	//����ʽ PID������ 20ms
	int16_t pidInput = 0, pidSetPoint = 0;
	int16_t pidEt = 0, pidLastErr = 0, pidLastTwoErr = 0;
	double pidOutput = 0.0;
	float pidKp = 0, pidKi = 0, pidKd  = 0;
	uint16_t pidMotorOutput = 0;
	float pidCarRealSpeed = 0;
	
	//ɲ��
	int16_t pidSetPointLast = 0;
	uint8_t pidCarBreak = 0;
	uint8_t pidCarBreakRemove = 0;
	uint32_t pidCarStopTime = 0;
	
	//����������
	float pidCarSetSpeed = 0;
	
	//��׼��ʱ
	TickType_t pidLastTick = xTaskGetTickCount();
	
	//����Լ�
	if(MotorSelfCheck())
		MotorMusicSuccess();
	else 
		MotorMusicError();
	//���³�ʼ��TIM3
	MX_TIM3_Init();			
	//���³�ʼ�������������Ϊ ��ͨGPIO
	MotorGpioReInit();
	//������
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	//��ȫ����������
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);	//��
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);	//��
	
	//�趨PID���� s=7,Ts=5; 
	//Kp = 5.6 = 0.8 * s, Ki = 1.5 = 0.3 * Ts, Kd = 0.5 = 0.1 * Ts;
	pidKp = 4.0,	pidKi = 0.4, pidKd  = 0.3;
	
	//H�ŵ��
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);						//MotorSD
	//������ ��ֵ
	__HAL_TIM_SET_COUNTER(&htim4, 30000);
	
	//����ֵ * 0.03 = �ٶ�(m/s)
  /* Infinite loop */
  for(;;)
  {
		//ι��
		HAL_IWDG_Refresh(&hiwdg);
		
		//��ȡ ����
		xQueuePeek(carSetSpeedHandle, &pidCarSetSpeed, 0);
		pidSetPoint = (int16_t)(pidCarSetSpeed * 33.33);	//m/s - > ����ֵ
		
		//�趨ֵ �޷�
		pidSetPoint = pidSetPoint < -167 ? -167 : pidSetPoint;
		pidSetPoint = pidSetPoint > 167 ? 167 : pidSetPoint;
		
		//��ȡ����������ֵ
		pidInput = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim4)) - 30000;
		__HAL_TIM_SET_COUNTER(&htim4, 30000);
		
		//PID
		pidEt = pidSetPoint - pidInput;
		pidOutput += pidKp * (pidEt - pidLastErr) + pidKi * pidEt + pidKd * (pidEt - 2 * pidLastErr + pidLastTwoErr);
		//PID�޷�
		pidOutput = pidOutput > 265 ? 265 : pidOutput;
		pidOutput = pidOutput < -265 ? -265 : pidOutput;
		
		//ͣ��ʱ�����Լ�С���������1��
		if(!pidSetPoint)
		{
			if(pidSetPointLast)										//���·� ͣ�� ָ��
				pidCarStopTime = 0, pidCarBreakRemove = 0;
			else
				pidCarStopTime++;
			if(pidCarStopTime > 50)								//1s ��
			{
				if((!pidInput) && (pidLastErr))			//�� �˶���ͣ��
					pidCarBreakRemove++;
				if(pidCarBreakRemove == 0)					
					pidOutput += pidOutput < 0 ? 0.072 : -0.072;
			}
		}
		pidSetPointLast = pidSetPoint;
		
		//������ߡ����ƶ������ߡ�
		//�ֽ�� 25:256.7; 0.002453; -0.09653;
		//�ֽ�� 18:266.2; 0.002425; -0.125;
		//�ֽ�� 15:266.2; 0.002425; -0.155; use
		//�ɿ��ٴ�פ��״̬�˳����ҵ���ʱɲ�������̡�
		//f(x) = 256.7[227.6, 285.8] * exp(0.002453[0.001927, 0.002979] * x) 
		//				+ (-256.7[-297.8, -215.6]) * exp(-0.09653[-0.1635, -0.02957] * x);
		pidMotorOutput = 266.2 * (exp(0.002425 * (pidOutput < 0 ? - pidOutput : pidOutput)) 
											- exp(-0.155 * (pidOutput < 0 ? - pidOutput : pidOutput))) + 0.5;
		//��� �޷�
		pidMotorOutput = pidMotorOutput > 499 ? 499 : pidMotorOutput;
		
		//�趨����	
		HAL_GPIO_WritePin(MotorLift_GPIO_Port, MotorLift_Pin, pidOutput < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MotorRight_GPIO_Port, MotorRight_Pin, pidOutput < 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
				
		//��� ���
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pidMotorOutput);
		
		//��д��Ϣ
		pidCarBreak = (pidInput * pidOutput <= 0 ? 1 : 0);
		xQueueOverwrite(carBreakHandle, &pidCarBreak);
		pidCarRealSpeed = (pidInput * 33.33);
		xQueueOverwrite(carRealSpeedHandle, &pidCarRealSpeed);
		
		#if DEBUG	
			//���� Debug ����
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
    ң����Ϊ DJI DT7&DR16 Robomaster����̼�
    �ٷ���δ��������&Э�����ϣ���������ɾ���˽��벿�֣����ɸ����Լ���ң������д��Ӧ�Ľ��룻
    ���������Բ�ʹ��ң�������ƣ�ֻʹ�� USART1 ���Ƴ������ɲ��޸ĸ�������룻
    ʹ��USART3+DMA�����жϵķ�ʽ���գ�����ʱ�����жϣ������ź���д�����ݣ�
    ����ȴ��ź����������롣�����д���ٶȡ����򡢸����������ȶ��У�����������ʹ�ã�
    */
	//D-BUS 
	//ͨ��1���ٶȣ�ͨ��2����������ϵ����
	//�󲦸ˣ��ϣ�UART���У����٣�MAX 2.5m/s)���£����٣�MAX 5m/s����
	//�Ҳ��ˣ��ϣ��л����״̬�������ش��������У��޶������£��򿪷���������ƽ������
	//ͨ��0&3�����֣�����Ԥ����
	uint8_t DBUS_RxBuffer[18] = {0};	//�������ݻ�����
	int16_t dbusRemoteSpeed = 0;		//�ٶ�
	int16_t dbusRemoteAngle = 0;		//�Ƕ�
	uint8_t dbusRemoteSwitchLift = 0;		//ң���󿪹�״̬
	uint8_t dbusRemoteSwitchRight = 0, dbusRemoteSwitchRightLast;	//ң���ҿ���״̬
	uint8_t dbusErrorCount = 0;			//ң�����ݴ������
	
	float dbusRemoteSetSpeed = 0, dbusRemoteSetDirection = 0;
	uint16_t dbusCarControlMode = 0, dbusCarControlGear = 0;
	uint16_t dbusCarLight = 0, dbusCarBuzzer = 0;

	//���ж� �����һ��
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);					//DBUS �����ж�
	HAL_UART_Receive_DMA(&huart3, DBUS_RxBuffer, 18);			//DMA
	
	//��ʼ��ң���� ��ͨ�� Ϊ ��λ�����������������ң��������Ϊ ��λ ��Ӧ�Ĳ�����
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
		
		//�������ݡ����������ң�����޸ģ�����ֻ����ʾ���޷����У���
		//ң������� 0~3ͨ��
//		CH0 = DBUS_RxBuffer[0];
		dbusRemoteSpeed = DBUS_RxBuffer[1];
		dbusRemoteAngle = DBUS_RxBuffer[2];
//		CH3 = DBUS_RxBuffer[3];
      
		dbusRemoteSwitchLift = (DBUS_RxBuffer[4] & 0x0003;		//�󲦸� ��λ:��1����2����3
		dbusRemoteSwitchRight = DBUS_RxBuffer[5] & 0x0003;		//�Ҳ��� ��λ:��1����2����3
//		TrackWheel = DBUS_RxBuffer[16];		//����
		
		//�ж�ң�������Ƿ���ȷ
		dbusErrorCount = 0;
		dbusErrorCount += dbusRemoteSpeed > 660 ? 1 : 0;
		dbusErrorCount += dbusRemoteSpeed < -660 ? 1 : 0;
		dbusErrorCount += dbusRemoteAngle > 660 ? 1 : 0;
		dbusErrorCount += dbusRemoteAngle < -660 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchLift > 3 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchLift == 0 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchRight > 3 ? 1 : 0;
		dbusErrorCount += dbusRemoteSwitchRight == 0 ? 1 : 0;
		if(dbusErrorCount)	//���ݴ��ڴ�����������ѭ��
			continue;
		
        // �����²��ֲ����޸ģ���ֱ��ʹ�á�
		//�󲦸�
		if(dbusRemoteSwitchLift != 1)									//���� -> ң��������
		{
			dbusCarControlMode = 1;
			if((dbusRemoteSpeed < 10) && (dbusRemoteSpeed > -10)) dbusRemoteSpeed = 0;		//ҡ������
			if(dbusRemoteSwitchLift == 3)	//�� -> ����
			{
				dbusCarControlGear = 1;
				dbusRemoteSpeed *= 0.5;
			}
			else
				dbusCarControlGear = 2;			//�� -> ����
			dbusRemoteSetSpeed = (float)dbusRemoteSpeed * 0.00758;	//ҡ��ֵ ת �ٶ�m/s(0.00758 = 1 / (660 / 5m/s))
			dbusRemoteSetDirection = (float)dbusRemoteAngle * 0.5;
			
			//��д��Ϣ����
			xQueueOverwrite(remoteSpeedHandle, &dbusRemoteSetSpeed);
			xQueueOverwrite(remoteDirectionHandle, &dbusRemoteSetDirection);			
			xQueueOverwrite(carControlGearHandle, &dbusCarControlGear);
		}
		else			//�� -> ���ڿ���
		{
			dbusCarControlMode = 2;
		}
		xQueueOverwrite(carControlModeHandle, &dbusCarControlMode);
		
		//�Ҳ��� -> ��� & ����
		if(dbusRemoteSwitchRight != dbusRemoteSwitchRightLast)
		{
			if(dbusRemoteSwitchRight == 1)			//�� -> ���
			{
				xQueuePeek(carLightHandle, &dbusCarLight, 7);
				dbusCarLight = (dbusCarLight != 0) ? 0 : 1;
				xQueueOverwrite(carLightHandle, &dbusCarLight);
			}
			else
				if(dbusRemoteSwitchRight == 2)				//�� -> ����
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
	//115200bps��8λ���ݣ�1λֹͣ����У�飻
	//16/֡
	//0&1&2��֡ͷ0x4D 0x6F 0x65
	//3&4 ���ٶȣ�m/s��* 100 + 1000 [���ģʽ]
	//5&6 �����򣨶������ + 1000������λ 0 + 1000��[���ģʽ]
	//7����ƣ�8����������9��WS2812B�ʵ�
	//10 - 15��Ԥ��
	uint8_t UARTRxBuffer[16] = {0};
	float uartRemoteSpeed = 0, uartRemoteDirection = 0;
	uint16_t uartCarControlMode = 0;
	uint16_t uartCarControlGear = 0;
	uint16_t uartCarLight = 0, uartCarBuzzer = 0,uartCarSpiLight = 0;
	
	//���ж� �����һ��
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);					// �����ж�
	HAL_UART_Receive_DMA(&huart1, UARTRxBuffer, 16);			//DMA
	
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(succRemoteUARTHandle, osWaitForever);
		
		xQueuePeek(carControlModeHandle, &uartCarControlMode, 1);
		if(uartCarControlMode == 2)		//�����ڿ���
		{
			if((UARTRxBuffer[0] == 0x4D) && (UARTRxBuffer[1] == 0x6F) 
					&& (UARTRxBuffer[2] == 0x65))														//У��֡ͷ
			{
				uartRemoteSpeed = ((((UARTRxBuffer[3] << 8) & 0xFF00) | UARTRxBuffer[4]) - 1000) / 100;
				uartRemoteDirection = ((((UARTRxBuffer[5] << 8) & 0xFF00) | UARTRxBuffer[6]) - 1000);
				uartCarControlGear = 2;     // Ĭ�ϸ��ٵ�λ max 5m/s
				//��д ��Ϣ����
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
	//WS2812B�ʵ�
	//SPIģ�ⵥ�߹�����
	//DBUS���ƣ����� -> �̵ƣ����� -> ���ƣ�
	//UART���ƣ��Ƶ�
	//ʧ��    �����
	//β�ƣ�ɲ�� �������г� ����
	uint8_t i;
	uint8_t spiValue[6] = {0x00};							//��������ֵ
	
	float spiCarSetSpeed = 0;
	uint16_t spiCarControlMode = 0, spiCarControlGear = 0;
	uint16_t spiCarRunaway = 0,spiCarSpiLight = 0;
	uint8_t spiCarBreak = 0;
	
  /* Infinite loop */
  for(;;)
  {
		xQueuePeek(carSpiLightHandle, &spiCarSpiLight, 1);
		if(spiCarSpiLight == 0)		//ʧ�� �ƴ�
			Ws2812ExtinguishAll();	//��յƴ�
		else
		{
			Ws2812ExtinguishAll();	//��յƴ�
			xQueuePeek(carRunawayHandle, &spiCarRunaway, 1);
			
			if(spiCarRunaway == 1)					//ʧ�� -> ���
				Ws2812bSetContinuous(1, 12, 50, 0, 0);
			else			//δʧ��
			{
				//��ȡ��Ϣ����
				xQueuePeek(carControlModeHandle, &spiCarControlMode, 1);
				xQueuePeek(carControlGearHandle, &spiCarControlGear, 1);
				xQueuePeek(carSetSpeedHandle, &spiCarSetSpeed, 1);
				//ȡ����ֵ
				spiCarSetSpeed = spiCarSetSpeed < 0 ? -spiCarSetSpeed : spiCarSetSpeed;	
				//����ʱ ����;(spiCarControlGear = 1�����٣�2������)
				spiCarSetSpeed *= (3 - spiCarControlGear);
				//���spiValue
				for(i = 0; i < 6; i++)
					spiValue[i] = 0;
				//���õƴ�����
				spiValue[0] = 255;
				for(i = 1; i < (uint8_t)(spiCarSetSpeed + 1); i++)
					spiValue[i] = 255;
				spiValue[i] = (uint8_t)((spiCarSetSpeed - (i - 1)) * 258);
				//ѡ����ʾ��ɫ
				for(i = 1; i <= 6; i++)
				{
					if(spiCarControlMode == 2)				//���ڿ��� -> ��
					{
						Ws2812bSet(i, spiValue[i - 1], spiValue[i - 1], 0);		
						Ws2812bSet(13 - i, spiValue[i - 1], spiValue[i - 1], 0);
					}
					else
						if(spiCarControlGear == 2)			//����
						{
							Ws2812bSet(i, 0, 0, spiValue[i - 1]);		
							Ws2812bSet(13 - i, 0, 0, spiValue[i - 1]);
						}
						else
							if(spiCarControlGear == 1)		//����
							{
								Ws2812bSet(i, 0, spiValue[i - 1], 0);		
								Ws2812bSet(13 - i, 0, spiValue[i - 1], 0);
							}
				}
			}
			//β��
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
	//��̨ ��δ��ɡ�
	//���ᣬ������
	uint16_t Count = 800;
	
	//���
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);						//Pitch
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);						//Yaw
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);						//Roll
	
  /* Infinite loop */
  for(;;)
  {
      // ���Զ�� ������
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
	//���뿪�� ����ѡ��
	//500ms �ж�һ�Σ�����״̬ ����/�ָ� ��Ӧ����
	//1��UARTʹ��
	//2��DBUSʹ��
	//3����̨ʹ��
	//4��WS2812Bʹ��
	uint8_t LastStateSW1, LastStateSW2, LastStateSW3, LastStateSW4;
	uint8_t NowStateSW1, NowStateSW2, NowStateSW3, NowStateSW4;
	uint16_t selectCarSpiLight = 0;
	uint16_t selectCarControlMode = 0;
		
	//����ʵ��������෴�������һ������������
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
		
		//UART����
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
		
		//DBUS���� �������޸Ľ��벿�֣�������д���ɲ�ʹ��DBUS���ơ�
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
		
		//��ֻ�� UART��������UART����
		if((NowStateSW2 == GPIO_PIN_SET) && (NowStateSW1 == GPIO_PIN_RESET))
		{
				selectCarControlMode = 2;
				xQueueOverwrite(carControlModeHandle, &selectCarControlMode);
		}
		
		//��̨ ���������δ��ɣ������塿
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
		
		//Ws2812b�ƴ�
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
	//״̬�ش�
	//ͨ�����ڻش������в�����4Hz�����250ms��
	//0&1&2��֡ͷ��0x4D 0x6F 0x65
	//3&4  ���趨�ٶ�m/s * 100 + 1000�����ģʽ
	//5&6  ��ʵ���ٶ�m/s * 100 + 1000�����ģʽ
	//7&8  ���趨���򣨶��ƫ���е����� + 1000�����ģʽ
	//9&10 ����ѹ * 10������ * 10��
	//12   �����豸״̬����λ���ã�
	//13 - 15��Ԥ��
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
		
		//�趨�ٶ�
		UARTTxBuffer[3] = (statusCarSetSpeed * 100 + 1000) >> 8;
		UARTTxBuffer[4] = (statusCarSetSpeed * 100 + 1000);
		
		//ʵ���ٶ�		
		UARTTxBuffer[5] = (statusCarRealSpeed * 100 + 1000) >> 8;	
		UARTTxBuffer[6] = (statusCarRealSpeed * 100 + 1000);
		
		//�趨�Ƕ�
		UARTTxBuffer[7] = (statusCarSetDirection + 1000) >> 8;
		UARTTxBuffer[8] = (statusCarSetDirection + 1000);
		
		UARTTxBuffer[9] = (uint8_t)(statusCarVoltage * 10);		//��ѹ
		UARTTxBuffer[10] = (uint8_t)(statusCarCurrent * 10);		//������Ԥ����
		
		UARTTxBuffer[11] = 0;
		UARTTxBuffer[12] = ((statusCarSpiLight & 0x0001) << 6) | ((statusCarBuzzer & 0x0001) << 5) 
											| ((statusCarLight & 0x0001) << 4) 
											| (((statusCarControlGear & 0x0003) - 1) << 3) 
											| ((statusCarControlMode == 2 ? 1 : 0) << 2)
											| ((statusCarControlMode == 1 ? 1 : 0) << 1)
											| ((statusCarRunaway & 0x0001));
											
		//Ԥ��
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
	//��ѹ���
	//�����ع��ţ����Ƶ�ʣ����� 4Hz���͵�ѹ 10Hz
	//������ѹ��3.5 * 3 = 10.5V����ʱ 1s ����
	//���������������� ���Ρ� 10Hz��
	volatile uint16_t voltageAdcValue[2] = {0};		//ADCֵ
	float voltageVoltage = 0.0;			//ʵ�ʵ�ѹֵ
	float voltageLow = 10.5, voltageSeriousLow = 10.0;				//������ѹ
	uint32_t voltageLowTime = 0, voltageSeriousLowTime = 0;		//�͵�ѹ����
	float voltageCurrent = 0.0;			//����ֵ
	uint32_t voltageCurrentCalibration = 0;
	
	//�� ����
	uint16_t voltageCarHeadLight = 0, voltageCarBuzzer = 0;
	
	//У׼ADC
	HAL_ADCEx_Calibration_Start(&hadc1);
	osDelay(2);
	
	//��DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)voltageAdcValue, 2);
	
//	//У׼ ����ADCֵ
//	for(uint8_t i = 0; i < 100; i++)
//		voltageCurrentCalibration += voltageAdcValue[0], osDelay(1);
//	voltageCurrentCalibration = (uint32_t)((float)voltageCurrentCalibration / 100 + 0.5);
	
	//�����ʼ��
	SectionPeripheralReInit();
	
	//�Լ�
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
		//���
		xQueuePeek(carLightHandle, &voltageCarHeadLight, 1);
		HAL_GPIO_WritePin(HeadLight_GPIO_Port, HeadLight_Pin, voltageCarHeadLight);
		
		//����ʵ�ʵ�ѹֵ������д��Ϣ
		voltageVoltage = voltageAdcValue[1] * 0.00349 + 0.005;
		xQueueOverwrite(carVoltageHandle, &voltageVoltage);
		
		//����ʵ�ʵ���ֵ������д��Ϣ
		voltageCurrent = (voltageAdcValue[0]) * 0.00533 + 0.005;
		xQueueOverwrite(carCurrentHandle, &voltageCurrent);
		
		//���ص͵�����
		if(voltageVoltage > voltageSeriousLow)
			voltageSeriousLowTime = xTaskGetTickCount();
		
		//�͵�ѹ��
		if(voltageVoltage > voltageLow)
			voltageLowTime = xTaskGetTickCount();
		
		//������
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
		
		//���� Debug ����
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
