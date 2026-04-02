/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "CommSpiSensor.h"
#include "CommPCS.h"
#include "fdcan.h"
#include "usart.h"
#include "Sequence.h"
#include "stm32h7xx_hal.h"
#include "./../Drivers/SYS/sys_command_line.h"
#include "rtc.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t text[20];
RTC_DateTypeDef sdatestructureget;
RTC_TimeTypeDef stimestructureget;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

char tData = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t CanRxTaskHandle;
const osThreadAttr_t CanRxTask_attributes = {
  .name = "AdcTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CanTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t CAN2TaskHandle;
const osThreadAttr_t CAN2Task_attributes = {
  .name = "PcsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t SeqTaskHandle;
const osThreadAttr_t SeqTask_attributes = {
  .name = "SeqTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void CANRxTask(void *argument);
void CAN2Task(void *argument);
void CANTask(void *argument);
void SeqTask(void *argument);
void LCD_Timer();

static void RTC_CalendarShow(RTC_DateTypeDef *sdatestructureget,RTC_TimeTypeDef *stimestructureget)
{
  /* 必须?��?��?��?��?��?��?��?��?�� 不然会�?�致下次RTC不能读取 */
  /* Both time and date must be obtained or RTC cannot be read next time */
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, sdatestructureget, RTC_FORMAT_BIN);
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  CanRxTaskHandle = osThreadNew(CANRxTask, NULL, &CanRxTask_attributes);
  CAN2TaskHandle  = osThreadNew(CAN2Task, NULL, &CAN2Task_attributes);
  CANTaskHandle   = osThreadNew(CANTask, NULL, &CANTask_attributes);
  SeqTaskHandle   = osThreadNew(SeqTask, NULL, &SeqTask_attributes);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	TcpModbusCtrl();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



void CANRxTask(void *argument)
{
  //uint16_t fan_map = 0xFABD; /* 0x0B = 00001011 */

  for(;;)
  {
	//MAX22530_Update(1);
	CanRxData();
	CanRxData2();
	osDelay(1);
  }
}

void CAN2Task(void *argument)
{

  for(;;)
  {
    //while(HAL_UART_Receive(&huart1, &tData, 1, 10) != HAL_OK){osDelay(1);}
	//QUEUE_IN(cli_rx_buff, tData);
    //CLI_RUN();

	// SST CAN
	CanTxData2();
	osDelay(1);
  }
}

void CANTask(void *argument)
{
  for(;;)
  {
	// PCS AND SENSOR BOARD
	CanTxData();
	osDelay(1);
  }
}

void SeqTask(void *argument)
{
  for(;;)
  {
	LCD_Timer();
	SequenceMain();
	osDelay(2000);
  }
}


void LCD_Timer()
{
	RTC_CalendarShow(&sdatestructureget,&stimestructureget);

	if (stimestructureget.Seconds % 2 == 1)
		sprintf((char *)&text,"RunTime: %02d:%02d:%02d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	else
		sprintf((char *)&text,"RunTime: %02d %02d %02d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	LCD_ShowString(4, 58, 160, 16, 16, text);

}
/* USER CODE END Application */

