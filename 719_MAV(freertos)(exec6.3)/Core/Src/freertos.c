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
#include "gpio.h"
#include "usart.h"
#include "719_FC.h"
#include "Drv_Imu.h"
#include "Drv_ws2812.h"
#include "Drv_ppm.h"
#include "719_User_Tasks.h"
#include "Drv_Echo.h"

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
/* Definitions for ClassisTask */
osThreadId_t ClassisTaskHandle;
const osThreadAttr_t ClassisTask_attributes = {
  .name = "ClassisTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TriflesTask */
osThreadId_t TriflesTaskHandle;
const osThreadAttr_t TriflesTask_attributes = {
  .name = "TriflesTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ThreadsTimer */
osTimerId_t ThreadsTimerHandle;
const osTimerAttr_t ThreadsTimer_attributes = {
  .name = "ThreadsTimer"
};
/* Definitions for User_TasksTimer */
osTimerId_t User_TasksTimerHandle;
const osTimerAttr_t User_TasksTimer_attributes = {
  .name = "User_TasksTimer"
};
/* Definitions for LedTimer */
osTimerId_t LedTimerHandle;
const osTimerAttr_t LedTimer_attributes = {
  .name = "LedTimer"
};
/* Definitions for ChassisMutex */
osMutexId_t ChassisMutexHandle;
const osMutexAttr_t ChassisMutex_attributes = {
  .name = "ChassisMutex"
};
/* Definitions for SensorMutex */
osMutexId_t SensorMutexHandle;
const osMutexAttr_t SensorMutex_attributes = {
  .name = "SensorMutex"
};
/* Definitions for TriflesMutex */
osMutexId_t TriflesMutexHandle;
const osMutexAttr_t TriflesMutex_attributes = {
  .name = "TriflesMutex"
};
/* Definitions for ClassisEvent */
osEventFlagsId_t ClassisEventHandle;
const osEventFlagsAttr_t ClassisEvent_attributes = {
  .name = "ClassisEvent"
};
/* Definitions for SensorEvent */
osEventFlagsId_t SensorEventHandle;
const osEventFlagsAttr_t SensorEvent_attributes = {
  .name = "SensorEvent"
};
/* Definitions for TriflesEvent */
osEventFlagsId_t TriflesEventHandle;
const osEventFlagsAttr_t TriflesEvent_attributes = {
  .name = "TriflesEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Sensor_Tsak(void *argument);
void Trifles_Task(void *argument);
void ThreadsCallback(void *argument);
void User_TasksCallback(void *argument);
void LedCallback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
/******************************************************************************************
模块名：统一任务调度模块
作者：	719-张木悦，程子峻
*******************************************************************************************/
//====================================任务列表============================================//	
/*底盘控制任务：							ClassisTaskHandle
 *传感器感知任务：						SensorMutexHandle
 *杂事处理任务：							TriflesMutexHandle
 */
//====================================任务列表============================================//
//
//====================================互斥量列表==========================================//	
/*底盘数据容器保护互斥量：		ChassisMutexHandle	
 *传感器数据容器保护互斥量：	SensorMutexHandle
 *杂事数据容器保护互斥量：		TriflesMutexHandle
 */
//====================================互斥量列表==========================================//
//
//====================================事件标志组列表======================================//	
/*底盘事件标志组：						ClassisEventHandle
 *传感器事件标志组：					SensorEventHandle
 *杂事事件标志组：						TriflesEventHandle
 */
//====================================事件标志组列表======================================//
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of ChassisMutex */
  ChassisMutexHandle = osMutexNew(&ChassisMutex_attributes);

  /* creation of SensorMutex */
  SensorMutexHandle = osMutexNew(&SensorMutex_attributes);

  /* creation of TriflesMutex */
  TriflesMutexHandle = osMutexNew(&TriflesMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of ThreadsTimer */
  ThreadsTimerHandle = osTimerNew(ThreadsCallback, osTimerPeriodic, NULL, &ThreadsTimer_attributes);

  /* creation of User_TasksTimer */
  User_TasksTimerHandle = osTimerNew(User_TasksCallback, osTimerPeriodic, NULL, &User_TasksTimer_attributes);

  /* creation of LedTimer */
  LedTimerHandle = osTimerNew(LedCallback, osTimerPeriodic, NULL, &LedTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ClassisTask */
  ClassisTaskHandle = osThreadNew(StartDefaultTask, NULL, &ClassisTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(Sensor_Tsak, NULL, &SensorTask_attributes);

  /* creation of TriflesTask */
  TriflesTaskHandle = osThreadNew(Trifles_Task, NULL, &TriflesTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of ClassisEvent */
  ClassisEventHandle = osEventFlagsNew(&ClassisEvent_attributes);

  /* creation of SensorEvent */
  SensorEventHandle = osEventFlagsNew(&SensorEvent_attributes);

  /* creation of TriflesEvent */
  TriflesEventHandle = osEventFlagsNew(&TriflesEvent_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	osTimerStart(ThreadsTimerHandle, 1);
	osTimerStart(User_TasksTimerHandle, 20);
	osTimerStart(LedTimerHandle, 2000);
  for(;;)
  {   
		Control_Server();
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Sensor_Tsak */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sensor_Tsak */
void Sensor_Tsak(void *argument)
{
  /* USER CODE BEGIN Sensor_Tsak */
  /* Infinite loop */
  for(;;)
  {
		Get_ImuData_Server();
		get_rc_data();
    osDelay(10);
  }
  /* USER CODE END Sensor_Tsak */
}

/* USER CODE BEGIN Header_Trifles_Task */
/**
* @brief Function implementing the TriflesTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Trifles_Task */
void Trifles_Task(void *argument)
{
  /* USER CODE BEGIN Trifles_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END Trifles_Task */
}

/* ThreadsCallback function */
void ThreadsCallback(void *argument)
{
  /* USER CODE BEGIN ThreadsCallback */
	//用途：用于做类似于ANO那样的假多线程
	
  /* USER CODE END ThreadsCallback */
}

/* User_TasksCallback function */
void User_TasksCallback(void *argument)
{
  /* USER CODE BEGIN User_TasksCallback */
//	Vertical_Move(40, 1500);
	User_Tasks();
	Echo_init();			
  /* USER CODE END User_TasksCallback */
}

/* LedCallback function */
void LedCallback(void *argument)
{
  /* USER CODE BEGIN LedCallback */
	ws2812_Server();
  /* USER CODE END LedCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

