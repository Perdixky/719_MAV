/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Drv_ws2812.h"
#include "iic.h"
#include "Drv_wit_c_sdk.h"
#include "systerm.h"
#include "stdio.h"
#include "Drv_ppm.h"
#include "structconfig.h"
#include "719_Interface.h"
#include "719_FC.h"
#include "Drv_Imu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define ACC_UPDATE		0x01
//#define GYRO_UPDATE		0x02
//#define ANGLE_UPDATE	0x04
//#define MAG_UPDATE		0x08
//#define READ_UPDATE		0x80

//static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;

//static void AutoScanSensor(void);
//static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
//static void Delayms(uint16_t ucMs);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
	MotorPwm_Init();//电调校准
  /* USER CODE BEGIN 2 */
  /***********************************************************
  TIM1-------------           TIM2-------------
  TIM6-------------sys        TIM7-------------Cnt_TIM  
  TIM3-------------motor_pwm  TIM4-------------Cnt_TIM
  TIM5-------------Base_TIM   TIM8-------------PWM_DMA_WS2812（全彩灯）
  ***********************************************************/
   systerm_init();
	 Get_ImuData_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		printf("hello world\r\n");
//    Task_Schedule();
//		printf("angle:%.3f %.3f %.3f\n", fAngle[0], fAngle[1], fAngle[2]);
//			printf("%f\n",fAngle[2]);
//	  if(SBUS_FLAG)
//	  {
//		  SBUS_FLAG = 0;
//	      printf_CH();
//	  }
	  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//extern int AttControl_2_flag_s;
//extern int AttControl_1_flag_s;
//extern int LocControl_1_flag_s;
//extern int XYZVelocity_Set_flag;
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
//  static uint16_t led_flag_s = 0;	
//  static uint16_t tim_flag_s = 0;
//  static uint8_t FLAG1_s = 1;
//	static uint8_t AttControl_2_flag_s;		//速度环标志位，10节拍控制一次
//	static uint8_t AttControl_1_flag_s;		//角速度环标志位，5节拍控制一次
//	static uint8_t LocControl_1_flag_s;		//高度速度环标志位，20节拍控制一次
//	static FLOAT_ANGLE oular_angle_s;
//	static int flag_s[GroupOfFlag];
//	flag_s[unlock_f] = 0; 				//1<<unlock_f
//	flag_s[imu_init_f] = 0;				//1<<imu_init_f
//	flag_s[led_station_f] = 0;		//1<<led_station_f
//	flag_s[AttControl_1_f] = 0;		//1<<AttControl_1_f
//	flag_s[AttControl_2_f] = 0;		//1<<AttControl_2_f
//	flag_s[LocControl_1_f] = 0;		//1<<LocControl_1_f
	extern osEventFlagsId_t ClassisEventHandle;	//管理标志位公共资源：存储各个事件的标志位
//	uint32_t retval;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM5) {
//		uint32_t retval;
//		retval = osEventFlagsSet(ClassisEventHandle, 1<<unlock_f);
//		printf("标志位设置是否成功%d\r\n", retval);
//		Classis_Interface_in("b", &oular_angle_s);
//		Flag_Interface_in(flag_s);	//括号里不能填NULL，写就是寄，系统直接崩溃
//	  tim_flag_s++;
//	  led_flag_s++;							//IMU_Init_Flag，control_flag，led_station_flag， lock
//		AttControl_2_flag_s++;		//速度环标志位，10节拍控制一次
//		AttControl_1_flag_s++;		//角速度环标志位，5节拍控制一次
//		LocControl_1_flag_s++;		//高度速度环标志位，20节拍控制一次
//		XYZVelocity_Set_flag++;	//飞控系统速度目标量综合设定标志位，1节拍重新设定一次(干掉)

//	  if(oular_angle_s.rol <= 1.0 && oular_angle_s.pit <= 1.0 && tim_flag_s >= 5000 && FLAG1_s == 1)
//	  {
//		  FLAG1_s = 0;
//			flag_s[imu_init_f] = 1;
//	  }		  

//	  if(AttControl_1_flag_s == 5)
//	  {
//			AttControl_1_flag_s = 0;
//			flag_s[AttControl_1_f] = 1;
//	  }
//		
//	  if(AttControl_2_flag_s == 10)
//	  {
//			AttControl_2_flag_s = 0;
//			flag_s[AttControl_2_f] = 1;
//	  }
//		
//		  if(LocControl_1_flag_s == 20)
//	  {
//			LocControl_1_flag_s = 0;
//			flag_s[LocControl_1_f] = 1;
//	  }
//	  
//	  if(led_flag_s == 2000)
//	  {
//		  led_flag_s = 0;
//		  if(FLAG1_s == 0)
//				flag_s[led_station_f] = 1;
	  }
//		printf("定时器函数%d, %d, %d\r\n", flag_s[AttControl_1_f],flag_s[AttControl_2_f],  flag_s[LocControl_1_f]);
//		if(flag_s[AttControl_1_f]) osEventFlagsSet(ClassisEventHandle, 1<<AttControl_1_f); else osEventFlagsClear(ClassisEventHandle, 1<<AttControl_1_f);
//		if(flag_s[AttControl_2_f]) osEventFlagsSet(ClassisEventHandle, 1<<AttControl_2_f); else osEventFlagsClear(ClassisEventHandle, 1<<AttControl_2_f);
//		if(flag_s[LocControl_1_f]) osEventFlagsSet(ClassisEventHandle, 1<<LocControl_1_f); else osEventFlagsClear(ClassisEventHandle, 1<<LocControl_1_f);
//		retval = osEventFlagsSet(ClassisEventHandle, 1<<AttControl_1_f);
//		printf("返回值是否成功：%d\r\n",retval);
//  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
