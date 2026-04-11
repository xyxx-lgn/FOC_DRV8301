/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ALL_H.h"
#include "KEY.h"
#include "LED.h"
#include "MT6701.h"
#include "DRV8301.h"
#include "Start.h"
#include "Motor.h"
#include "FOC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t angle_raw=0;
float angle=0;
uint8_t field_status=0;

uint16_t ADC1InjectDate[3];     //注入组采样数组
uint16_t ADC1RegularDate[3];    //规则组采样数组 
AdcValue adcvalue;              //ADC采样变量
AllFlag allflag;                //标志位变量
SVPWM_Struct svpwm_str;         //SVPWM结构体
Encoder_Struct encoder_str;     //编码器结构体
PID pid_m1;                     //PID参数结构体
float angle_rad;
float Uq_set=0;

uint16_t status0;
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
    //按键初始化
	Key_Struct_Init();
	Key_EventCallback_Listen(&key1,HandleKeyEvent1);
	Key_EventCallback_Listen(&key2,HandleKeyEvent2);
	
//	DRV8301_ENGATE_OFF();     
//	HAL_Delay_us(25); // 等待电源稳定
//	DRV8301_ENGATE_ON();  
//	HAL_Delay_us(300);

	//中断初始化
	Enable_IT();   

	while(allflag.Drv8301_flag!=1)
	{
		DRV8301_Init();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(pid_m1.Uq>pid_m1.position_out_max)
//	  {
		status0 = DRV8301_ReadReg(0x00);
//	  }
	  
//	encoder_str.Encoder = MT6701_ReadRaw();
	  //MT6701_Read_ALL(&angle_raw, &angle, &field_status);
	  
//	  angle_rad+=0.01f;
//	  if(allflag.Adc_Adjust_flag==1 && allflag.Zero_flag==1)
//		  SetPhaseVoltage(1,0,ElectAngle_Turn(angle_rad,7)); 
//	  printf("%f,%f,%f,%f,%f \r\n",adcvalue.Ia,adcvalue.Ib,adcvalue.Ic,encoder_str.Return_Angle,pid_m1.Uq);
//	  printf("%f,%f,%f,%f \r\n",pid_m1.Uq,pid_m1.Ud,pid_m1.Iq_current,pid_m1.Id_current);
//	  printf("%f,%f,%f,%f \r\n",adcvalue.Ia,adcvalue.Ib,adcvalue.Udc,adcvalue.Ia_offect);
//	  printf("%d,%d,%d,%f,%f \r\n",encoder_str.Encoder_raw_sum,encoder_str.Encoder_raw_erro,encoder_str.Encoder_raw,pid_m1.Iq_current,pid_m1.Id_current);
	  
//	  printf("%d,%f,%f,%f \r\n",encoder_str.Encoder_raw,encoder_str.Shaft_Angle,encoder_str.Elect_Angle,encoder_str.Zero_Angle);
//	  printf("angle=%.1f  iq_aim=%.2f  iq=%.2f  Uq=%.2f\r\n",
//       encoder_str.Elect_Angle, pid_m1.Iq_aim,
//       pid_m1.Iq_current, pid_m1.Uq);
//	  printf("%d,%f,%f,%f,%f \r\n",encoder_str.Encoder_raw,pid_m1.Speed_now,pid_m1.speed_out,pid_m1.Iq_current,pid_m1.Id_current);
//	  printf("%d,%f,%f,%f \r\n",encoder_str.Encoder_raw,encoder_str.Shaft_Angle,encoder_str.Elect_Angle,pid_m1.position_out);
//	  printf("%d,%d,%d \r\n",svpwm_str.PWMA,svpwm_str.PWMB,svpwm_str.PWMC);
//	  if(angle_rad>360)
//		  angle_rad-=360;
//	  else if(angle_rad<0)
//		  angle_rad+=360;

	  
//	  printf("%d,%d,%d,%f,%f,%f \r\n",svpwm_str.PWMA,svpwm_str.PWMB,svpwm_str.PWMC,adcvalue.Ia,adcvalue.Ib,adcvalue.Ic);
//	  printf("%f,%f,%f,%f \r\n",encoder_str.Return_Angle,encoder_str.Elect_Angle,encoder_str.Shaft_Angle,encoder_str.Zero_Angle);
//	  HAL_Delay(100);
	  
	  
//	  if (cnt<10)
//	  {
//		  cnt++;
//		  uint16_t status0 = DRV8301_ReadReg(0x01);
//		  printf("%d\n",status0);
//		  if (status0 & 0x0001) // 检查FAULT位
//			printf("Fail"); // 初始化失败 
//		  else
//			printf("Success");
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
