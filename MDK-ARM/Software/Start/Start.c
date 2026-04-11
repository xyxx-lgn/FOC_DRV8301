#include "Start.h"
#include "Motor.h"

//UART1
uint8_t rx_buffer[100];//接收数组
uint8_t rx_len = 0; //接收到的数据长度
extern uint16_t ADC1RegularDate[3];    //规则组采样数组 
extern AdcValue adcvalue;
extern SVPWM_Struct svpwm_str;         //SVPWM结构体
//自定义us延时器，用TIM7作为us定时器
void HAL_Delay_us(uint16_t nus)
{
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	__HAL_TIM_ENABLE(&htim7);
	while (__HAL_TIM_GET_COUNTER(&htim7) < nus)
	{
	}
	__HAL_TIM_DISABLE(&htim7);
}



void Enable_IT(void)
{
	//1.初始化TIM15定时器中断
	HAL_TIM_Base_Start_IT(&htim15);   //使能中断
//	HAL_TIM_Base_Stop(&htim15);       //关闭中断
	
	
	//2.UART1重定向，接收和发送，中断与DMA开启
	//2-1.在uart.c加入重定向 

	/*
	  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
	  * 输入参数: 无
	  * 返 回 值: 无
	  * 说    明：无
	  */
//	int fputc(int ch, FILE *f)
//	{
//	  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
//	  return ch;
//	}
	 
	/*
	  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
	  * 输入参数: 无
	  * 返 回 值: 无
	  * 说    明：无
	  */
//	int fgetc(FILE *f)
//	{
//	  uint8_t ch = 0;
//	  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
//	  return ch;
//	}
	
	//2-2.接收和发送部分，在it.c里的void USART1_IRQHandler(void)加入下面注释部分

//	if (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)// 通过IDLE标志位判断接收是否结束
//	   { 
//		  __HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
//		  HAL_UART_DMAStop(&huart1);               
//		  rx_len = 100 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //计算出数据长度
//		  HAL_UART_Transmit_DMA(&huart1, rx_buffer,rx_len);//将收到的数据发送出去
//		  HAL_UART_Receive_DMA(&huart1,rx_buffer,100);//开启DMA接收，方便下一次接收数据
//	   }

    //2-3.中断与DMA开启
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);    
	HAL_UART_Receive_DMA(&huart1,rx_buffer,100);
	
	//3.定时器1PWM开启
	//开启定时器1
	HAL_TIM_Base_Start(&htim1);
	
	//打开PWM通道
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	//给初始值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4200);   //Duty=4200/8400
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4200);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,4200);
	
	//开启TIM1的OC4中断  在HAL里面要先勾选TIM1 update interrupt and TIM16 global interrupt
//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);   // 开中断标志位
//	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4); // 启动通道 4 并附中断
	
	//并写中断回调
	/* 每完成一次 CCR4 比较匹配就进来 -------------------------- */
	/*
	void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
	{
		if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   // 示例：翻转 LED
		}
	}
	*/
	
	//ADC1初始化，用于AB相电流采样和母线电压采样
	//HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC1RegularDate,3);   //轮询和注入二选一，注入速度更快，不开启DMA
	
	Data_Init();                             //ADC结构体初始化
	__HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);    //ADC注入通道中断    //还需要在it.c里面打开HAL_ADCEx_InjectedConvCpltCallback中断读取    
	HAL_ADCEx_InjectedStart(&hadc1);            //开启ADC注入采样
}
