#include "KEY.h"
#include "LED.h"

Key key1;
Key key2;
KEY_status key_status;

/*
使用按键时
在main函数初始化：
	Key_Struct_Init();
	Key_EventCallback_Listen(&key1,HandleKeyEvent1);
	Key_EventCallback_Listen(&key2,HandleKeyEvent2);

使用定时器来进行按键检测如TIM15  配置10kHZ
使能中断，加上：
	HAL_TIM_Base_Start_IT(&htim15);   //使能中断
//	HAL_TIM_Base_Stop(&htim15);       //关闭中断

然后在it.c里的
void TIM1_BRK_TIM15_IRQHandler(void)
加入按键检测：
	//按键检测   1ms一次
	static uint16_t count;
	count++;
	if(count>=10)
	{
		Key_Presstime(&key1,10);
		Key_Presstime(&key2,10);
		count=0;
	}		

完成上述即可正常使用(将其放入PID的定时中断即可)

*/



void Key_Struct_Init()
{
	key1.id=1;
	key1.longPressTime=600;
	key1.last_status=KeyIO_Read(1);
	
	key2.id=2;
	key2.longPressTime=600;
	key2.last_status=KeyIO_Read(2);
}

bool KeyIO_Read(uint8_t id)
{
	switch(id)
	{
		case 1: if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==GPIO_PIN_SET) return 1;
			    else return 0;
		case 2:if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==GPIO_PIN_SET) return 1;
			    else return 0;
		default:
			return false;
		
	}
}

//判断按键按下的时长
void Key_Presstime(Key *key,uint16_t use_time)
{
	key->timer+=use_time;
	bool Key_Status=KeyIO_Read(key->id);
	if(key->last_status!=Key_Status)
	{
		if(Key_Status==1)
		{
			key->KeyEvent(up);
			if(key->timer - key->presstime >key->longPressTime)
				key->KeyEvent(long_press);
			else
				key->KeyEvent(click);
		}
		else
		{
			key->KeyEvent(down);
			key->presstime = key->timer;
		}
		key->last_status=Key_Status;
	}
}

//放在中断里面用于计次判断长短按，也可以放在主循环
void Key_use(void)
{
	static uint16_t count;
	count++;
	if(count>=10)   //对应频率选择计数值
	{
		Key_Presstime(&key1,10);
		Key_Presstime(&key2,10);
		count=0;
	}		
}

//按键1回调函数，想执行功能在此处更改
void HandleKeyEvent1(KEY_status key_status)
{
	switch(key_status)
	{
		case up:
			break;
		case down:
			break;
		case long_press:
//			LED_OFF();
			break;
		case click:
//			pid_m1.Id_aim += 0.1f;
//			LED_ON();
			break;
	}
}

//按键2回调函数，想执行功能在此处更改
void HandleKeyEvent2(KEY_status key_status)
{
	switch(key_status)
	{
		case up:
			break;
		case down:
			break;
		case long_press:
//			LED_ON();
			break;
		case click:
//			pid_m1.Id_aim -= 0.1f;
//			LED_OFF();
			break;
	}
}



//按键回调函数
void Key_EventCallback_Listen(Key *key,EventCallback callback)
{

	key->KeyEvent = callback;
	//当传入&key1, HandleKeyEvent1时等于下句 
	//key1.KeyEvent = HandleKeyEvent1;  // 把 HandleKeyEvent1 函数地址存入结构体 
	//因此在Key_Presstime函数里面，key->KeyEvent(down);会调用HandleKeyEvent1函数，
	//因为key1.KeyEvent已经改变，key1.KeyEvent = HandleKeyEvent1;只是为了调用该函数
}



