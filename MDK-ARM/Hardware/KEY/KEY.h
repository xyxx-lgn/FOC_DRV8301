#ifndef __KEY_H
#define __KEY_H		


//头文件包含
#include "ALL_H.h"

typedef enum
{
	up,           //抬起
	down,         //按下
	long_press,   //长按
	click         //短按
}KEY_status; 
//EventCallback:void:函数指针返回数据类型；Event：函数指针形参数据类型
typedef void (*EventCallback)(KEY_status);  //自定义回调函数声明

typedef struct
{
	uint8_t id;               //按键号
	bool last_status;         //上一次按键状态
	uint16_t timer;           //计数
	uint16_t presstime;       //按下时长
	uint16_t longPressTime;   //长按的时长
	EventCallback KeyEvent;   //事件回调函数
}Key;
extern Key key1;
extern Key key2;
extern PID pid_m1;                     //PID参数结构体

void Key_Struct_Init(void);
bool KeyIO_Read(uint8_t id);
void Key_Presstime(Key *key,uint16_t use_time);
void Key_use(void);
void Key_EventCallback_Listen(Key *key,EventCallback callback);  //将回调函数和按键绑定
void HandleKeyEvent1(KEY_status key_status);     //自定义回调函数
void HandleKeyEvent2(KEY_status key_status);
#endif
