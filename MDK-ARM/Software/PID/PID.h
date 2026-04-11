#ifndef __PID_H
#define __PID_H		


//头文件包含
#include "ALL_H.h"

extern PID pid_m1;                     //PID参数结构体
extern Encoder_Struct encoder_str;     //编码器结构体

void PID_I_Control(PID *pid);
void PID_Speed_Control(PID *pid,Encoder_Struct *encoder_str);
void PID_Position_Control(PID *pid,Encoder_Struct *encoder_str);
#endif
