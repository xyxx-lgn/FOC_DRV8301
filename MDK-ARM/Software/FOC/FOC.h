#ifndef __FOC_H
#define __FOC_H		


//头文件包含
#include "ALL_H.h"



extern AdcValue adcvalue;              //ADC采样变量
extern Encoder_Struct encoder_str;     //编码器结构体
extern SVPWM_Struct svpwm_str;         //SVPWM结构体
extern PID pid_m1;                     //PID参数结构体

float ElectAngle_Turn(float RawAngle,uint8_t pole);
float ElectAngle_Limit(float Angle);
void SetPwm(float Ua,float Ub,float Uc);
void SetPhaseVoltage(float Uq,float Ud,float elect_angle);
void Clark_Park(AdcValue *adcvalue,Encoder_Struct *encoder_str,PID *pid);
void SVPWM(float Uq,float Ud,float Angle,SVPWM_Struct *svpwm_str);
#endif
