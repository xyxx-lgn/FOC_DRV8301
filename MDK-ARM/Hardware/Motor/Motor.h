#ifndef __Motor_H
#define __Motor_H		


//头文件包含
#include "ALL_H.h"


extern AdcValue adcvalue;              //ADC采样变量
extern AllFlag allflag;                //标志位变量
extern Encoder_Struct encoder_str;     //编码器结构体
extern uint16_t ADC1InjectDate[3];     //注入组采样数组
extern float angle_rad;

void Data_Init(void);
void Adcpro(AdcValue *adcvalue,AllFlag *allflag,uint16_t *adc_raw);
void Encoderpro(Encoder_Struct *encoder_str,AllFlag *allflag);
void Modepro(Encoder_Struct *encoder_str,AllFlag *allflag);     
void Data_Treating(void);
#endif
