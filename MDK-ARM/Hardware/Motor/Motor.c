#include "Motor.h"
#include "MT6701.h"
#include "FOC.h"
#include "PID.h"
#include "KEY.h"

void Data_Init()
{
	//adcvalue结构体初始化
	adcvalue.Ia_Sample=0.0f;
	adcvalue.Ib_Sample=0.0f;
	adcvalue.Udc_Sample=0.0f;           //母线电压采样值
	adcvalue.Ia_offect=1.65f;
	adcvalue.Ib_offect=1.65f;
	adcvalue.Ia=0.0f;
	adcvalue.Ib=0.0f;
	adcvalue.Ic=0.0f;
	adcvalue.Udc=0.0f;
	adcvalue.Iadc_count=0;
	adcvalue.Gain_I=4.8f;        //电流增益倍数 Gain*Rs=80*0.02=1.6  
	
	//allflag结构体初始化
	allflag.Drv8301_flag=0;      //Drv8301初始化标志位，0代表未初始化，1代表初始化成功，2代表初始化失败   
	allflag.Adc_Adjust_flag=0;   //Adc校准标志位，0代表未校准，1代表校准完成，2代表过流
	allflag.Zero_flag=1;         //编码器零点校准标志位,0代表未校准，1代表校准完成  
	allflag.Encoder_flag=0;      //编码器模式 1.编码器开环控制 2.编码器闭环控制
	allflag.Mode_flag=0;         //电机控制模式选择
	
	//svpwm_str结构体初始化
	svpwm_str.Udc=12.0f;
	svpwm_str.U1=0.0f;
	svpwm_str.U2=0.0f;
	svpwm_str.U3=0.0f;
	svpwm_str.A=0;
	svpwm_str.B=0;
	svpwm_str.C=0;
	svpwm_str.N=0;
	svpwm_str.Sector=0;
	svpwm_str.X=0.0f;
	svpwm_str.Y=0.0f;
	svpwm_str.Z=0.0f;
	svpwm_str.Ts=8400;      //PWM寄存器值
	svpwm_str.T4=0.0f;
	svpwm_str.T6=0.0f;
	svpwm_str.T4_temp=0.0f;
	svpwm_str.T6_temp=0.0f;
	svpwm_str.Ta=0.0f;
	svpwm_str.Tb=0.0f;
	svpwm_str.Tc=0.0f;
	svpwm_str.PWMA=0;
	svpwm_str.PWMB=0;
	svpwm_str.PWMC=0;
	
	//编码器结构体
	encoder_str.motordir=1;              //电机方向 取值{0,1}
	encoder_str.r_s_speed=0;              //开环设置的转速，单位：圈/秒(指的是机械角度)
	encoder_str.Encoder_raw=0;
	encoder_str.Encoder_raw=0;  		  //编码器原始值
	encoder_str.Encoder_old_raw=0;        //上一次编码器原始值
	encoder_str.Encoder_raw_erro=0;       //编码器误差值
	encoder_str.Encoder_raw_erro_last=0; //编码器上一次误差值
	encoder_str.Encoder_raw_sum=0;        //编码器原始值和
	encoder_str.Shaft_Angle=0.0f;         //机械角度
	encoder_str.Elect_Angle=0.0f;         //电角度
	encoder_str.Encoder_Mode1_Angle=0.0f; //模式1临时角度变量
	encoder_str.Return_Angle=0.0f;        //真实返回角度(输出给SVPWM)
	encoder_str.Zero_Angle_cal=0.0f;      //零点偏移角度计算中间变量
	encoder_str.Zero_Angle=344.64f;          //零点偏移角度
	encoder_str.zero_count=0;             //编码器方向校准和零点校准
	
	//PID结构体
	//下面为电流环参数
	pid_m1.Kp_iq=0.0924f;         //iq的Kp值        相电感*350*2*pi=0.000042*350*2*pi=0.0924f   里面的350是指电角度多少转每秒，换算为350/7*60=3000转每分(机械角度)
	pid_m1.Ki_iq=0.01759f;         //iq的Ki值        相电阻*350*2*pi/电流环执行频率=0.08*350*2*pi/10000=0.01759f

	pid_m1.Iq_aim=0.0f;        //iq目标值
	pid_m1.Iq_current=0.0f;    //iq实际值
	pid_m1.erro_iq=0.0f;       //PI控制器里面的误差，等于目标值-实际值
	pid_m1.erro_iq_sum=0.0f;   //Ki的积分项
	pid_m1.Uq=0.0f;            //输出的Uq值
	
	pid_m1.Ki_SumMax=0.0f;     //KI的积分限幅最大值  Udc/sqrt(3),一般再乘以0.9,但本工程出问题，具体看pid_m1.speed_out_max处注释
	pid_m1.Ialfa=0.0f;         //电流环帕克变换
	pid_m1.Ibeta=0.0f;
	pid_m1.Vmax=0.0f;          //电流环输出最大电压矢量

	pid_m1.Id_aim=0.0f;        //id目标值
	pid_m1.Id_current=0.0f;    //id实际值
	pid_m1.erro_id=0.0f;       //PI控制器里面的误差，等于目标值-实际值
	pid_m1.erro_id_sum=0.0f;   //Ki的积分项
	pid_m1.Ud=0.0f;            //输出的Ud值
	
	//速度环参数
	pid_m1.Kp_speed=0.005f;      //速度环Kp值       0.005f
	pid_m1.Ki_speed=0.000001f;      //速度环Ki值       0.000001f
	
	pid_m1.Speed_aim=0.0f;     //目标速度，单位：转/分 
	pid_m1.Speed_last=0.0f;    //实际速度
	pid_m1.Speed_now=0.0f;     //实际速度
	pid_m1.Speed_show=0.0f;    //滤波后速度
	pid_m1.erro_speed=0.0f;    //速度误差
	pid_m1.erro_speed_sum=0.0f;//速度积分误差
	pid_m1.speed_out=0.0f;     //PI输出结果，输出为电流环Iq_aim
	
	pid_m1.speed_out_max=1.8f*0.6f; //速度环输出Iq限幅值   Imax=(3.3/2)/(G*S)=1.65/0.8=2.06, 一般取0.9，但是测试发现GVDD欠压有问题，可能电容原因，只能取1A，因此要乘系数为1/2.06
	pid_m1.speed_max=3000;      //速度设定限幅  [-360,360]
	pid_m1.speed_count=0;      //速度环计次
	
	//下面为位置环
	pid_m1.Kp_position=9.2f;        //位置环Kp值
	pid_m1.Ki_position=0.00005f;        //位置环Ki值 
	
	pid_m1.Position_aim=180.0f;       //目标位置，取值[0,360]
	pid_m1.erro_positon=0.0f;       //位置误差=Position_aim-Shaft_Angle
	pid_m1.erro_position_sum=0.0f;  //位置积分误差 
	pid_m1.position_out=0.0f;       //位置环PI输出，输出为速度环的Speed_aim  
	
	pid_m1.position_out_max=1000.0f;   //位置环的输出限幅
}


/********************
ADC数据采样处理
	电流计算公式
		Vo=1/2Vref-G*V(Sn-Sp)
		Vrs=V(Sn-Sp)
		即-(Vo-1/2Vref)/G=Vrs
		因此I=Vrs/Rs(采样电阻值)
		I=-(Vo-1/2Vref)/(G*Rs)   ,此处的Vref最好是先校准，一般不是1.65V
********************/
void Adcpro(AdcValue *adcvalue,AllFlag *allflag,uint16_t *adc_raw)
{
	adcvalue->Ia_Sample = adc_raw[0];  //A相adc采样值
	adcvalue->Ib_Sample = adc_raw[1];  //B相adc采样值
	adcvalue->Udc_Sample = adc_raw[2]; //Udc电压adc采样值
	//1.先对电流ADC偏置值校准
	if(allflag->Drv8301_flag==1 && allflag->Adc_Adjust_flag==0)    //电流adc未校准
	{
		if(adcvalue->Iadc_count<30000)  //PWM频率10K，进行30000次，用时3s
		{
			adcvalue->Ia = adcvalue->Ia_Sample*3.3f/4096;
			adcvalue->Ib = adcvalue->Ib_Sample*3.3f/4096;
			adcvalue->Ia_offect = adcvalue->Ia_offect*0.95f+adcvalue->Ia*0.05f;      //一阶滤波
			adcvalue->Ib_offect = adcvalue->Ib_offect*0.95f+adcvalue->Ib*0.05f;
			adcvalue->Iadc_count++;
		}
		else
		{
			allflag->Adc_Adjust_flag = 1;     //校准完成
			adcvalue->Iadc_count = 0;
		}
	}
	else if(allflag->Drv8301_flag==1 && allflag->Adc_Adjust_flag==1)
	//ADC系数转换 
	{
		adcvalue->Ia = -((adcvalue->Ia_Sample*3.3f/4096-adcvalue->Ia_offect)/adcvalue->Gain_I);
		adcvalue->Ib = -((adcvalue->Ib_Sample*3.3f/4096-adcvalue->Ib_offect)/adcvalue->Gain_I);
		adcvalue->Ic = -(adcvalue->Ia+adcvalue->Ib);
		//(Udc_Sample-504)/4096*3.3 * (4.99+34.8)/4.99 =Udc,这个504是我自己给的一个补偿值，这里有点问题，理论上应该是减0，不减的话电压太大已经大于供电电压了
		//0.00642432326=1/4096*3.3 * (4.99+34.8)/4.99
		adcvalue->Udc = (adcvalue->Udc_Sample-0)*0.00642432326f;
        if(adcvalue->Udc>12.6f)   //防止意外超出
			adcvalue->Udc=12.6f;
		svpwm_str.Udc = adcvalue->Udc;
		pid_m1.Ki_SumMax = adcvalue->Udc*one_sqrt3*0.6f;     //0.2522404f
//		pid_m1.Vmax = adcvalue->Udc*one_sqrt3;
	}
}

/********************
磁编码器数据处理
********************/
void Encoderpro(Encoder_Struct *encoder_str,AllFlag *allflag)
{
//	encoder_str->Encoder_raw = (int16_t)MT6701_ReadRaw();                           //编码器原始数据获取 0-16384
	encoder_str->Encoder = MT6701_ReadRaw();
	if(encoder_str->motordir ==1)
		encoder_str->Encoder_raw = 16384-encoder_str->Encoder;
	else
		encoder_str->Encoder_raw = encoder_str->Encoder;
	encoder_str->Shaft_Angle = (float)encoder_str->Encoder_raw * 0.0219726f - 14.084125f;   //机械角度获取 14.084125 
	if(encoder_str->Shaft_Angle>360)
		encoder_str->Shaft_Angle -= 360.0f;
	else if(encoder_str->Shaft_Angle<0)
		encoder_str->Shaft_Angle += 360.0f;
	
	encoder_str->Elect_Angle = ElectAngle_Limit(encoder_str->Shaft_Angle); //电角度获取，0-360
	if(encoder_str->Elect_Angle>360)
		encoder_str->Elect_Angle -= 360;
	else if(encoder_str->Elect_Angle<0)
		encoder_str->Elect_Angle +=360;
	
	
	encoder_str->Encoder_raw_erro = (encoder_str->Encoder_raw-encoder_str->Encoder_old_raw);
	//16384/2，判断正反转过零点
	if(encoder_str->Encoder_raw_erro>8192)          //反转过零点
		encoder_str->Encoder_raw_erro = encoder_str->Encoder_raw-encoder_str->Encoder_old_raw-16384;
	else if(encoder_str->Encoder_raw_erro<-8192)    //正转过零点  
		encoder_str->Encoder_raw_erro = 16384-encoder_str->Encoder_old_raw+encoder_str->Encoder_raw;
	
	encoder_str->Encoder_raw_sum += (float)encoder_str->Encoder_raw_erro;
	
	encoder_str->Encoder_old_raw = encoder_str->Encoder_raw;               //保留上一次编码器数值
	
	
	if(allflag->Adc_Adjust_flag==1 && allflag->Zero_flag==0)               //ADC电流采样校准完成后进行磁编码器校准
	{
		SVPWM(0,1.5,0,&svpwm_str);
		encoder_str->zero_count++;
		if(encoder_str->zero_count>8000)        //让对齐0.8s后再校准
		{
			encoder_str->Zero_Angle_cal += encoder_str->Shaft_Angle;
			if(encoder_str->zero_count>=10000)
			{
				allflag->Zero_flag = 1;
				encoder_str->zero_count = 0;
				encoder_str->Zero_Angle = encoder_str->Zero_Angle_cal/2000.0f;
			}
		}	
	}
	
	if(allflag->Encoder_flag==1)  //编码器模式1：开环角度自增(方向由motordir决定)
	{
		encoder_str->motordir = 1;                                        //(-1为逆时针正向旋转，1为顺时针反向旋转)
		encoder_str->r_s_speed = 1;
		encoder_str->Encoder_Mode1_Angle = encoder_str->Encoder_Mode1_Angle+encoder_str->motordir*encoder_str->r_s_speed*0.036f;  
		if(encoder_str->Encoder_Mode1_Angle>360)
			encoder_str->Encoder_Mode1_Angle -= 360.0f;
		else if(encoder_str->Encoder_Mode1_Angle<0)
			encoder_str->Encoder_Mode1_Angle += 360.0f;
		encoder_str->Encoder_Mode1_Angle = encoder_str->Encoder_Mode1_Angle;  //机械角度 344.640015 37.242882f
		encoder_str->Return_Angle = ElectAngle_Limit(encoder_str->Encoder_Mode1_Angle);;
		if(encoder_str->Return_Angle>360)
			encoder_str->Return_Angle -= 360.0f;
		else if(encoder_str->Return_Angle<0)
			encoder_str->Return_Angle += 360.0f;
		

	}
	else if(allflag->Encoder_flag==2) //编码器模式2：闭环角度控制  返回校准后的电角度值
	{
		encoder_str->Return_Angle = encoder_str->Elect_Angle;
//		if(encoder_str->Return_Angle>360)
//			encoder_str->Return_Angle -= 360;
//		else if(encoder_str->Return_Angle<0)
//			encoder_str->Return_Angle +=360;
	}
	
	

}

/********************
电机控制模式选择
********************/
void Modepro(Encoder_Struct *encoder_str,AllFlag *allflag)     
{
	//电压开环控制
	if(allflag->Mode_flag==1)
	{
		allflag->Encoder_flag = 1;
		SVPWM(1,0,encoder_str->Return_Angle,&svpwm_str);
	}
	//电流环控制
	else if(allflag->Mode_flag==2)
	{
		allflag->Encoder_flag = 2;
//		pid_m1.Iq_aim = 0.2f;
//		pid_m1.Id_aim = 0.0f;
		PID_I_Control(&pid_m1);
		SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//速度环-电流环
	else if(allflag->Mode_flag==3)
	{
		allflag->Encoder_flag = 2;
//		pid_m1.Speed_aim = 120;
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
	//位置环-速度环-电流环
	else if(allflag->Mode_flag==4)
	{
		allflag->Encoder_flag = 2;
//		pid_m1.Position_aim=120.0f;
		PID_Position_Control(&pid_m1,encoder_str);    //位置环
		pid_m1.Speed_aim = pid_m1.position_out;
		PID_Speed_Control(&pid_m1,encoder_str);      //速度环
		pid_m1.Iq_aim = pid_m1.speed_out;
		pid_m1.Id_aim = 0;
		PID_I_Control(&pid_m1);
		SVPWM(pid_m1.Uq,pid_m1.Ud,encoder_str->Return_Angle,&svpwm_str);
	}
}

/********************
ADC数据采样处理和编码器采样数据处理
********************/
void Data_Treating()
{
	
	Adcpro(&adcvalue,&allflag,ADC1InjectDate);
	Encoderpro(&encoder_str,&allflag);
	Clark_Park(&adcvalue,&encoder_str,&pid_m1);   //用于计算电流环的Iq和Id

	allflag.Mode_flag=2;
  if(allflag.Adc_Adjust_flag==1 && allflag.Zero_flag==1)
  {
	Modepro(&encoder_str,&allflag);
  }
	Key_use();
}
