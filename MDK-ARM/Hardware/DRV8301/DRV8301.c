#include "DRV8301.h"


extern AllFlag allflag;                       //标志位变量
extern uint16_t status0;
//======================================================================
// SPI通信核心函数
//======================================================================
/****************************
DRV8301写入函数：
	reg：寄存器地址
	data：写入数据，只有0x02和0x03能写
****************************/
void DRV8301_WriteReg(uint8_t reg, uint16_t data) 
{
	uint16_t txData = (0 << 15) | ((reg & 0x0F) << 11) | (data & 0x07FF);
	//uint16_t rxData;
  
	DRV8301_CS_ON();
	HAL_SPI_Transmit(&hspi3, (uint8_t*)&txData, 1, 100);
	DRV8301_CS_OFF();
}



/****************************
DRV8301读取函数：
	reg：寄存器地址,读取寄存器写入值为0x00,0x01,0x02,0x03
****************************/
uint16_t DRV8301_ReadReg(uint8_t reg)    
{
	uint16_t zerobuff = 0xffff;
	uint16_t txData = (1 << 15) | ((reg & 0x0F) << 11);
	uint16_t rxData = 0xbeef;
  
	
	DRV8301_CS_ON();
	HAL_SPI_Transmit(&hspi3, (uint8_t*)(&txData), 1, 1000);                  
	DRV8301_CS_OFF();

	
	
	DRV8301_CS_ON();
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)(&zerobuff), (uint8_t*)(&rxData), 1, 1000);
	DRV8301_CS_OFF();
	
//	printf("aaa");
//	printf("%x",rxData);      //验证数据是否正确
//	printf("aaa");
	
	
//	if(rxData==0xbeef)   //检查数据是否正常读出
//		printf("erro");  
//	else
//		printf("success");
//	
	return (rxData & 0xFFFF); // 返回数据
}

//======================================================================
// DRV8301初始化
//======================================================================
void DRV8301_Init(void) 
{
	DRV8301_ENGATE_ON();     
	HAL_Delay(10); // 等待电源稳定
	
	
	/*配置DRV8301控制寄存器1:
	1.栅极驱动电流峰值:0.7A
	2.正常工作模式
	3.6路互补PWM输入模式
	4.过流时断电
	5.过载电流(去看看怎么计算)
	6.二进制表示:0 0010 11111 00 0 0 01 ->  0001 0111 1100 0001  即17C1 */
	
	// 配置控制寄存器1 (0x02)
    uint16_t ctrl1 =(0x00 << 0) |   // 栅极驱动电流：1.7A D[0-1]
	                (0x00<<2)   |   // 正常工作模式 D[2]
				    (0x00 << 3) |   // 6-PWM模式 D[3]
					(0x00 << 4) |   // OCP限流模式（非闩锁）D[4-5]
					(0x1F << 6);    // OCP阈值=0.197V（根据MOSFET Rds(on)计算）D[6-10] 取值0-31			   
      
	DRV8301_WriteReg(DRV8301_REG_CTRL1,ctrl1); 

	/*配置DRV8301控制寄存器2:
	1.同时汇报过温过流(OC,OT引脚未连接，无影响)
	2.并联放大器增益:80V/V
	3.不启用两个分流放大器的矫正功能
	4.循环往复-不设置限流模式反应(限流模式未开启)
	5.二进制表示:0 0011 0000 0 00 01 00	-> 0001 1000 0000 0100  即1804 */
	
    // 配置控制寄存器2 (0x03)
    uint16_t ctrl2 = (0x00 << 0) | // 开启过温过流报警 D[0-1]
                     (0x03 << 2) | // 电流增益80V/V D[2-3]
					 (0x00 << 4) | // 不启用DC_CAL（不启用直流校准）D[4-5]
					 (0x00 << 6);  // 限流模式未开启  过流保护的限流模式(0x01) D[6]	
					 //D[7-10]不用设置

	DRV8301_WriteReg(DRV8301_REG_CTRL2,ctrl2);
  
    //检查有无错误
	status0 = DRV8301_ReadReg(0x00);
//	printf("111\r\n");
//	printf("%x",status0);
//	
//	status0 = DRV8301_ReadReg(0x01);
//	printf("222\r\n");
//	printf("%x",status0);
//	
//	status0 = DRV8301_ReadReg(0x02);
//	printf("333\r\n");
//	printf("%x",status0);
//	
//	status0 = DRV8301_ReadReg(0x03);
//	printf("444\r\n");
//	printf("%x",status0);
	
	
	if (status0 !=0)    // 检查FAULT位
	{
        printf("Fail\r\n"); // 初始化失败 
		allflag.Drv8301_flag = 2;
	}
	else
	{
		printf("Success\r\n");	
		allflag.Drv8301_flag = 1;
	}
	HAL_Delay(500);
}




