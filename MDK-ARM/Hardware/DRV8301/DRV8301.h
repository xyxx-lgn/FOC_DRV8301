#ifndef __DRV8301_H
#define __DRV8301_H		


//头文件包含
#include "spi.h"
#include "ALL_H.h"

#define DRV8301_CS_ON()        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET)
#define DRV8301_CS_OFF()       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET)


#define DRV8301_ENGATE_ON()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define DRV8301_ENGATE_OFF()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

#define DRV8301_NFAULT_ON()    (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)==GPIO_PIN_RESET) //出现故障
#define DRV8301_NFAULT_OFF()   (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10)==GPIO_PIN_SET)


/* DRV8301寄存器地址 */
#define DRV8301_REG_STATUS1  0x00    //故障指示
#define DRV8301_REG_STATUS2  0x01    //设备ID
#define DRV8301_REG_CTRL1    0x02    //控制寄存器1
#define DRV8301_REG_CTRL2    0x03    //控制寄存器2


void DRV8301_Init(void);
void DRV8301_WriteReg(uint8_t reg, uint16_t data);
uint16_t DRV8301_ReadReg(uint8_t reg);



#endif
