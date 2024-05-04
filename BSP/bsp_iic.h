#ifndef __BSP_IIC_SW_H
#define __BSP_IIC_SW_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

#include "BSP_GPIO.h"

typedef struct {
	I2C_TypeDef *I2Cx;					//I2C��
	BSP_GPIOSource_TypeDef *I2C_SCL;	//SPI_NSS ����
	BSP_GPIOSource_TypeDef *I2C_SDA;	//SPI_SCK ����

}BSP_I2C_TypeDef;


//IIC���в�������
void BSP_I2C_Init(BSP_I2C_TypeDef* BSP_I2Cx);
uint8_t BSP_I2C_ReadOneByte(BSP_I2C_TypeDef* BSP_I2Cx,uint8_t Start_Address, uint8_t REG_Address);
void BSP_I2C_WriteOneByte(BSP_I2C_TypeDef* BSP_I2Cx,uint8_t Start_Address, uint8_t REG_Address,uint8_t REG_data);


/**************����I2C����*******************/
extern BSP_I2C_TypeDef BSP_I2C1;
extern BSP_I2C_TypeDef BSP_I2C2;
extern BSP_I2C_TypeDef BSP_I2C3;
extern unsigned char I2C_Err;

#endif
