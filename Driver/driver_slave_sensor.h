#ifndef __DRIVER_SLAVE_SENSOR_H
#define __DRIVER_SLAVE_SENSOR_H

#include "bsp.h"
#include "d_imu.h"

/**********************************SLAVE_SENSORʼ������****************************************/
#define SLAVE_SENSOR_USARTX										USART3			//SLAVE_SENSOR���ں�
#define SLAVE_SENSOR_USARTX_RX_PIN						BSP_GPIOD9	//SLAVE_SENSOR��������
#define SLAVE_SENSOR_USARTX_TX_PIN						BSP_GPIOD8	//SLAVE_SENSOR��������
#define SLAVE_SENSOR_USART_PRE 								3						//SLAVE_SENSOR_USART�ж���ռ���ȼ�
#define SLAVE_SENSOR_USART_SUB 								0						//SLAVE_SENSOR_USART�ж���Ӧ���ȼ�

void driver_slaveSensorInit(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u32 baudRate,u8 PreemptionPriority,u8 SubPriority);


#endif
