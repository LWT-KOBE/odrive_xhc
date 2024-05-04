#ifndef __OPENMV_H
#define __OPENMV_H

#include "BSP.h"
#define OPENMV_USARTX					UART7			//OPENMV���ں�
#define OPENMV_USARTX_RX_PIN			BSP_GPIOE8	//OPENMV��������
#define OPENMV_USARTX_TX_PIN			BSP_GPIOE7		//SLAVE_SENSOR��������

#define OPENMV_USART_PRE_PRIORITY 3						//OPENMV_USART�ж���ռ���ȼ�
#define OPENMV_USART_SUB_PRIORITY 0
#define OPENMV_USART_BOUND		 921600


#define TFMINI_USARTX					USART2			//OPENMV���ں�
#define TFMINI_USARTX_RX_PIN			BSP_GPIOA3	//OPENMV��������
#define TFMINI_USARTX_TX_PIN			BSP_GPIOA2		//SLAVE_SENSOR��������

#define TFMINI_USART_PRE_PRIORITY 3						//OPENMV_USART�ж���ռ���ȼ�
#define TFMINI_USART_SUB_PRIORITY 0
#define TFMINI_USART_BOUND		 115200




void Driver_Openmv_Init(USART_TypeDef* USARTx, BSP_GPIOSource_TypeDef *USART_RX, BSP_GPIOSource_TypeDef *USART_TX, \
														uint32_t baudRate, uint8_t PreemptionPriority, uint8_t SubPriority);
void Openmv_Recive(uint8_t *data);


void TFmini_Recive(uint8_t *rebuf);
void Driver_TFmini_Init(USART_TypeDef* USARTx, BSP_GPIOSource_TypeDef *USART_RX, BSP_GPIOSource_TypeDef *USART_TX, \
														uint32_t baudRate, uint8_t PreemptionPriority, uint8_t SubPriority);

extern uint16_t distance;
extern int color,shape;
extern u8 adition_falg;			//�ж�λ���Ƿ��Ѱ�λ
void Spin(void);
void Set_Yaw(float angle);
extern s16 cnt;
extern s8 FLAG;
extern float x_trend;   //x������˶�����
extern int  Lm_1,Lm_2,Lm_3;
extern int size;

#endif
