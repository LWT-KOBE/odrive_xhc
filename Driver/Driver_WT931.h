#ifndef __DRIVE_WT931_H
#define __DRIVE_WT931_H
#include "Util.h"
#include "bsp.h"
#include "BSP_GPIO.h"
#include "stdbool.h"

#define MPU_WT931_USARTX										USART6		  //���ں�
#define MPU_WT931_USARTX_RX_PIN							BSP_GPIOC7	//��������
#define MPU_WT931_USARTX_TX_PIN							BSP_GPIOC6	//��������
#define MPU_WT931_USART_PreemptionPriority 	3						//�ж���ռ���ȼ�
#define MPU_WT931_USART_SubPriority 				0						//�ж���Ӧ���ȼ�

typedef enum{
	NOT_READY = 0,
	ACC_READY,		//���ٶ�ͨ��У���
	GYRO_READY,		//���ٶ�ͨ��У���
	ANGLE_READAY,	//�Ƕ�ͨ����У���
}imuSumFlag_e;

typedef struct{
	float x; 
	float y; 
	float z; 
	float T;
}mpuDirection_t;

typedef struct{
    
	bool initFlag;        
   	uint32_t loops;
    formatTrans32Struct_t yaw, pitch, roll;
	formatTrans16Struct_t CNTR;        
	mpuDirection_t stcAcc;
	mpuDirection_t stcGyro;
	mpuDirection_t stcAngle;
}wt931Data_t;


extern wt931Data_t wt931Data;




void Driver_WT931_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);
void dataToCopy(uint8_t array[]);
void Driver_WT931ReadDMA(uint8_t *arrayWT931Receive);
void caliInitWT931(uint8_t * sendData);


#endif



