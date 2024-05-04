#ifndef __VISION_H
#define __VISION_H

#include "bsp.h"
#include "util.h"

#define VISION_USARTX						USART3				//���ں�
#define VISION_USARTX_TX_PIN				BSP_GPIOB10			//��������
#define VISION_USARTX_RX_PIN				BSP_GPIOB11			//��������
#define VISION_USART_BAUD_RATE				460800
#define VISION_USART_PRE_PRIORITY			3					//SBUS_USART�ж���ռ���ȼ�
#define VISION_USART_SUB_PRIORITY			0					//SBUS_USART�ж���Ӧ���ȼ�

#define VISION_GAIN								 30						//�Ӿ�����2.5

typedef enum{
	TX2_STOP = 0,							//ֹͣ����		
	TX2_DISTINGUISH_ARMOR ,		//ʶ��װ�װ�
	TX2_DISTINGUISH_BUFF 			//ʶ���С��	
} visionWorkMode_e;

typedef enum{
	SMALL_BULLET = 0,
	BIG_BULLET
} bulletType_e;

typedef enum{
	ENEMY_RED = 0,
	ENEMY_BLUE
} enemyType_e;

typedef enum{
	SPEED0,
	SPEED1,
	SPEED2,
	SPEED3,
}speedLevelType_e;

typedef struct{
	formatTrans32Struct_t x;
	formatTrans32Struct_t y;
	formatTrans32Struct_t z;
}point3FStruct_t;

typedef struct{
	formatTrans16Struct_t armorCoordinate[3];        //װ�װ�������Ϣ����tx2���գ����͵�����
	formatTrans16Struct_t armorCoordinateReal[3];	//װ�װ�ʵ��������Ϣ����tx2���գ����͵�����
	uint8_t distingushState;
	formatTrans16Struct_t CNTR;				//TX2�ش��İ���ţ���tx2���գ����͵�����
	
	uint8_t mainControlCmd;					//���ض�TX2��������
	uint32_t checkSeq;
	
	uint32_t visionErrorCount;				// ���л�ʧĿ�� ����
	uint32_t visionLastErrorCount;
	uint32_t intervalNum;
	uint16_t counter;
	speedLevelType_e speedLevel;
	errorScanStruct_t visionError;
	uint8_t shootSpeed;
} visionStruct_t;

extern visionStruct_t visionData;

void visionReceive(u8 *array);
void visionControlInit(void);
void visionSendData(USART_TypeDef *USARTx);

#endif
