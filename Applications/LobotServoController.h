/*******************************************************************************
* �ļ���: LobotServoController.h
* ����: �����ֻ������Ƽ�
* ���ڣ�20160806
* LSCϵ�ж�����ư���ο���ʾ��
*******************************************************************************/

#ifndef LOBOTSERVOCONTROLLER_H_
#define LOBOTSERVOCONTROLLER_H_

#include "stm32f4xx.h"
#include "FreeRTOS_board.h"
#include "stdbool.h"
#include "bsp.h"
#include "util.h"

#define FRAME_HEADER 0x55             //֡ͷ
#define CMD_SERVO_MOVE 0x03           //����ƶ�ָ��
#define CMD_ACTION_GROUP_RUN 0x06     //���ж�����ָ��
#define CMD_ACTION_GROUP_STOP 0x07    //ֹͣ������ָ��
#define CMD_ACTION_GROUP_SPEED 0x0B   //���ö����������ٶ�
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //��ȡ��ص�ѹָ��

//�꺯�� ���A�ĵͰ�λ
#define GET_LOW_BYTE(A) ((uint8_t)(A))

//�꺯�� ���A�ĸ߰�λ
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

/**********************************Lobot��ʼ������****************************************/
//Lobot���ں�
#define Lobot_USARTX							USART3
//Lobot��������
#define Lobot_USARTX_RX_PIN						BSP_GPIOD9	
//DTU��������
#define Lobot_USARTX_TX_PIN						BSP_GPIOD8	
//Lobot_USART�ж���ռ���ȼ�
#define Lobot_USART_PreemptionPriority 			3					
//Lobot_USART�ж���Ӧ���ȼ�
#define Lobot_USART_SubPriority 				0	







extern bool isUartRxCompleted;
extern uint8_t LobotRxBuf[16];
extern uint16_t batteryVolt;
extern void receiveHandle(void);

typedef struct _lobot_servo_ {  //���ID,���Ŀ��λ��
	uint8_t ID;
	uint16_t Position;
} LobotServo;


void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
void moveServos(uint8_t Num, uint16_t Time, ...);
void runActionGroup(uint8_t numOfAction, uint16_t Times);
void stopActionGroup(void);
void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
void setAllActionGroupSpeed(uint16_t Speed);
void getBatteryVoltage(void);
void Driver_LobotReadDMA(uint8_t *arrayLobotReceive);



#endif
