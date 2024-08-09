#ifndef __SUPERVISOR_H
#define __SUPERVISOR_H

#include "bsp.h"
#include "FreeRTOS_board.h"
#include "board.h"

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�				   //
//			���ض�����������ɿ��Ը����⣬		  		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

#define SPUER_PRIORITY   10
#define SPUER_STACK_SIZE 512
#define SUPER_STACK_PERIOD 100

enum {
	IMU_TASK = 0,
	SUPERVISOR_TASK,
	AHRS_TASK,
	CONTROL_TASK,
	UKF_INIT_TASK,
	WIRELESS_TASK,	
	LCD_TASK,
	ODRIVE_TASK,
	XHC_Task,
	CAN2_Task,
	LIST_OF_TASK,
	
};

#define IMU_SENSOR_ERROR	 15
#define SHOOT_ERROR        1
#define UART_NO_RECEIVE		 2
#define ALL_DEVICE_IS_WELL 9

#define LED_IMU_HARDWORK_FALSE 20			//20 ǳ���ƿ��� 

#define LED_VISION_HARDWORK_FALSE	1		//�������
#define LED_ODRIVE_FALSE 15				//��ƿ���
#define LED_IMU_CALI_FALSE	16				//�̵ƿ���
#define LED_RADIO_LOSS	18						//���ƿ���
#define LED_STATE_NORMAL	9						//�̵������ٶ���

/*
3 �Ƶ�����
4 ����������
5 �ϵ�����
6 ǳ��������
7 ǳǳ��������
8 ���������
10 �Ƶ�������
11 ����������
12 �۵�������
13 ǳ����������
14 ǳǳ����������
17 �Ƶƿ��� 
19 �ϵƿ��� 
20 ǳ���ƿ��� 
21 ǳǳ���ƿ��� 

*/





enum supervisorStates {
	STATE_INITIALIZING	= 0x0000,														//��ʼ��״̬
	STATE_MAGCALI	= 0x0001,																	//������У׼״̬
	STATE_DISARMED	= 0x0002,																//����
	STATE_ARMED		= 0x0004,																	//����
	STATE_IMUCALI	= 0x0008,																	//IMUУ׼
	STATE_RADIO_LOSS	= 0x0010,															//ң�ض�ʧ
	STATE_SENSOR_ERROR = 0x0020,														//����������
	STATE_LOW_BATTERY	= 0x0040,															//���
	STATE_VISION_ERROR = 0x0100,														//�Ӿ��˴���
	STATE_ODRIVE_ERROR = 0x0200,												//��ODRIVEͨ�Ŵ���
	STATE_IMU_LOSS = 0x0400,																//�����Ƕ�ʧ
};

enum{
	MODULAR_BULLET_MONITR = 0x01,
	MODULAR_IMU_LOSS = 0x02,
	MODULAR_IMU_CALI = 0x04,
};

typedef struct {
	TaskHandle_t xHandleTask;
	BaseType_t taskEvent[LIST_OF_TASK];
	uint8_t taskState[LIST_OF_TASK];
	uint16_t state;
	bool imuTempState;
	bool busyState;
	uint8_t flashSave;
	volatile uint8_t imuTempFinish;
	uint16_t rgbState;
    uint16_t beepState;
	uint32_t loops;
	uint8_t modularState;
    uint32_t intervalNum;	    	
    uint32_t Lobot_ErrorCount;
    uint32_t Lobot_LastErrorCount;    
    uint16_t gimbalCaliReset ;       
} supervisorStruct_t;

supervisorStruct_t* getsupervisorData(void);

void supervisorInit(void);
void shootCheckDevice(void);
void supervisrCheckModule(void);
void supervisorGimbalCali(void);
void Odrive_CheckDevice(void);

#endif
