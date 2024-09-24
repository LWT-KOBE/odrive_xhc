#ifndef __STATE_H
#define __STATE_H	 

#include "application.h"
#define ST0 0    //��ʼ��
#define ST1 1	  //�Լ�
#define ST2 2		//�Լ��վ
#define ST3 3		//Ͷ�ݽ�վ
#define ST4 4		//װ�ؽ���
#define ST5 5   //�����վ
#define ST6 6		//��վ
#define ST7 7   //��վ����

#define FLAG_HEAD 0xfe
#define FLAG_TAIL 0xfe

#define FRAME_HEADER      0X7B //Frame_header //֡ͷ
#define FRAME_TAIL        0X7D //Frame_tail   //֡β
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

#define UART2_RX_LEN     1024+50
#define UART2_TX_LEN     256
#define UART1_RX_LEN     1024+50
#define UART1_TX_LEN     256


 


extern u8 Uart1_Rx[UART1_RX_LEN];
extern u8 Uart1_Tx[UART1_TX_LEN];
extern u16 Uart1_Rx_length;
extern u16 Uart1_Tx_length;

extern u8 Uart2_Rx[UART2_RX_LEN];
extern u8 Uart2_Tx[UART2_TX_LEN];
extern u16 Uart2_Rx_length;
extern u16 Uart2_Tx_length;


extern u8 InStationLSFlag;
extern u8 GoTrainflag ;
extern u8 TravelCommand;
extern u8 DeliverResulBasketNumber;
extern u8 BasketNumber;
//extern u8 StopAPPToHeadAddr;
extern u8 Reply_Result_Flag;
extern  float MBSpeed;
extern float MBSpeedOld;
extern u16 MBSpeeddelay;
extern u8 TrainState;
extern u8 OldTrainState,NewTrainState;
extern u8 StepState;
extern u8 TrainDircition;
extern u8 zeroCount;
extern u8 zeroFlag;
extern u32 TrainMileage;
extern u32 OldTrainMileage;
extern u8 vofa_sendFlag; 

extern u8 TimerSendDataCount ;
extern u8 SendCount;
extern u8 DelaySendFlag;
extern u16 CageNumber;
extern u8 ApplicationAddrFlag;
extern u8 MoterBrakeFlag,MoterBrakeFlagH;

extern u8 IntermediateVariable1;
extern u8 IntermediateVariable2;

extern u16 StopLG;
extern u8 ShieldBasket;
extern u8 RecycleBasket;
extern u8 RecycleBasketFlag;
extern u8 CanSendOutStationFlag;
extern u8 PocketStep;
extern u8 SendUDPDataDelay200ms;
extern u8 SendUDPDataDelay100ms;
extern u8 HeartDelayCount;
extern u8 BreakStopFlag;
extern u8 WIFISendDataDelayCount ;
extern u8 WIFISendDataBuf[150];
extern u8 WIFIdatalen;
extern u8 AlltrainST1Flag;
extern u8 trainST1Flag;
extern u8 CarGoGoFlag;
extern u8 TrainWarning;
extern u8 SensorWarning;

extern u16 SensorWarningDelay1,SensorWarningDelay2,SensorWarningDelay3,SensorWarningDelay4,\
	 SensorWarningDelay5,SensorWarningDelay6,SensorWarningDelay7,SensorWarningDelay8;

u8 WIFIdataSend(u8 *Databuff,u8 DataLen);
void TrainContral (void);
void ApplicationCommandUart1(u8 Type,u8 DataLen);
void DeliverResultToApp (u8 Type,u8 DataLen,u8 Number);
void HeartToApp (u8 Type,u8 DataLen);
void ST1ACKToApp (u8 Type,u8 DataLen);
void  ApplyForExitToAPP(u8 Type,u8 DataLen);	
void  RecycleACKToAPP(u8 Type,u8 DataLen);
void ConfigrationAck(u8 Type,u8 DataLen);
void  InStationToAPP(u8 Type,u8 DataLen);
void ApplyForExit(u8 Type,u8 DataLen);
void MotorBrake(void);
void TrainFollowTrain(void);
void ReadE2promData(void);
void ADDRACKToApp (u8 Type,u8 DataLen);	
void ReadDataACK(u8 Type,u8 DataLen);
void USART_Data_Send_Task (void);
void TrainBusinessLogic (void);
void IO_Init(void);
void CanSendDataTask (void);
#endif




