#ifndef __APP_H
#define __APP_H

#include "application.h"

/**************************�ṹ����***********************************/

extern u32 CAN_ID ;
extern u8 RxRAM0[8];
extern u8 Rxflag1 ;
extern u32 PocketCount,PocketCountold;//U�͵�Ƭ����
extern u32 Pocket_A_Count,OldPocket_A_Count,NewPocket_A_Count;

extern u16 gCheckHeartLiveCount;
extern u16 gCheckTrailLiveCount;
extern u16 gCheckFroCarLiveCount;
extern u16 ChargePosition;
extern u8 ChargeFlag;

#define DriveMode  1//����������� 0--�������� 1--ODRIVE
#define NFCMode    0  //NFCģʽ   0--��Ƭģʽ  1--NFCģʽ
#define TrainVersion  1  //�汾��
void CheckCarCanCmd(void);
void SaveData(CanRxMsg temp_CAN_Msg); 

void PowerValueLedShow(void);
void IAP_APP_Init(void);
void ChargeStop (void);
void Motor_Contral(u8 Dir,u16 Speed,u8 MotorEnable);
extern u16 testflag;
extern u8 MotorDir;
extern u8 LD_ONFlag;
extern unsigned  int gCarBatPower;
extern u32 Pocket_A_Count1,Pocket_A_Count2;
extern u32 PocketCount1,PocketCount2;
void GetCarSpeed(void);
void Motor_ContralA(u16 Speed);
void Motor_ContralB(u16 Speed); 
void time2_init(u16 arr,u16 psc);
void TIM14_PWM_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr, u16 psc);
void TM1620_Config(void);
void TM1620_init(void);
void Display (void);
void LED_Tube_Choose_DisPlay1(u8 num1,u8 num2,u8 num3);
void KeyScan(void);
u8 LoadingStation(void);
//��ʾ�����ķ�Χ
#define CARPOWER_VUALE_MAX_FLOAT  (320)
#define CARPOWER_VUALE_MIN_FLOAT  (220)

#define IAP_Bootloat_SIZE 		0x2000	//8K

#define PAIR_POLE    10   //������
#define HALL_STATE_NUM  3  //һ����Ƕ��ڻ���״̬����
#define SPEED_VAULE    (u8)(HALL_STATE_NUM*PAIR_POLE)

#define MOTOR_DIRA 			PCout(4)	 
#define MOTOR_DIRB 			PCout(5)
#define MOTOR_ENA_B 		PAout(5)


#define Q_GDSW PBin(3) 
#define H_GDSW PDin(2) 

#define Q_LD PCin(12) 
#define H_LD PAin(15) 
#define MF_SW PCin(3)


/*  		 	Backups 				  */
#define Q_GDSW_B PCin(14) 
#define H_GDSW_B PCin(13) 

#define Q_LD_B PCin(0) 
#define H_LD_B PCin(15) 

#define MF_SW_B PCin(2)
/*  		 	Backups 				  */

#define HeadLed PAout(11)

#define KeyNumHead  (u8)((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)<<3)\
											 |(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)<<2)\
											 |(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)<<1)\
											 |(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9))) 	
											 
											 
#define SW1 PAin(0)											 
#define SW2 PBin(12)	

#define SCLH GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define SDAH GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define SCLL GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define SDAL GPIO_ResetBits(GPIOB, GPIO_Pin_0)

extern u16 LedNumDisplay;
extern u8 KeyValue;
extern u8 KeyUpFlag;
extern u8 KeyMode;

extern u32 PulseCntA;
extern u32 PulseCntB;
extern float  gSpeedR;
extern float  gSpeedRB;
extern float  gSpeedRA;
extern float  gSpeedRD;
extern float  gSpeedRC;

extern float gPowerValueFloat;
extern u8 ReadSpeedFlag;
extern u16 ReadSpeedTimerCount;
extern u16 PositionTail;
extern u16 powervol;
extern int LastDeliverResultNum[21] ;
extern unsigned short int LastPoketNum[21];
extern u8 BasketError[24];
extern u32 APPBasketNum;
extern u32 APPSendADDRFlag;
extern int AppSendAddr[21];
extern u32 NFCNUM,NFCNUMold,NFCNUMnew;
extern u32 TailstockNFC;
extern u8 HeadSendBasketFlag;
extern u32 BasketPocketNum,BasketPocketNumold,BasketPocketNumnew,LastDeliverResult;
extern u16 FrontCarPositionNFC;
extern u16 FrontCarPositionHead;
extern u8 MotorGoFlag;
extern u16 FrontCarPositionPokec;
extern u16 FrontCarPositionPokec1;
extern u16 FrontCarPositionPokec2;
extern u8 FrontCarPositionFlag;
//extern u8 TrailPositionFlag;

extern u16 CarPositionPocket;
extern u16 CarTailNFCNum;
extern u16 CarDistance;
extern u8 SendDataFlag;
extern u8 ReciveRightFlag;
extern u8 CarSignalNum;
extern u8 CarGo;
extern u8 CarDirection;
extern u8 ScranTrain;
extern u16 TrainBasketPosition;
extern u32 BasketApplicationFlag;
extern u32 DeliverResultACKFlag;
extern u8 ST1ACKFlag;
extern u8 StionStop;
extern u8 MoveOneFlag;
extern u8 TrainApplyForExitFlag;
extern u32 DeliverResultFlag;
extern u8 BreakCargo;
extern u8 CarBack;
extern u8 CarScranFlag;
extern u16 IDMaxNum;
extern u8 TrainMaxNum;
extern u8 TrainHeadNum;
extern u8 CarDriveFlag;
extern u8 ApplicationtTavel;
extern u8 ForceTravel1;
extern u8 ForceTravel2;
extern u8 ForceTravel3;
extern u8 ConfigrationFlag;
extern u8 InStationFlag;
extern u32 BasketLifeFlag ;
extern u32 BasketReciveAddrFlag;
extern u8 ReadDataFlag;
extern u16 BasketNumACKADDRToAPP;
extern u8 APPSendADDRACKFlag;
extern u8 CanSendCoount;
extern u8 MFContralZS;
extern u8 MFContralDS;
extern u32 AppAddrACK;
extern u8 InStationCount;
extern u8 InStationCount1;
//extern u8 DriveMode;
////////////////////////////���ò���///////////////////////////

extern u16 CaseNum;
extern u8 Pocket4;
extern u8 TrainBasketMaxNum;
extern u8 TrainMode;
extern u8 HigSpeed ;
extern u8 MidSpeed ;
extern u8 LowSpeed ;
extern u8 OutStationQuicken ;
extern u8 FollowHigSpeed ;
extern u8 FollowMidSpeed ;
extern u8 FollowLowSpeed ;

extern u8 InStationHigtoMidSpeed ;
extern u8 InStationMidtoLowSpeed ;
extern u16 RecycleCaseNum;

////////////////////////////���ò���///////////////////////////

#endif

