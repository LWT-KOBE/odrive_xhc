#ifndef __APP_H
#define __APP_H

#include "application.h"

/**************************结构类型***********************************/

extern u32 CAN_ID ;
extern u8 RxRAM0[8];
extern u8 Rxflag1 ;
extern u32 PocketCount,PocketCountold;//U型挡片计数
extern u32 Pocket_A_Count,OldPocket_A_Count,NewPocket_A_Count;

extern u16 gCheckHeartLiveCount;
extern u16 gCheckTrailLiveCount;
extern u16 gCheckFroCarLiveCount;

#define DriveMode  0//电机驱动类型 0--老驱动板 1--ODRIVE


void CheckCarCanCmd(void);
void SaveData(CanRxMsg temp_CAN_Msg); 

void PowerValueLedShow(void);
void IAP_APP_Init(void);

void Motor_Contral(u8 Dir,u16 Speed,u8 MotorEnable);
extern u16 testflag;
extern float gSpeedR;
extern u8 MotorDir;
extern unsigned  int gCarBatPower;
void GetCarSpeed(void);
void Motor_ContralA(u16 Speed);
void Motor_ContralB(u16 Speed); 
void time2_init(u16 arr,u16 psc);

//显示电量的范围
#define CARPOWER_VUALE_MAX_FLOAT  (320)
#define CARPOWER_VUALE_MIN_FLOAT  (220)

#define IAP_Bootloat_SIZE 		0x2000	//8K


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
////////////////////////////设置参数///////////////////////////

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

////////////////////////////设置参数///////////////////////////

#endif

