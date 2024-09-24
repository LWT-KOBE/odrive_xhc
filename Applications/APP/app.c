#include "app.h"
#include "stdlib.h"
#include <string.h>
#include "application.h"
#include "MyADC.h"  
#include "pid.h"  
CanRxMsg RxMessage;
CanTxMsg TxMessage;

u8 gShowNumberData[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6d, 0x7d, 0x07, 0x7F, 0x6F};
// ��ʾ��   E   P  A  b  C
u8 gShowAlphabetData[7] = {0x00, 0x79, 0x73, 0x77, 0x7c, 0x39,0x3e};

u16 gTim4UpdateCounter = 0;
u32 gNewGetTim4Counter = 0;
u32 gOldGetTim4Counter = 0;

u32 gErrorVaule = 0;
u32 GetSpeedVaule[SPEED_VAULE] = {0};  
u16 gGetSpeedVauleCounter = 0;
u32 gSum = 0;
u32 PulseCntA = 0;
u32 PulseCntB = 0;
//u16 gCheckHeartLiveCount = 0;
//u16 gCheckTrailLiveCount = 0;
//u16 gCheckFroCarLiveCount = 0;

u16 KeyUpCount = 0;
u8 KeyUpFlag = 0;
u8 KeyValue,KeyValueold,KeyValuenew = 0;
u16 LedNumDisplay = 0;
u8 KeyMode = 1;

u16 KeyUpCount2 = 0;
u8 KeyUpFlag2 = 0;
u8 KeyValue2,KeyValue2old,KeyValue2new = 0;

u32 CAN_ID = 0;
u8 RxRAM0[8];
u8 Rxflag1 = 0;

u32 PocketCount,PocketCountold = 0;//U�͵�Ƭ����

u32 PocketCount1,PocketCount2 = 0;

u32 Pocket_A_Count,OldPocket_A_Count,NewPocket_A_Count = 0;//������Ƭ

u32 Pocket_A_Count1,Pocket_A_Count2 = 0;

u16 LongLDCount = 0;
u16 Timer40msCount = 0;
u16 Timer10msCount = 0;
u16 Timer100msCount = 0; 
u16 gCheckHeartLiveCount = 0;//�������߼��
u16 gCheckTrailLiveCount = 0;
u16 gCheckFroCarLiveCount = 0; 
u16 SwitchCount1 = 0;
u16 SwitchCount1_B = 0;
u32 SwitchCount2 = 0; 
u8 SwitchCount = 0; 
u16 LedCnt = 0; 
u8 LD_ONFlag = 0;
u8 LD_B_ONFlag = 0;
u16 CarGoDelay = 0; 

float gSpeedR = 0;
float gSpeedRA = 0;
float gSpeedRB = 0;
float gSpeedRC = 0;
float gSpeedRD = 0;
u8 ReadSpeedFlag = 0;//��������쳣1
u16 ReadSpeedTimerCount = 0;
u8 MotorDir = 0; //
u8 MotorGoFlag = 0;
u32 APPBasketNum = 0x00000000;//����B3  �����B2  Ͷ�ݸ�ں�B0\1

u32 APPSendADDRFlag = 0;//APP�·���ַ��־λ  21��λ����21�������·��ĵ�ַ
u8 APPSendADDRACKFlag = 0;//APP�·���ַ��־λ-�Զ�ģʽ
int AppSendAddr[21];//�·���ַ  ����B3  �����B2  Ͷ�ݸ�ں�B0\1
u32 AppAddrACK = 0;//Ӧ��ظ�

int LastDeliverResultNum[21];//����B1  �����B2  Ͷ�ݸ�ں�B3\4 
unsigned short int LastPoketNum[21];//��һȦ��Ƭ����

u32 BasketApplicationFlag = 0;//21��λ����21�������Ӧ�����ַ״̬    1--�����ַ��־
u32 DeliverResultFlag = 0;//21��λ����21�������Ӧ�ϱ����״̬  1--�ϱ�
u32 DeliverResultACKFlag = 0;//21��λ����21�������Ӧ�ϱ����Ӧ��״̬  1--Ӧ��

u8 BasketError[24];//������ϻ���

u32 NFCNUM,NFCNUMold,NFCNUMnew = 0;//��ͷλ��
u32 TailstockNFC = 0;

u16 CarTailNFCNum = 0;//��βλ��
u8 MoveOneFlag = 0;

u8 HeadSendBasketFlag = 0; // 0x01--�·���ַ  0x02--ֹͣ�·�  0x04--�·��ظ�Ͷ�ݽ��Ӧ���ź�  0x08--ֹͣͶ�ݽ��Ӧ���·�
u32 BasketPocketNum,BasketPocketNumold,BasketPocketNumnew,LastDeliverResult = 0;

u32 BasketReciveAddrFlag = 0;//�����յ���ַ��־λ 21bit����21������--�Զ�ģʽ

u16 BasketNumACKADDRToAPP = 0;

u16 PositionTail = 0;//��Ƭ��βλ��
u16 FrontCarPositionNFC = 0;//ǰ��λ�ã�ָ����ǰ�����һ�ڳ����λ��

u16 testflag = 0;

u16 FrontCarPositionPokec = 0;//ǰ����βλ��
u16 FrontCarPositionPokec1 = 0;//ǰ����βλ��
u16 FrontCarPositionPokec2 = 0;//ǰ����βλ��

u8 FrontCarPositionFlag = 0;//ǰ��λ�û�ȡ�ɹ���־λ
//u8 TrailPositionFlag = 0;//��βλ�����߱�־

u16 FrontCarPositionHead = 0;//ǰ����ͷλ��λ��
u16 CarPositionPocket = 0;//��Ƭ��+�ӽ���Ƭ
u16 CarDistance = 0;  //��ǰ������������
u8 SendDataFlag = 0;//�յ�ǰ���Ĺ㲥���������ݱ�־
u8 ReciveRightFlag = 0;//�������ձ�־λ

u8 CarSignalNum = 0;//��ʱ�ȴ�����ʱ�����
u16 TrainBasketPosition = 0;//��βλ�ø�ں�

float gPowerValueFloat = 0;
u16 powervol = 0;
unsigned  int gCarBatPower = 0; //��Դ��ѹ
u16 carpower = 0;

u8 ST1ACKFlag = 0;//�Լ�ָ���־λ

u8 TrainApplyForExitFlag = 0;//��վ�����־λ
u8 BreakCargo = 0;
u8 CarGo = 0;//0--ǰ��
u8 CarBack = 1;//����
u8 ScranTrain = 0;//�����־--1
u8 StionStop  = 0;   		//��վͣ����Ƭ

u8 CarDirection = 0;//0-stop 1ǰ��  2����
u16 IDMaxNum = 0;//����ں�
u8 CarScranFlag = 0;
u8 CarDriveFlag = 0;

u8 ApplicationtTavel=9;	//���뷢����Ƭ
u8 ForceTravel1 =10;
u8 ForceTravel2 =11;
u8 ForceTravel3 =29;

u8 ConfigrationFlag = 0;//���ñ�־λ 1--����
u8 InStationFlag = 0;//��վ��־
u8 InStationCount = 0;
u8 InStationCount1 = 0;
u32 BasketLifeFlag = 0;//�������߱�־

u8 ReadDataFlag = 0;
u8 CanSendCoount = 0;

u16 ChargePosition = 0x45;//ָ��λ�ó��
u8 ChargeFlag = 0;//ָ��λ�ó���־λ

//u8 DriveMode = 1;//����������� 0--�������� 1--ODRIVE

////////////////////////////���ò���///////////////////////////

u16 CaseNum = 77;//�����   ADDR 0-1
u8 Pocket4=11;						//װ�ص�Ƭ ADDR 2
u8 TrainBasketMaxNum = 7;//��������������β��� ADDR 3
u8 TrainMode = 1;//ɨ��ģʽ  //0--�ֶ�ɨ��    1--�Զ�ɨ��  ADDR 4
u8 HigSpeed = 200;//ADDR 5
u8 MidSpeed = 150;//ADDR 6
u8 LowSpeed = 80;//ADDR 7

u8 OutStationQuicken = 4;//��վ���ٸ�� ADDR 8

u8 FollowHigSpeed = 20;//���������� ADDR 9
u8 FollowMidSpeed = 10;//���������� ADDR 10
u8 FollowLowSpeed = 5;//���������� ADDR 11

u8 InStationHigtoMidSpeed = 10;//����װ����ǰ���ټ����� ADDR 12
u8 InStationMidtoLowSpeed = 4;//����װ����ǰ���ټ����� ADDR 13
u8 TrainMaxNum = 3;//������    ADDR 14
u16 RecycleCaseNum = 77;//���ո��  ADDR 15-16

u8 MFContralZS = 1;//���������ں�����  ADDR 17
u8 MFContralDS = 2;//���������ںŵ���  ADDR 18
u8 TrainHeadNum = 0;   //����           ADDR 19
////////////////////////////���ò���///////////////////////////




void IAP_APP_Init(void)
{
	SCB->VTOR = FLASH_BASE | IAP_Bootloat_SIZE; //IAP_Bootloat_SIZE:0x2000=8K �ֽ�
//	GetLockCode(&g_Lock_Code);						//��ȡоƬΨһID
}





u32 count = 0;
u8 Icount[21];
u8 Icount1[21];
u8 Icount2[21];
u8 i = 0;
void SaveData(CanRxMsg temp_CAN_Msg)
{ 
  u8 i; 
	
	CAN_ID = temp_CAN_Msg.StdId;
	
	if(temp_CAN_Msg.IDE == CAN_Id_Standard)
	{
		for(i = 0; i < 8; i++)
		{
			RxRAM0[i] = temp_CAN_Msg.Data[i];
		}
	}	
		Rxflag1 = SUCCESS;

}

void CheckCarCanCmd(void)
{	
	if(Rxflag1 == SUCCESS)
	{		
		for(i=0;i<21;i++)
		{
			if(CAN_ID == i+1)	
			{			
				BasketError[i] = RxRAM0[3];//������������
				BasketLifeFlag |= 1 << i;
				gCheckHeartLiveCount = 0;//�������߼��ʱ������
			}
			if(TrainMode == 0)//�ֶ�ɨ��
			{
				if((RxRAM0[0] & 0x0f)== 0x01)//�����ַ
				{
					if(CAN_ID == i+1)	
					{
						Icount1[i]++;
						if(Icount1[i] == 1)
						{
							MoveOneFlag = 0x01;
							BasketApplicationFlag |= 1 << i;//�������������״̬
							IntermediateVariable2 = 0;
						}
						if(Icount1[i] >= 0xee)
							Icount1[i] = 0xee;
						
						Icount[i] = 0; 
					}
				}	
				else if((RxRAM0[0]& 0x0f) == 0x02)//�����յ���ַֹͣ�·�
				{
					if(CAN_ID == i+1)	
					{					
						Icount[i]++;
						APPSendADDRFlag &= ~(1<<i);//0808
						if(Icount[i] == 1)
						{
							APPSendADDRFlag &= ~(1<<i);//
							HeadSendBasketFlag &= ~0x03;
							HeadSendBasketFlag |= 0x02;
							BasketApplicationFlag &= ~(0x00000001 << i);//�������״̬
						}
						if(Icount[i] >= 0xee)
							Icount[i] = 0xee;
						Icount1[i] = 0;
					}
				}
			}
			else//�Զ�ɨ��
			{
				if(CAN_ID == i+1)		
				{
					if((RxRAM0[0]& 0x0f) == 0x02)//�����յ���ַֹͣ�·�
					{					
						Icount[i]++;
						APPSendADDRFlag &= ~(1<<i);
						if(Icount[i] == 1)
						{
//							APPSendADDRFlag &= ~(1<<i);
							HeadSendBasketFlag &= ~0x03;
							HeadSendBasketFlag |= 0x02;	
							BasketReciveAddrFlag |= (1<<i);//�����յ���ַ
						}
						if(Icount[i] >= 0xee)
							Icount[i] = 0xee;
					}
					else if((RxRAM0[0]& 0x0f) == 0x03)
					{
						Icount[i] = 0;
						BasketReciveAddrFlag &= ~(1<<i);
					}					
				}
			}
			
			if((RxRAM0[0] & 0xf0)== 0x10)//�յ������ϱ����
			{
				if(CAN_ID == i+1)	
				{	
					Icount2[i]++;
					if(Icount2[i] == 1)
					{					
						LastDeliverResultNum[i] = RxRAM0[1] <<8 | RxRAM0[2];  //���������Ͷ�ݽ��
						LastPoketNum[i] = RxRAM0[4] <<8 | RxRAM0[5];
						DeliverResultFlag |= 1 << i;//����������ϱ�״̬
					}
					if(Icount2[i] >= 0xee)
						Icount2[i] = 0xee;					
					
				}	
			}
			else if((RxRAM0[0]& 0xf0) == 0x20)//�����յ�Ͷ�ݽ��ȷ��Ӧ���ź�
			{
				if(CAN_ID == i+1)	
				{					
					HeadSendBasketFlag &= ~0x0c;
					HeadSendBasketFlag |= 0x08;			
					DeliverResultFlag &= ~(1 << i);//����ϱ�״̬
					Icount2[i] = 0;
				}
			}
		}
//	#if !DriveMode
//		if(CAN_ID == TrainBasketMaxNum-1)//��ȡ��βλ��
//		{
//			PositionTail = RxRAM0[4]<<8 | RxRAM0[5];
//			PositionTail = PositionTail/2+PositionTail%2;
//			TrailPositionFlag = 1;
//			gCheckTrailLiveCount = 0;
//		}	//0814
//	#else
		
//		if(CAN_ID == TrainBasketMaxNum-1)//��ȡ��βλ��
//		{
//			PositionTail = RxRAM0[4]<<8 | RxRAM0[5];
//			PositionTail = PositionTail/2+PositionTail%2;
//			TrailPositionFlag = 1;
//		gCheckTrailLiveCount = 0;
//		}		
		
//	#endif		

		
	}
}




void PowerValueLedShow(void)//
{

//	float dat=0;
	gPowerValueFloat = ((1.0 * gCarBatPower) * 3.3 * (18.0 + 1.0) / 4096.0 / 1.0);
	powervol = gPowerValueFloat *10;	
//	dat = (powervol - CARPOWER_VUALE_MIN_FLOAT)*100.0/(CARPOWER_VUALE_MAX_FLOAT-CARPOWER_VUALE_MIN_FLOAT);

//	if(dat>99.0)
//	{
//		dat = 99; 
//	}
//	
//	if(dat < 1)
//	{
//		dat = 1;
//	}
//	carpower=(u8)(dat);
	
}


void Read_vol_contral(void)
{
	uint16_t  cnt = 0;
	u8 TransmitMailbox=0;	
	{
		TxMessage.ExtId = 0x0F585944;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.DLC = 8;
			
		TxMessage.Data[0] = 0x02;
		TxMessage.Data[1] = 2;
		TxMessage.Data[2] = 0xa7;//vol
		TxMessage.Data[3] = 0xa8;//cur
		TxMessage.Data[4] = 0;
		TxMessage.Data[5] = 0;
		TxMessage.Data[6] = 0;
		TxMessage.Data[7] = 0;

		//send to meter
		TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
		cnt = 0;
		while((CAN_TransmitStatus(CAN1,TransmitMailbox) != CANTXOK) && (cnt != 0xffff))
		{
			cnt++;
		}
	}	
	
}

void time2_init(u16 arr,u16 psc)//
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrct;
	NVIC_InitTypeDef NVIC_InitStrct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitStrct.TIM_Period=arr;
	TIM_TimeBaseInitStrct.TIM_Prescaler=psc;
	TIM_TimeBaseInitStrct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStrct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStrct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrct);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM2,ENABLE);//ʧ��
	
	NVIC_InitStrct.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStrct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStrct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStrct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStrct);	
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM2,TIM_IT_Update)==SET)//
	{	
		if(gSpeedR >= 10 && (TrainState == ST6 || TrainState == ST1))
			LongLDCount++;
		
		Timer40msCount++;
		Timer10msCount++;
		Timer100msCount++;
		CarGoDelay++;
	
		gCheckHeartLiveCount++;
		if(gCheckHeartLiveCount >=800)//0.8S�������߼��ͳ�βλ�ü��
		{
			BasketLifeFlag = 0;
			
		}
//		gCheckTrailLiveCount++;
//		if(gCheckTrailLiveCount >= 2000)
//		{
//			TrailPositionFlag = 0;
//		}
		gCheckFroCarLiveCount++;
		if(gCheckFroCarLiveCount >= 1000)
		{
			FrontCarPositionFlag = 0;
		}		
		



		
///////////////////////////////��Ƭ����//////////////////////////////		
		if(Q_GDSW == 0 && H_GDSW == 0)		
		{
			SwitchCount1++;
			if(SwitchCount1 == 1)
			{
				PocketCount1++;				
			}
			else if(SwitchCount1 >= 20)
			{
				SwitchCount1 = 20;
			}
		}
		if(Q_GDSW == 1 && H_GDSW == 1)	
		{
			SwitchCount1 = 0;
		}	
		
		
		if(Q_GDSW_B == 0 && H_GDSW_B == 0)		
		{
			SwitchCount1_B++;
			if(SwitchCount1_B == 1)
			{
				SwitchCount1_B++;			
			}
			else if(SwitchCount1_B >= 20)
			{
				SwitchCount1_B = 20;
			}
		}
	  if(Q_GDSW_B == 1 && H_GDSW_B == 1)
		{
			SwitchCount1_B = 0;
		}

		if(PocketCount1 >= PocketCount2)
			PocketCount = PocketCount1;
		else
			PocketCount = PocketCount2;
		CageNumber = PocketCount/2+PocketCount%2;//��ں�		
	
///////////////////////////////��Ƭ����//////////////////////////////			
		
		LoadingStation();//װ��̨����

	
	
		#if !DriveMode //��������
	
	
//	GetCarSpeed();//�ٶȻ�ȡ   ���벶���𲽲�׼ȷ		
				//	PWM����  
			if(Timer100msCount >=100)
			{
				Timer100msCount = 0;
				if(ReadSpeedFlag == 0)
				{
						gSpeedRA = PulseCntA*3.666f;
						PulseCntA = 0;
					
						gSpeedRB = PulseCntB*3.666f;
						PulseCntB = 0;
					
						if(gSpeedRA > gSpeedRB)
							gSpeedR = gSpeedRA;
						else
							gSpeedR = gSpeedRB;
						PIDA.PWM = Position_PIDA(&PIDA,MBSpeed);//PID������Ҫ���µ����� 
						Motor_Contral(CarGo,PIDA.PWM,MOTOR_ENA_B);//	
				}	
			}
			 

			if(MBSpeed == 0)
			{
				MOTOR_ENA_B = 0;
				CarDriveFlag = 0;
			}	
			else
			{
				MOTOR_ENA_B = 1;
				CarDriveFlag = 1;
			}
			
/////////////////////��ֹ�ٶȶ���������ʧ��//////////////////////////			
//			if(MOTOR_ENA_B==1)
//			{
//				if(PulseCounter <2)
//				{
//					ReadSpeedTimerCount++;
//					if(ReadSpeedTimerCount >=3000)//3S
//					{
//						MBSpeed = 0;
//						MOTOR_ENA_B = 0;
//						ReadSpeedFlag = 1;
//					}
//					else if(ReadSpeedTimerCount >3000)
//						ReadSpeedTimerCount = 3000;
//				}
//				else 
//					ReadSpeedTimerCount = 0;				
//			}

/////////////////////��ֹ�ٶȶ���������ʧ��//////////////////////////					
	
    #endif		
		
/* ��ͷ��ָʾ  */	
		if(!MF_SW)
		{
			HeadLed = 1;
		}
		else
		{
			if(TrainState == 1)
			{
				LedCnt++;
				if(LedCnt <=1000)
					HeadLed = 1;
				else if(LedCnt >1000 && LedCnt <=2000)
				{
					HeadLed = 0;
				}
				else
					LedCnt = 0;
			}
			else
				HeadLed = 0;
		}
/* ��ͷ��ָʾ  */	
		
	}
	
	TIM_ClearFlag(TIM2,TIM_IT_Update);
}


u8 LoadingStation(void)
{
///////////////////////////////������Ƭ����//////////////////////////////		
//	if(H_LD == 0 && Q_LD ==0 && H_LD_B == 1 && Q_LD_B ==1)
	if(H_LD == 0 && Q_LD ==0)
	{
		LD_ONFlag = 1;
		SwitchCount++;
		if(gSpeedR >= 10 && (TrainState == ST6 || TrainState == ST1))
			SwitchCount2++;
		
		if(SwitchCount == 1)
		{
			Pocket_A_Count1++;
			if(Pocket_A_Count1 >= Pocket_A_Count2)
				Pocket_A_Count = Pocket_A_Count1;
			else
				Pocket_A_Count = Pocket_A_Count2;
			SwitchCount2 = 0;
			if(Pocket_A_Count == Pocket4)//0802
			{
				PocketCountold = 0;
			}			
		}
		else if(SwitchCount >= 20 )
		{
			SwitchCount = 20;
		}
		
		if(SwitchCount2 >= 200)//������վ
		{
			InStationCount1++;
			if(InStationCount1 ==1)
			{
				InStationCount++;
				Pocket_A_Count = 0;
				PocketCountold = PocketCount;
				CageNumber = 0;PocketCount = 0;//�����ڼ���
			}
			InStationFlag = 1;
			StionStop = 1;
			PocketStep = 0;
			APPSendADDRFlag = 0;
			memset(AppSendAddr, 0, 21*sizeof(int));//����·���ַ	

			if(InStationCount1 >2)	
				InStationCount1 =2;			
			if(InStationCount >2)	
				InStationCount =2;
			
			if(TrainState == ST6)
			{
				TrainState = ST3;//��վ	
				StepState = 2;
			}
			else if(TrainState == ST1)		
			{
				//if(PocketCountold == (CaseNum*2-1) && InStationCount ==2)//��Ƭ������ȷ�˳��Լ�
				if(InStationCount ==2)//��Ƭ������ȷ�˳��Լ�	
				{
					TrainState = ST2;	
					StepState = 2;	
				}
			}
		}
	}
//	else if(H_LD == 1 && Q_LD ==1 && H_LD_B == 0 && Q_LD_B ==0)
	else if(H_LD_B == 0 && Q_LD_B ==0)
	{
		LD_B_ONFlag = 1;
		SwitchCount++;
		if(gSpeedR >= 10 && (TrainState == ST6 || TrainState == ST1))
			SwitchCount2++;
		
		if(SwitchCount == 1)
		{
			Pocket_A_Count2++;
			if(Pocket_A_Count1 >= Pocket_A_Count2)
				Pocket_A_Count = Pocket_A_Count1;
			else
				Pocket_A_Count = Pocket_A_Count2;			
			SwitchCount2 = 0;
			if(Pocket_A_Count == Pocket4)//0802
			{
				PocketCountold = 0;
			}			
		}
		else if(SwitchCount >= 20 )
		{
			SwitchCount = 20;
		}
		
		if(SwitchCount2 >= 200)//������վ
		{
			InStationCount1++;
			if(InStationCount1 ==1)
			{
				InStationCount++;
				Pocket_A_Count = 0;
				PocketCountold = PocketCount;
				CageNumber = 0;PocketCount = 0;//�����ڼ���
			}
			InStationFlag = 1;
			StionStop = 1;
			PocketStep = 0;
			APPSendADDRFlag = 0;
			memset(AppSendAddr, 0, 21*sizeof(int));//����·���ַ	

			if(InStationCount1 >2)	
				InStationCount1 =2;			
			if(InStationCount >2)	
				InStationCount =2;
			
			if(TrainState == ST6)
			{
				TrainState = ST3;//��վ	
				StepState = 2;
			}
			else if(TrainState == ST1)		
			{
				//if(PocketCountold == (CaseNum*2-1) && InStationCount ==2)//��Ƭ������ȷ�˳��Լ�
				if(InStationCount ==2)//��Ƭ������ȷ�˳��Լ�	
				{
					TrainState = ST2;	
					StepState = 2;	
				}
			}
		}
	}
	
	
	if(H_LD == 1 && Q_LD == 1 && H_LD_B == 1 && Q_LD_B ==1)
//	else if(H_LD == 1 && Q_LD ==1)
	{
		LD_ONFlag = 0;
		SwitchCount = 0;
		InStationCount1 = 0;
	}	
///////////////////////////////������Ƭ����//////////////////////////////	
	if(Pocket_A_Count == 0)
	{
		Pocket_A_Count1 = 0;
		Pocket_A_Count2 = 0;
	}
	if(PocketCount == 0)
	{
		PocketCount1 = 0;
		PocketCount2 = 0;
	}
}

void TIM3_IRQHandler(void)
{
	u8 i = 0;
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
	{
			gOldGetTim4Counter = gNewGetTim4Counter;
			gNewGetTim4Counter =  TIM_GetCounter(TIM3);
		if(gTim4UpdateCounter < 5)
		{		
			gErrorVaule = gNewGetTim4Counter + 65535 * gTim4UpdateCounter - gOldGetTim4Counter;
			GetSpeedVaule[gGetSpeedVauleCounter] = gErrorVaule;
			gGetSpeedVauleCounter++;
			if(gGetSpeedVauleCounter >= SPEED_VAULE)
			{
				gGetSpeedVauleCounter = 0;
			}
		}
		gTim4UpdateCounter = 0;
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	}
	else if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		gTim4UpdateCounter++;
		if(gTim4UpdateCounter >= 5) //HALL״̬��ʱ��ץ
		{
			gTim4UpdateCounter = 0;
			for(i = 0; i < SPEED_VAULE; i++)
			{
				GetSpeedVaule[i] = 0;
			}
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}



/*	��ʦ������������ʼ��	*/

void Motor_Contral(u8 Dir,u16 Speed,u8 MotorEnable)  
{

	BreakCargo = Dir;
	MOTOR_ENA_B = MotorEnable; 
	
	//DIR 0 ǰ��
	MOTOR_DIRA = Dir;	
	MOTOR_DIRB = ~Dir;
	TIM_SetCompare1(TIM14,Speed);		
	
}


//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM14_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);	//ʹ�ܶ�ʱ��14ʱ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM14);	//TIM14
		
	//���TIM14 CH1��PWM���岨��	GPIOA.7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//TIM14 CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO		
	
 
   //��ʼ��TIM14
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM14 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM14 OC1
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);  //ʹ��TIM14��CCR2�ϵ�Ԥװ�ؼĴ���	
	
	TIM_Cmd(TIM14, ENABLE);  //ʹ��TIM3
	

}

void TIM3_Int_Init(u16 arr, u16 psc)
{

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);	//TIM3
		
	//���TIM3 CH2��PWM���岨��	GPIOA.6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//TIM3 CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO			
	
	
	
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM3, DISABLE);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);



	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�4��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_ClearFlag(TIM3, TIM_IT_CC1 | TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
	TIM_SetCounter(TIM3,0);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����

//  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
//  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
//	TIM_SetCounter(TIM4,0);
//  TIM_Cmd(TIM4, ENABLE); 

}

void GetCarSpeed(void)
{
	u8 i = 0;
	u32 buf = 0;
	//����С���ٶ�
	for(i = 0; i < SPEED_VAULE; i++)
	{
		buf += GetSpeedVaule[i];
	}
	gSum = buf;
	gSpeedRA = (u32)(22.0 / (1.0 /1000000 * gSum));

	
	buf = 0;
}


/*	��ʦ������������ʼ��	*/


void TM1620_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIODʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOEʱ��
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//SDA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO		
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//SSCL
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO			
	
	
}

void ryydelay(void)
{ 
//	delay_us(50);
}

void Start(void)
{
//    PA_ODR |=0x02;   //CLK=1;
	SCLH;//CLK=1
	ryydelay();
//    PA_ODR |=0x04;    //DAT=1;
	SDAH;//DAT=1

	ryydelay();
//    PA_ODR &=~0x04;     //DAT=0;
	SDAL;//DAT=0
	ryydelay();

//    PA_ODR &=~0x02;     //CLK=0;
	SCLL;//CLK=0
	ryydelay();
//	SCLK=0;
//	_asm("nop");//NOP();
}
void Stop(void)
{
//    PA_ODR &=~0x04;     //DAT=0;
	SDAL;//DAT=0
	ryydelay();
//    PA_ODR |=0x02;   //CLK=1;
	SCLH;//CLK=1
	ryydelay();
//    PA_ODR |=0x04;    //DAT=1;
	SDAH;//DAT=1
	ryydelay();
//	_asm("nop"); //NOP();
}

void TM1620_Write(unsigned char	DATA)
{
	unsigned char i;

	for(i = 0; i < 8; i++)
	{
//    PA_ODR &=~0x02;     //CLK=0;
		SCLL;//CLK=0
		ryydelay();
//		_asm("nop"); //NOP();
		if(DATA & 0x80)
//    	PA_ODR |=0x04;    //DAT=1;
			SDAH;//DAT=1
		else
//    	PA_ODR &=~0x04;     //DAT=0;
			SDAL;//DAT=0
		ryydelay();
//		_asm("nop"); //NOP();
//		_asm("nop"); //NOP();
//    PA_ODR |=0x02;   //CLK=1;
		SCLH;//CLK=1
		ryydelay();
//    PA_ODR &=~0x02;     //CLK=0;
		SCLL;//CLK=0
		ryydelay();
		DATA <<= 1;
//    PA_ODR &=~0x02;     //CLK=0;
		SCLL;//CLK=0
		ryydelay();
//		_asm("nop"); //NOP();
	}
//	 	PA_ODR &=~0x04;     //DAT=0;
	SDAL;//DAT=0
	ryydelay();
//	  PA_ODR |=0x02;   //CLK=1;
	SCLH;//CLK=1
	ryydelay();
//    PA_ODR &=~0x02;     //CLK=0;
	SCLL;//CLK=0

}

void Write_DATA(unsigned char add, unsigned char DATA)
{
//	Write_COM(0x44);
	Start();
	TM1620_Write(add);
	TM1620_Write(DATA);
	Stop();
}

void TM1620_init(void)
{

	SCLH;
	SDAH;
	Start();
	TM1620_Write(0x44);
	Stop();
	Start();
	TM1620_Write(0x8f);
	Stop();
	Write_DATA(0x48, 0x31);
	
	Write_DATA(0x48, 0x31);
	Write_DATA(0X68, 0x00);
	Write_DATA(0X6A, 0x00);
	Write_DATA(0X6C, 0x00);			
}

void LED_Tube_Choose_DisPlay1(u8 num1,u8 num2,u8 num3)
{
	Write_DATA(0X68,num1);
	Write_DATA(0X6A,num2);
	Write_DATA(0X6C,num3);
	
}


void Display (void)
{
	u16 buf,buf1,buf2 = 0;
	
//	KeyValueold = KeyValuenew;
//	KeyValuenew = KeyValue;
	if(KeyValue ==1 && KeyMode == 1)
	{	
		WPS = 0;//д�����ر�
		TrainHeadNum++;
		if(TrainHeadNum>21)TrainHeadNum = 1;
			
		EEPROM_Write_Byte(19, TrainHeadNum);	
		delay_ms(5);
		WPS = 1;
		TrainHeadNum = EEPROM_Read_Byte(19);
		KeyValue = 0;
	}

	else if(KeyValue ==3)
	{
		KeyMode++;
		if(KeyMode > 3)
			KeyMode = 1;
		KeyValue = 0;
	}
	
	
	
	buf = LedNumDisplay/10%10;
	buf1 = LedNumDisplay%10;	
	buf2 = LedNumDisplay/100;
	if(KeyMode == 1)
	{
		LedNumDisplay = TrainHeadNum;
		LED_Tube_Choose_DisPlay1(gShowAlphabetData[3], gShowNumberData[buf], gShowNumberData[buf1]);	
	}
	else if(KeyMode == 2)
	{
		LedNumDisplay = Pocket_A_Count;
		LED_Tube_Choose_DisPlay1(gShowAlphabetData[4], gShowNumberData[buf], gShowNumberData[buf1]);		
	}
	else if(KeyMode == 3)
	{
		LedNumDisplay = CageNumber;
		LED_Tube_Choose_DisPlay1(gShowNumberData[buf2], gShowNumberData[buf], gShowNumberData[buf1]);		
	}		
}

void KeyScan(void)
{
	if(SW2 == 0)
	{
		KeyUpCount++;	
		KeyUpFlag = 1;
	}
	else
	{
		if(KeyUpFlag ==1)
		{
			if(KeyUpCount >2 && KeyUpCount <=500)
			{
				KeyValue = 1;
			}
			else if(KeyUpCount >500 && KeyUpCount <=1500) 
			{
				KeyValue = 2;
			}
			else if(KeyUpCount >=2000) 
			{
				KeyValue = 3;
				KeyUpCount = 2000;
			}		
			
			KeyUpCount = 0;
			KeyUpFlag = 0;
		}
	}
	
	
	
	
	if(SW1 == 0)
	{
		KeyUpCount2++;	
		KeyUpFlag2 = 1;
	}
	else
	{
		if(KeyUpFlag2 ==1)
		{
			if(KeyUpCount2 >2 && KeyUpCount2 <=500)
			{
				KeyValue2 = 1;
			}
			else if(KeyUpCount2 >500 && KeyUpCount2 <=1500) 
			{
				KeyValue2 = 2;
			}
			else if(KeyUpCount2 >=2000) 
			{
				KeyValue2 = 3;
				KeyUpCount2 = 2000;
			}		
			
			KeyUpCount2 = 0;
			KeyUpFlag2 = 0;
		}
	}	
	
	
}


void ChargeStop (void)
{
	if(ChargePosition - CageNumber >= 20)
	{
		MBSpeed = 180;
		BreakStopFlag = 0;
		MOTOR_ENA_B = 1;			
	}
	else if((ChargePosition - CageNumber < 20) && (ChargePosition - CageNumber >= 15))
	{
		MBSpeed = 100;
		BreakStopFlag = 0;
		MOTOR_ENA_B = 1;	
	}
	else if((ChargePosition - CageNumber < 15) && (ChargePosition - CageNumber >= 10))
	{
		MBSpeed = 60;
		BreakStopFlag = 0;
		MOTOR_ENA_B = 1;
	}	
	else if((ChargePosition - CageNumber < 10) && (ChargePosition - CageNumber >= 5))
	{
		MBSpeed = 40;
		BreakStopFlag = 0;
		MOTOR_ENA_B = 1;
	}	
	else if((ChargePosition - CageNumber < 5) && (ChargePosition - CageNumber >= 1))
	{
		MBSpeed = 20;
		BreakStopFlag = 0;
		MOTOR_ENA_B = 1;
	}	
	else if(ChargePosition - CageNumber <1)
	{
		MoterBrakeFlag = 1;//ɲ��
		MBSpeed = 0;	
		MOTOR_ENA_B = 0;
	}	
}





