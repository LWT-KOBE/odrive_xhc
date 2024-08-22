#include "app.h"
#include "stdlib.h"
#include <string.h>
#include "application.h"
#include "MyADC.h"  
CanRxMsg RxMessage;
CanTxMsg TxMessage;


u32 CAN_ID = 0;
u8 RxRAM0[8];
u8 Rxflag1 = 0;

u32 PocketCount,PocketCountold = 0;//U型挡片计数
u32 Pocket_A_Count,OldPocket_A_Count,NewPocket_A_Count = 0;//金属挡片
u16 LongLDCount = 0;
u16 Timer40msCount = 0;
u16 Timer10msCount = 0;
u16 Timer100msCount = 0; 
u16 gCheckHeartLiveCount = 0;//车厢在线检测
u16 gCheckTrailLiveCount = 0;
u16 gCheckFroCarLiveCount = 0; 
u16 SwitchCount1 = 0;
u32 SwitchCount2 = 0; 
u8 SwitchCount = 0; 
 
 
 
float gSpeedR = 0;
u8 ReadSpeedFlag = 0;//电机读速异常1
u16 ReadSpeedTimerCount = 0;
u8 MotorDir = 0; //
u8 MotorGoFlag = 0;
u32 APPBasketNum = 0x00000000;//车号B3  车厢号B2  投递格口号B0\1

u32 APPSendADDRFlag = 0;//APP下发地址标志位  21个位代表21个车厢下发的地址
u8 APPSendADDRACKFlag = 0;//APP下发地址标志位-自动模式
int AppSendAddr[21];//下发地址  车号B3  车厢号B2  投递格口号B0\1
u32 AppAddrACK = 0;//应答回复

int LastDeliverResultNum[21];//车号B1  车厢号B2  投递格口号B3\4 
unsigned short int LastPoketNum[21];//上一圈挡片数量

u32 BasketApplicationFlag = 0;//21个位代表21个车厢对应申请地址状态    1--申请地址标志
u32 DeliverResultFlag = 0;//21个位代表21个车厢对应上报结果状态  1--上报
u32 DeliverResultACKFlag = 0;//21个位代表21个车厢对应上报结果应答状态  1--应答

u8 BasketError[24];//车厢故障缓存

u32 NFCNUM,NFCNUMold,NFCNUMnew = 0;//车头位置
u32 TailstockNFC = 0;

u16 CarTailNFCNum = 0;//车尾位置
u8 MoveOneFlag = 0;

u8 HeadSendBasketFlag = 0; // 0x01--下发地址  0x02--停止下发  0x04--下发回复投递结果应答信号  0x08--停止投递结果应答下发
u32 BasketPocketNum,BasketPocketNumold,BasketPocketNumnew,LastDeliverResult = 0;

u32 BasketReciveAddrFlag = 0;//车厢收到地址标志位 21bit缓存21个车厢--自动模式

u16 BasketNumACKADDRToAPP = 0;

u16 PositionTail = 0;//挡片车尾位置
u16 FrontCarPositionNFC = 0;//前车位置，指的是前车最后一节车厢的位置

u16 testflag = 0;

u16 FrontCarPositionPokec = 0;//前车车尾位置
u16 FrontCarPositionPokec1 = 0;//前车车尾位置
u16 FrontCarPositionPokec2 = 0;//前车车尾位置

u8 FrontCarPositionFlag = 0;//前车位置获取成功标志位
//u8 TrailPositionFlag = 0;//车尾位置在线标志

u16 FrontCarPositionHead = 0;//前车车头位置位置
u16 CarPositionPocket = 0;//挡片数+接近挡片
u16 CarDistance = 0;  //与前车距离格口数量
u8 SendDataFlag = 0;//收到前车的广播允许发送数据标志
u8 ReciveRightFlag = 0;//正常接收标志位
u8 TrainHeadNum = 0;   //车号
u8 CarSignalNum = 0;//超时等待发送时间计算
u16 TrainBasketPosition = 0;//车尾位置格口号

float gPowerValueFloat = 0;
u16 powervol = 0;
unsigned  int gCarBatPower = 0; //电源电压
u16 carpower = 0;

u8 ST1ACKFlag = 0;//自检指令标志位

u8 TrainApplyForExitFlag = 0;//出站申请标志位
u8 BreakCargo = 0;
u8 CarGo = 0;//0--前进
u8 CarBack = 1;//后退
u8 ScranTrain = 0;//跟随标志--1
u8 StionStop  = 0;   		//进站停车挡片

u8 CarDirection = 0;//0-stop 1前进  2后退
u16 IDMaxNum = 0;//最大格口号
u8 CarScranFlag = 0;
u8 CarDriveFlag = 0;

u8 ApplicationtTavel=9;	//申请发车挡片
u8 ForceTravel1 =10;
u8 ForceTravel2 =11;
u8 ForceTravel3 =29;

u8 ConfigrationFlag = 0;//配置标志位 1--配置
u8 InStationFlag = 0;//进站标志
u8 InStationCount = 0;
u8 InStationCount1 = 0;
u32 BasketLifeFlag = 0;//车厢在线标志

u8 ReadDataFlag = 0;
u8 CanSendCoount = 0;

//u8 DriveMode = 1;//电机驱动类型 0--老驱动板 1--ODRIVE

////////////////////////////设置参数///////////////////////////

u16 CaseNum = 77;//格口数   ADDR 0-1
u8 Pocket4=11;						//装载挡片 ADDR 2
u8 TrainBasketMaxNum = 20;//车厢数量，即车尾编号 ADDR 3
u8 TrainMode = 1;//扫码模式  //0--手动扫码    1--自动扫码  ADDR 4
u8 HigSpeed = 200;//ADDR 5
u8 MidSpeed = 150;//ADDR 6
u8 LowSpeed = 80;//ADDR 7

u8 OutStationQuicken = 4;//出站加速格口 ADDR 8

u8 FollowHigSpeed = 20;//跟随间隔高速 ADDR 9
u8 FollowMidSpeed = 10;//跟随间隔中速 ADDR 10
u8 FollowLowSpeed = 5;//跟随间隔低速 ADDR 11

u8 InStationHigtoMidSpeed = 10;//进入装载区前高速减中速 ADDR 12
u8 InStationMidtoLowSpeed = 4;//进入装载区前中速减低速 ADDR 13
u8 TrainMaxNum = 3;//火车数量    ADDR 14
u16 RecycleCaseNum = 77;//回收格口  ADDR 15-16

u8 MFContralZS = 1;//跟随区间格口号正数  ADDR 17
u8 MFContralDS = 2;//跟随区间格口号倒数  ADDR 18

////////////////////////////设置参数///////////////////////////




void IAP_APP_Init(void)
{
	SCB->VTOR = FLASH_BASE | IAP_Bootloat_SIZE; //IAP_Bootloat_SIZE:0x2000=8K 字节
//	GetLockCode(&g_Lock_Code);						//获取芯片唯一ID
}


//void GetCarSpeed(void)
//{
//	u8 i = 0;
//	u32 buf,gSum = 0;
//	
//	//计算小车速度
//	for(i = 0; i < SPEED_VAULE; i++)
//	{
//		buf += GetSpeedVaule[i];
//	}
//	gSum = buf;
//	gSpeedR = (u32)(60.0 / (1.0 / 1000000 * gSum));
//	buf = 0;
//}



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
				BasketError[i] = RxRAM0[3];//缓存各车厢故障
				BasketLifeFlag |= 1 << i;
				gCheckHeartLiveCount = 0;//车厢在线检测时间清零
			}
			if(TrainMode == 0)//手动扫码
			{
				if((RxRAM0[0] & 0x0f)== 0x01)//申请地址
				{
					if(CAN_ID == i+1)	
					{
						Icount1[i]++;
						if(Icount1[i] == 1)
						{
							MoveOneFlag = 0x01;
							BasketApplicationFlag |= 1 << i;//缓存各车厢申请状态
							IntermediateVariable2 = 0;
						}
						if(Icount1[i] >= 0xee)
							Icount1[i] = 0xee;
						
						Icount[i] = 0; 
					}
				}	
				else if((RxRAM0[0]& 0x0f) == 0x02)//车厢收到地址停止下发
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
							BasketApplicationFlag &= ~(0x00000001 << i);//清除申请状态
						}
						if(Icount[i] >= 0xee)
							Icount[i] = 0xee;
						Icount1[i] = 0;
					}
				}
			}
			else//自动扫码
			{
				if(CAN_ID == i+1)		
				{
					if((RxRAM0[0]& 0x0f) == 0x02)//车厢收到地址停止下发
					{					
						Icount[i]++;
						APPSendADDRFlag &= ~(1<<i);
						if(Icount[i] == 1)
						{
//							APPSendADDRFlag &= ~(1<<i);
							HeadSendBasketFlag &= ~0x03;
							HeadSendBasketFlag |= 0x02;	
							BasketReciveAddrFlag |= (1<<i);//车厢收到地址
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
			
			if((RxRAM0[0] & 0xf0)== 0x10)//收到车厢上报结果
			{
				if(CAN_ID == i+1)	
				{	
					Icount2[i]++;
					if(Icount2[i] == 1)
					{					
						LastDeliverResultNum[i] = RxRAM0[1] <<8 | RxRAM0[2];  //缓存各车厢投递结果
						LastPoketNum[i] = RxRAM0[4] <<8 | RxRAM0[5];
						DeliverResultFlag |= 1 << i;//缓存各车厢上报状态
					}
					if(Icount2[i] >= 0xee)
						Icount2[i] = 0xee;					
					
				}	
			}
			else if((RxRAM0[0]& 0xf0) == 0x20)//车厢收到投递结果确认应答信号
			{
				if(CAN_ID == i+1)	
				{					
					HeadSendBasketFlag &= ~0x0c;
					HeadSendBasketFlag |= 0x08;			
					DeliverResultFlag &= ~(1 << i);//清除上报状态
					Icount2[i] = 0;
				}
			}
		}
//	#if !DriveMode
//		if(CAN_ID == TrainBasketMaxNum-1)//获取车尾位置
//		{
//			PositionTail = RxRAM0[4]<<8 | RxRAM0[5];
//			PositionTail = PositionTail/2+PositionTail%2;
//			TrailPositionFlag = 1;
//			gCheckTrailLiveCount = 0;
//		}	//0814
//	#else
		
//		if(CAN_ID == TrainBasketMaxNum-1)//获取车尾位置
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
	gPowerValueFloat = ((1.0 * gCarBatPower) * 3.3 * (15.0 + 2.0) / 4096.0 / 2.0);
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
	
	TIM_Cmd(TIM2,ENABLE);//失能
	
	NVIC_InitStrct.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStrct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStrct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStrct.NVIC_IRQChannelCmd=ENABLE;
	
	NVIC_Init(&NVIC_InitStrct);	
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM2,TIM_IT_Update)==SET)//
//	if(TIM2->SR&0X0001) //Overflow interrupt //溢出中断
	{	
		if(gSpeedR >= 10 && (TrainState == ST6 || TrainState == ST1))
			LongLDCount++;
		
		Timer40msCount++;
		Timer10msCount++;
		Timer100msCount++;
	
		gCheckHeartLiveCount++;
		if(gCheckHeartLiveCount >=800)//0.8S车厢在线检测和车尾位置检测
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
		


///////////////////读电压//////////////////////////////////////////////	

		if(Timer40msCount>=40)//
		{
			Timer40msCount = 0;			
			ADC_Value = MyADC_GetValue();
			garry_ch0[gGetAdcCounter] = ADC_Value[0]; //电源电压
			gGetAdcCounter++;
			if(gGetAdcCounter >= 10)
			{
				gGetAdcCounter = 0;
			}		
			GetAdcAverage();//10次ADC平均值	
			PowerValueLedShow();	//电量
		}
///////////////////读电压//////////////////////////////////////////////
		
///////////////////////////////挡片计数//////////////////////////////		
		if(Q_GDSW == 0 && H_GDSW == 0)
		{
			SwitchCount1++;
			if(SwitchCount1 == 1)
			{
				PocketCount++;	
				CageNumber = PocketCount/2+PocketCount%2;//格口号				
			}
			else if(SwitchCount1 >= 20)
			{
				SwitchCount1 = 20;
			}
		}
		else if(Q_GDSW == 1 && H_GDSW == 1)
		{
			SwitchCount1 = 0;
		}
	
///////////////////////////////挡片计数//////////////////////////////			



///////////////////////////////金属挡片计数//////////////////////////////		
	
	if(H_LD == 0 && Q_LD ==0)
	{
		SwitchCount++;
		if(gSpeedR >= 10 && (TrainState == ST6 || TrainState == ST1))
			SwitchCount2++;
		
		if(SwitchCount == 1)
		{
			Pocket_A_Count++;
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
		
		if(SwitchCount2 >= 200)//触发进站
		{
			InStationCount1++;
			if(InStationCount1 ==1)
			{
				InStationCount++;
				Pocket_A_Count = 0;
				PocketCountold = PocketCount;
				CageNumber = 0;PocketCount = 0;//清零格口计数
			}
			InStationFlag = 1;
			StionStop = 1;
			PocketStep = 0;
			APPSendADDRFlag = 0;
			memset(AppSendAddr, 0, 21*sizeof(int));//清除下发地址	

			if(InStationCount1 >2)	
				InStationCount1 =2;			
			if(InStationCount >2)	
				InStationCount =2;
			
			if(TrainState == ST6)
			{
				TrainState = ST3;//进站	
				StepState = 2;
			}
			else if(TrainState == ST1)		
			{
				//if(PocketCountold == (CaseNum*2-1) && InStationCount ==2)//挡片计数正确退出自检
				if(InStationCount ==2)//挡片计数正确退出自检	
				{
					TrainState = ST2;	
					StepState = 2;	
				}
			}
		}
	}
	else if(H_LD == 1 && Q_LD ==1)
	{
		SwitchCount = 0;
		InStationCount1 = 0;
	}	
///////////////////////////////金属挡片计数//////////////////////////////	
//	TrainHeadNum = KeyNumHead; //读取车头号			
	TrainContral();//控制 
	gSpeedR = OdReceivedData.vel_estimate[1].float_temp *22.0;//读取速度
	TIM_ClearFlag(TIM2,TIM_IT_Update);
//	TIM2->SR&=~(1<<0); //Clear the interrupt flag bit //清除中断标志位
	}
}




