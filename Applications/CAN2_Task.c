#include "application.h"
#include "usartx.h"
#include "board.h"
CAN2_TaskStruct_t CAN2Data;



CAN2_TaskStruct_t* getCAN2_Task(){
    return &CAN2Data;
}

/* ���ó�ʼ�� */
void CAN2_TaskGlobalInit(void){
	//��ʼ��CAN2 ������125K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,24,CAN_Mode_Normal);
	//��ʼ��CAN2 ������250K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,12,CAN_Mode_Normal);
	//��ʼ��CAN2 ������1M
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
}

void CAN2_TaskUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getCAN2_Task()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,CAN2_Task_NORMAL_PERIOD);
        //��ֹ�ظ���ʼ��
		if(!CAN2Data.dataInitFlag){	
            //���п���ȫ����ʼ��            
			CAN2_TaskGlobalInit();																																							
			digitalHi(&getCAN2_Task()->dataInitFlag);
			
		}
		//////////////////
		TrainContral();//���� 
		//////////////////
		Display();
		
		KeyScan();
/*  �𳵹����뾯��   */		
		if(FrontCarPositionFlag == 0)
			TrainWarning |= 0x01;//ǰ��λ�ö�ʧ
		else
			TrainWarning &= ~0x01;//�������
		
		if(MBSpeed != 0)
		{
			if(gSpeedRA <30)
				TrainWarning |= 0x02;//���1�쳣
			else
				TrainWarning &= ~0x02;//������1�쳣
			
			if(gSpeedRB <30)
				TrainWarning |= 0x04;//���2�쳣
			else
				TrainWarning &= ~0x04;//������2�쳣	
				
		}
		if(gSpeedR >30)
		{
			SensorWarningDelay1++;
			SensorWarningDelay2++;
			SensorWarningDelay3++;
			SensorWarningDelay4++;
			SensorWarningDelay5++;
			SensorWarningDelay6++;
			SensorWarningDelay7++;
			SensorWarningDelay8++;
/*		SensorWarning		1			*/			
			if(Q_LD == 0)
			{
				if(SensorWarningDelay1 >=700)
					SensorWarning |= 0x01;//LD1�쳣
				else if(SensorWarningDelay1 > 700)
					SensorWarningDelay1 = 700;
				else
					SensorWarning &= ~0x01;//���LD1�쳣
			}
			else
				SensorWarningDelay1 = 0;
			
/*		SensorWarning		2			*/			
			if(H_LD == 0)
			{
				if(SensorWarningDelay2 >=700)
					SensorWarning |= 0x02;//LD2�쳣
				else if(SensorWarningDelay2 > 700)
					SensorWarningDelay2 = 700;
				else
					SensorWarning &= ~0x02;//���LD2�쳣
			}
			else
				SensorWarningDelay2 = 0;	

/*		SensorWarning		3			*/			
			if(Q_LD_B == 0)
			{
				if(SensorWarningDelay3 >=700)
					SensorWarning |= 0x04;//LD1B�쳣
				else if(SensorWarningDelay3 > 700)
					SensorWarningDelay3 = 700;
				else
					SensorWarning &= ~0x04;//���LD1B�쳣
			}
			else
				SensorWarningDelay3 = 0;	

/*		SensorWarning		4			*/			
			if(H_LD_B == 0)
			{
				if(SensorWarningDelay4 >=700)
					SensorWarning |= 0x08;//LD2B�쳣
				else if(SensorWarningDelay4 > 700)
					SensorWarningDelay4 = 700;
				else
					SensorWarning &= ~0x08;//���LD1B�쳣
			}
			else
				SensorWarningDelay4 = 0;	

/*		SensorWarning		5			*/			
			if(Q_GDSW == 0)
			{
				if(SensorWarningDelay5 >=700)
					SensorWarning |= 0x10;//ST1�쳣
				else if(SensorWarningDelay5 > 700)
					SensorWarningDelay5 = 700;
				else
					SensorWarning &= ~0x10;//���ST1�쳣
			}
			else
				SensorWarningDelay5 = 0;	

/*		SensorWarning		6			*/			
			if(H_GDSW == 0)
			{
				if(SensorWarningDelay6 >=700)
					SensorWarning |= 0x20;//ST2�쳣
				else if(SensorWarningDelay6 > 700)
					SensorWarningDelay6 = 700;
				else
					SensorWarning &= ~0x20;//���ST2�쳣
			}
			else
				SensorWarningDelay6 = 0;	

/*		SensorWarning		7			*/			
			if(Q_GDSW_B == 0)
			{
				if(SensorWarningDelay7 >=700)
					SensorWarning |= 0x40;//ST1B�쳣
				else if(SensorWarningDelay7 > 700)
					SensorWarningDelay7 = 700;
				else
					SensorWarning &= ~0x40;//���ST1B�쳣
			}
			else
				SensorWarningDelay7 = 0;	

/*		SensorWarning		8			*/			
			if(H_GDSW_B == 0)
			{
				if(SensorWarningDelay8 >=700)
					SensorWarning |= 0x80;//ST2B�쳣
				else if(SensorWarningDelay8 > 700)
					SensorWarningDelay8 = 700;
				else
					SensorWarning &= ~0x80;//���ST2B�쳣
			}
			else
				SensorWarningDelay8 = 0;			
			
		}
 
/*  �𳵹����뾯��   */			
		
		
		
		digitalIncreasing(&getCAN2_Task()->loops);        

	}
}



//     Frame
// nodeID | CMD
// 6 bits | 5 bits
CanRxMsg can1_rx_msg;
u32 rxbuf3;
void CAN1_RX0_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		// ����жϱ�־�ͱ�־λ
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
		
		// �ӽ��� FIFO �ж�ȡ��Ϣ		
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		SaveData(can1_rx_msg);	
		CheckCarCanCmd();


		
		digitalIncreasing(&OdriveData.OdError.errorCount);

		/*********�������Զ��岿��**********/
		


	}
}


/*
***************************************************
��������CAN1_TX_IRQHandler
���ܣ�CAN1�����ж�
��ע��
***************************************************
*/
void CAN1_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

		/*********�������Զ��岿��**********/
        OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_MOTOR_ERROR);
        
	}
}




void CAN2DataInit(void){
	getsupervisorData()->taskEvent[CAN2_Task] = xTaskCreate(CAN2_TaskUpdateTask,"CAN2_Task",CAN2_Task_STACK_SIZE,NULL,CAN2_Task_PRIORITY,&CAN2Data.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
