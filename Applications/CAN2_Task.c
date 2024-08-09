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
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
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
		
		
		digitalIncreasing(&getCAN2_Task()->loops);        

	}
}



/*
***************************************************
��������CAN2_RX0_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************
*/
u32 rxbuf2;
void CAN2_RX0_IRQHandler(void){
	CanRxMsg can2_rx_msg;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0);		
		CAN_Receive(CAN2, CAN_FIFO0, &can2_rx_msg);
		
		rxbuf2=can2_rx_msg.StdId;	
		digitalIncreasing(&OdriveData.OdError.errorCount);
		/*********�������Զ��岿��**********/
		

	}
}


/*
***************************************************
��������CAN2_TX_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************
*/
void CAN2_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);

		/*********�������Զ��岿��**********/
        
        
	}
}



void CAN2DataInit(void){
	getsupervisorData()->taskEvent[CAN2_Task] = xTaskCreate(CAN2_TaskUpdateTask,"CAN2_Task",CAN2_Task_STACK_SIZE,NULL,CAN2_Task_PRIORITY,&CAN2Data.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
