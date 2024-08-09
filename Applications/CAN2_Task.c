#include "application.h"
#include "usartx.h"
#include "board.h"
CAN2_TaskStruct_t CAN2Data;



CAN2_TaskStruct_t* getCAN2_Task(){
    return &CAN2Data;
}

/* 放置初始化 */
void CAN2_TaskGlobalInit(void){
	//初始化CAN2 波特率125K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,24,CAN_Mode_Normal);
	//初始化CAN2 波特率250K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,12,CAN_Mode_Normal);
	//初始化CAN2 波特率1M
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
}

void CAN2_TaskUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getCAN2_Task()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,CAN2_Task_NORMAL_PERIOD);
        //防止重复初始化
		if(!CAN2Data.dataInitFlag){	
            //所有控制全部初始化            
			CAN2_TaskGlobalInit();																																							
			digitalHi(&getCAN2_Task()->dataInitFlag);
			
		}
		
		
		digitalIncreasing(&getCAN2_Task()->loops);        

	}
}



/*
***************************************************
函数名：CAN2_RX0_IRQHandler
功能：CAN2接收中断
备注：
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
		/*********以下是自定义部分**********/
		

	}
}


/*
***************************************************
函数名：CAN2_TX_IRQHandler
功能：CAN2发送中断
备注：
***************************************************
*/
void CAN2_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);

		/*********以下是自定义部分**********/
        
        
	}
}



void CAN2DataInit(void){
	getsupervisorData()->taskEvent[CAN2_Task] = xTaskCreate(CAN2_TaskUpdateTask,"CAN2_Task",CAN2_Task_STACK_SIZE,NULL,CAN2_Task_PRIORITY,&CAN2Data.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
