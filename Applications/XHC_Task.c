#include "application.h"
#include "usartx.h"
#include "board.h"
XHC_TaskStruct_t XHCData;



XHC_TaskStruct_t* getXHC_Task(){
    return &XHCData;
}

/* 放置初始化 */
void XHC_TaskGlobalInit(void){
	
}

void XHC_TaskUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getXHC_Task()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,XHC_Task_NORMAL_PERIOD);
        //防止重复初始化
		if(!XHCData.dataInitFlag){	
            //所有控制全部初始化            
			XHC_TaskGlobalInit();																																							
			digitalHi(&getXHC_Task()->dataInitFlag);
			
		}
		
		
		digitalIncreasing(&getXHC_Task()->loops);        

	}
}

void XHCDataInit(void){
	getsupervisorData()->taskEvent[XHC_Task] = xTaskCreate(XHC_TaskUpdateTask,"XHC_Task",XHC_Task_STACK_SIZE,NULL,XHC_Task_PRIORITY,&XHCData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
