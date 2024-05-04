#include "application.h"
#include "usartx.h"
controlStruct_t controlData;
int Menu=1,Menu1=0,Menu2=0;
float Set_Cur=0,Set_Pos=0,Set_Vel=0;

controlStruct_t* getcontrolData(){
    return &controlData;
}

void congtrolGlobalInit(void){
	uart1_init(115200);
}

void controlUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getcontrolData()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,CONTROL_NORMAL_PERIOD);
        //防止重复初始化
		if(!controlData.dataInitFlag){	
            //所有控制全部初始化            
			congtrolGlobalInit();																																							
			digitalHi(&getcontrolData()->dataInitFlag);
		}  		
		
		digitalIncreasing(&getcontrolData()->loops);        

	}
}

void controlInit(void){
	getsupervisorData()->taskEvent[CONTROL_TASK] = xTaskCreate(controlUpdateTask,"CONTROL",CONTROL_STACK_SIZE,NULL,CONTROL_PRIORITY,&controlData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
