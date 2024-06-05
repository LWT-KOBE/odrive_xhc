#include "application.h"
#include "usartx.h"
#include "board.h"
#include "balance.h"
balanceStruct_t balanceData;
balance_target_t Angle_Goal;
balance_target_t Motor_SpeedA_Goal;
balance_target_t Motor_SpeedB_Goal;

balanceStruct_t* getbalanceData(){
    return &balanceData;
}

float pbuf[10];
void balanceGlobalInit(void){
//	uart1_init(115200);
//	uart2_init(9600);
//	uart5_init(115200);
//	cigan_Init();
}

void balanceUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getbalanceData()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,CONTROL_NORMAL_PERIOD);
        //防止重复初始化
		if(!balanceData.dataInitFlag){	
            //所有控制全部初始化            
			balanceGlobalInit();
			JY60_Calibration();
			digitalHi(&getbalanceData()->dataInitFlag);
		}
		//获取imu数据信息
		JY60_Get(pbuf);
		
		//
		if(Angle_Goal.finish == 1){
			OdriveData.SetVel[0].float_temp = Motor_SpeedA_Goal.target / (M_PI * 0.075f);
			OdriveData.SetVel[1].float_temp = -Motor_SpeedA_Goal.target / (M_PI * 0.075f);
		}
		
		if(Angle_Goal.finish == 0){
			OdriveData.SetVel[0].float_temp =  PID_angel(Angle_Goal.target,pbuf[9],2);
			OdriveData.SetVel[1].float_temp =  PID_angel(Angle_Goal.target,pbuf[9],2);
		}
		
		
		digitalIncreasing(&getbalanceData()->loops);        

	}
}

void balanceInit(void){
	/* uxPriority
	在调用任务创建函数xTaskCreate()时就为任务指定了优先级，在启动了任务调度函数之后，
	可以通过调用xTaskPrioritySet()函数来修改任务的优先级。系统中理论上没有对优先级做出上限要求，
	只要内存足够大就可以创建多个任务，设置不同的优先级，不同的任务可以赋予相同的优先级，
	在FreeRTOSConfig.h中configMAX_PRIORITIES 的大小决定最大优先级个数，
	优先级是从0开始到configMAX_PRIORITIES - 1为止，优先级为0的任务的优先级最高。
     系统中的调度器总是先让优先级最高的任务先运行，如果多个任务拥有相同的优先级，
	那么调度器将会使得任务轮流执行一个时间片。这里的时间片等于1/心跳时钟频率，心跳时钟即滴答时钟。
	可以通过设置FreeRTOSConfig.h中的configTICK_RATE_HZ来设置心跳中断时钟，
	当configTICK_RATE_HZ赋值为100HZ时，每100模式发生一次心跳时钟中断，系统会执行相应的中断函数。	
	*/
	
	getsupervisorData()->taskEvent[CONTROL_TASK] = xTaskCreate(balanceUpdateTask,"BALANCE",BALANCE_STACK_SIZE,NULL,BALANCE_PRIORITY,&balanceData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
