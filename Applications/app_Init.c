#include "board.h"

taskInit_t taskInitData;
void appInit(void *Parameters){
	taskInitData.eventGroups = NULL;	//事件标志组清零
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	taskInitData.eventGroups = xEventGroupCreate();
	/*- 在此处下写入初始化任务 -------*/
	configInit();               			//加载默认参数
	//定时器5初始化,用于精确时间节点计算
	clockClass.Init();	    
    //虚拟串口初始化
	usbVCP_Init(USB_USART_PreemptionPriority,USB_USART_SubPriority);        

    //监控机初始化
	//supervisorInit();	

	OdriveInit();
	//无线数传初始化
	//wirelessInit();
	//控制初始化
	controlInit();
	
	//运动控制初始化
	balanceInit();
    //虚拟串口初始化完成标志
	usbVCP_Printf("All Init Successfully \r\n");             
	/*- 在此处结束 ------------------*/
  vTaskDelete(NULL);								//删除当前任务
}




















