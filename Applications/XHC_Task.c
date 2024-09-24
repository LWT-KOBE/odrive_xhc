#include "application.h"
#include "usartx.h"
#include "board.h"
XHC_TaskStruct_t XHCData;



XHC_TaskStruct_t* getXHC_Task(){
    return &XHCData;
}

/* ���ó�ʼ�� */
void XHC_TaskGlobalInit(void){
	IO_Init();
	uart2_init(115200);
	uart3_init(115200);
	DMA_Config();
	time2_init(99,839);
	MyADC_Init();
	EEPROM_GPIO_Init();
	WPS = 1;//24C02д����
	EXTI_Configuration();
//	TrainHeadNum = KeyNumHead; //��ȡ��ͷ��	
	ReadE2promData();
	LedNumDisplay = TrainHeadNum;
	TIM14_PWM_Init(8399,0);	//10K  ����Ƶ��PWMƵ��=84000000/8400=10Khz  
	TM1620_Config();
	TM1620_init();	
//	TIM3_Int_Init(65535, 83); //����HALL�ٶ�
}

void XHC_TaskUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getXHC_Task()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,XHC_Task_NORMAL_PERIOD);
        //��ֹ�ظ���ʼ��
		if(!XHCData.dataInitFlag){	
            //���п���ȫ����ʼ��            
			XHC_TaskGlobalInit();																																							
			digitalHi(&getXHC_Task()->dataInitFlag);
			
		}
//////////////		
		USART_Data_Send_Task();//���ڷ���
//////////////	

///////////////////����ѹ//////////////////////////////////////////////	
	//	if(Timer40msCount>=40)//
		{
		//	Timer40msCount = 0;			
			ADC_Value = MyADC_GetValue();
			garry_ch0[gGetAdcCounter] = ADC_Value[0]; //��Դ��ѹ
			gGetAdcCounter++;
			if(gGetAdcCounter >= 10)
			{
				gGetAdcCounter = 0;
			}		
			GetAdcAverage();//10��ADCƽ��ֵ	
			PowerValueLedShow();	//����
		}
///////////////////����ѹ//////////////////////////////////////////////			
		
		digitalIncreasing(&getXHC_Task()->loops);        

	}
}

void XHCDataInit(void){
	getsupervisorData()->taskEvent[XHC_Task] = xTaskCreate(XHC_TaskUpdateTask,"XHC_Task",XHC_Task_STACK_SIZE,NULL,XHC_Task_PRIORITY,&XHCData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
