#include "board.h"

taskInit_t taskInitData;
void appInit(void *Parameters){
	taskInitData.eventGroups = NULL;	//�¼���־������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	taskInitData.eventGroups = xEventGroupCreate();
	/*- �ڴ˴���д���ʼ������ -------*/
	configInit();               			//����Ĭ�ϲ���
	//��ʱ��5��ʼ��,���ھ�ȷʱ��ڵ����
	clockClass.Init();	    
    //���⴮�ڳ�ʼ��
	usbVCP_Init(USB_USART_PreemptionPriority,USB_USART_SubPriority);        

    //��ػ���ʼ��
	//supervisorInit();	

	OdriveInit();
	//����������ʼ��
	//wirelessInit();
	LcdTaskInit();
	controlInit();
	
	
    //���⴮�ڳ�ʼ����ɱ�־
	usbVCP_Printf("All Init Successfully \r\n");             
	/*- �ڴ˴����� ------------------*/
  vTaskDelete(NULL);								//ɾ����ǰ����
}




















