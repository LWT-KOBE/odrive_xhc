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
        //��ֹ�ظ���ʼ��
		if(!balanceData.dataInitFlag){	
            //���п���ȫ����ʼ��            
			balanceGlobalInit();
			JY60_Calibration();
			digitalHi(&getbalanceData()->dataInitFlag);
		}
		//��ȡimu������Ϣ
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
	�ڵ������񴴽�����xTaskCreate()ʱ��Ϊ����ָ�������ȼ�����������������Ⱥ���֮��
	����ͨ������xTaskPrioritySet()�������޸���������ȼ���ϵͳ��������û�ж����ȼ���������Ҫ��
	ֻҪ�ڴ��㹻��Ϳ��Դ�������������ò�ͬ�����ȼ�����ͬ��������Ը�����ͬ�����ȼ���
	��FreeRTOSConfig.h��configMAX_PRIORITIES �Ĵ�С����������ȼ�������
	���ȼ��Ǵ�0��ʼ��configMAX_PRIORITIES - 1Ϊֹ�����ȼ�Ϊ0����������ȼ���ߡ�
     ϵͳ�еĵ����������������ȼ���ߵ����������У�����������ӵ����ͬ�����ȼ���
	��ô����������ʹ����������ִ��һ��ʱ��Ƭ�������ʱ��Ƭ����1/����ʱ��Ƶ�ʣ�����ʱ�Ӽ��δ�ʱ�ӡ�
	����ͨ������FreeRTOSConfig.h�е�configTICK_RATE_HZ�����������ж�ʱ�ӣ�
	��configTICK_RATE_HZ��ֵΪ100HZʱ��ÿ100ģʽ����һ������ʱ���жϣ�ϵͳ��ִ����Ӧ���жϺ�����	
	*/
	
	getsupervisorData()->taskEvent[CONTROL_TASK] = xTaskCreate(balanceUpdateTask,"BALANCE",BALANCE_STACK_SIZE,NULL,BALANCE_PRIORITY,&balanceData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
