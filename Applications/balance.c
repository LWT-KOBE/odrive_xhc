#include "application.h"
#include "usartx.h"
#include "board.h"
#include "balance.h"
#include "bsp_exit.h"
balanceStruct_t balanceData;
Encoding_Wheel EW;
balance_NFC_t NFC;
balance_target_t Angle_Goal;
balance_target_t Motor_SpeedA_Goal;
balance_target_t Motor_SpeedB_Goal;

//uint8_t temp[8]={0};

CascadePID mypid = {0}; //��������PID�ṹ�����

balanceStruct_t* getbalanceData(){
    return &balanceData;
}

//float pbuf[10];
void balanceGlobalInit(void){
//	uart1_init(115200);
//	uart2_init(9600);
//	uart5_init(115200);
//	uart3_init(115200);
	
	
//	cigan_Init();
//	BSP_GPIO_EXIT_Init(BSP_GPIOA6,EXTI_Trigger_Rising,5,0);
//	Encoder_Init_TIM3();
//	PID_Init(&mypid.inner, 0.125f, 0.0000f, 14.0f, 100, 2.0f); //��ʼ���ڻ�����
//    //PID_Init(&mypid.outer, 3.0f, 0.0f, 0.0f, 0, 200.0f); //��ʼ���⻷����
//	PID_Init(&mypid.outer, 7.0f, 0.0f, 15.0f, 0, 1000.0f); //��ʼ���⻷����
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
			digitalHi(&getbalanceData()->dataInitFlag);
//			PDout(2) = 1;
//			PBout(3) = 0;
		}
	 	
		
		
		//��ȡ����������
		EW.Encoder_pr = Read_Encoder(3);
		//�ۼ�
		EW.mileage -= EW.Encoder_pr;
		//�������
		EW.Current_Mileage = EW.mileage / 4000.0f * 20.0f;
		//�����ٶ�
		EW.speed = EW.Encoder_pr /4000.0f * 20.0f * 2.5f;
		
		//�������ٶ�ָ�����
		Motor_SpeedA_Goal.target = Incremental_PID((OdReceivedData.vel_estimate[1].float_temp * 0.2199f),MBSpeed / 100.0f);
			
			
		if(MBSpeed == 0){
			
			if(fabs(OdReceivedData.vel_estimate[1].float_temp * 0.2199f ) <= 0.1){
				Motor_SpeedA_Goal.target = 0;
			}else if(OdReceivedData.vel_estimate[1].float_temp * 0.2199f  > 0.1){
				Motor_SpeedA_Goal.target = -1.575;
			}else if(OdReceivedData.vel_estimate[1].float_temp * 0.2199f  < -0.1){
				Motor_SpeedA_Goal.target = 1.575;
			}
		}
			
			
		OdriveData.SetVel[0].float_temp = -Motor_SpeedA_Goal.target / (M_PI * 0.07f) * 1.00f;
		OdriveData.SetVel[1].float_temp = Motor_SpeedA_Goal.target / (M_PI * 0.07f) * 1.00f;

			
		digitalIncreasing(&getbalanceData()->loops);        
		
	}
}

// ����һ�������������ĸ��������е����ֵ
float max_of_four(float a, float b, float c, float d) {
    float max = a; // �����һ���������ֵ

    if (b > max) {
        max = b;
    }
    if (c > max) {
        max = c;
    }
    if (d > max) {
        max = d;
    }

    return max; // �������ֵ
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
