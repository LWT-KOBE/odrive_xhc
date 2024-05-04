#include "supervisor.h"
#include "driver.h"
#include "stdbool.h"

supervisorStruct_t supervisorData;

supervisorStruct_t* getsupervisorData(){
    return &supervisorData;
}



/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�				   //
//			���ض�����������ɿ��Ը����⣬		  		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

void supervisorStateSwitch(uint16_t state,uint8_t valve){							//�����л����״̬��״̬��																												
	if (valve)
		supervisorData.state |= state;
	else
		supervisorData.state &= ~state;
}

void supervisorImuCali(void){		
	
//	//IMUУ׼ָ��
//	supervisorStateSwitch(STATE_IMUCALI,ENABLE);		
//	
//	//�Ƿ������ٶȼ�У׼	
//	digitalHi(&imuSensorData.accTare);

}

void supervisrCheckModule(void){
//	//����Ƿ���IMUУ׼״̬
//	if(supervisorData.state & STATE_IMUCALI)
//		supervisorData.modularState |= MODULAR_IMU_CALI;
//	else
//		supervisorData.modularState &= ~MODULAR_IMU_CALI;
//	//���IMU�Ƿ�ʧ
//	if(supervisorData.state & STATE_IMU_LOSS)
//		supervisorData.modularState |= MODULAR_IMU_LOSS;
//	else
//		supervisorData.modularState &= ~MODULAR_IMU_LOSS;

}


//���Odrive�Ƿ������ݷ���
void Odrive_CheckDevice(void){
	
	OdriveData.OdError.intervalNum = OdriveData.OdError.errorCount - OdriveData.OdError.lastErrorCount;
	OdriveData.OdError.lastErrorCount = OdriveData.OdError.errorCount;
	if(!OdriveData.OdError.intervalNum){
		supervisorStateSwitch(STATE_ODRIVE_ERROR,ENABLE);							
	}
	else{
		supervisorStateSwitch(STATE_ODRIVE_ERROR,DISABLE);           
	}
}






void supervisorLedSwitch(void){
  if(supervisorData.state & STATE_IMU_LOSS){//�̵ƿ���
		supervisorData.rgbState = LED_IMU_CALI_FALSE;									//imu��������������
	}
	else if(supervisorData.state & STATE_IMUCALI){
		supervisorData.rgbState = LED_IMU_HARDWORK_FALSE;											//imuУ׼������
	}
	else if(supervisorData.state & STATE_ODRIVE_ERROR){
		supervisorData.rgbState = LED_ODRIVE_FALSE;										//��Odriveͨ�Ŵ���
	}
	else{
		supervisorData.rgbState = LED_STATE_NORMAL;															//��������
	}
}





//Flash����
void supervisorFlash(void){		
    //�����Ҫ��flash���в�����ֱ�ӽ�supervisorData.flashSave����һ�μ���    
	if(supervisorData.flashSave){                                              
       //��⵽��Ҫ����Flash		����������ʾ��
		supervisorData.beepState = MUSIC_PARAMCALI;		        
		configFlashWrite();                                             	//д��flash
		digitalLo(&supervisorData.flashSave);         
        
	} 	
}

bool testImuCali = false,testAccCali = false;
void supervisorUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){
		vTaskDelayUntil(&xLastWakeTime,SUPER_STACK_PERIOD);

        
		//RGB����
		supervisorLedSwitch();
		//����Ƿ���Ҫ����flash        
		supervisorFlash();
        //����������
        //LED����		���ذ���3ɫLED����ָʾ״̬										
        appSightClass.led(supervisorData.rgbState);                           
        //��������������
        //appSightClass.beep(supervisorData.beepState);	      
		Odrive_CheckDevice();		
        //״̬����
        digitalClan(&supervisorData.beepState);	        
		digitalIncreasing(&supervisorData.loops);
	}
}

void supervisorInit(void){
    
	//��������ʼ������LEDһ�����ڼ��״̬��
	sightClass.Init();																												
	supervisorData.taskEvent[SUPERVISOR_TASK] = xTaskCreate(supervisorUpdateTask,"SUPE",SPUER_STACK_SIZE,NULL,SPUER_PRIORITY,&supervisorData.xHandleTask);
}
