#include "application.h"


canSendStruct_t canSendData;
ODCANSendStruct_t ODSendData;

/****************************���ļ�ʹ�õľ�̬����******************************/
static BSP_CAN_TypeDef can1;
static BSP_CAN_TypeDef can2;
/*****************************************************************************/

void ODRequestedState(void){		
    //�����Ҫ��״̬�������в�����ֱ�ӽ�OdriveData.flashSave����һ�μ���    
	if(OdriveData.RequestedState){                                              
       //��⵽��Ҫ�ı�״̬����		����������ʾ��
		//supervisorData.beepState = MUSIC_PARAMCALI;		        
		//set_axis_requested_state(CAN1,AXIS0_ID,MSG_SET_AXIS_REQUESTED_STATE,1,&OdriveData,axis0,&ODSendData);
		set_axis_requested_state(CAN1,AXIS1_ID,MSG_SET_AXIS_REQUESTED_STATE,1,&OdriveData,axis0,&ODSendData);
		
		//ODSendControllerModes(CAN1,AXIS0_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis0,&ODSendData);
		//ODSendControllerModes(CAN1,AXIS1_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis0,&ODSendData);
		
		
//		vTaskDelay(1000);	
//		digitalLo(&OdriveData.RequestedState);         
        
	} 

}

void ODFlashSave(void){		
    //�����Ҫ��״̬�������в�����ֱ�ӽ�OdriveData.flashSave����һ�μ���    
	if(OdriveData.flashSave){                                              
       //��⵽��Ҫ�ı�״̬����		����������ʾ��
		//supervisorData.beepState = MUSIC_PARAMCALI;	

		//ODSendControllerModes(CAN1,AXIS0_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis0,&ODSendData);		

		
		//�������
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_CLEAR_ERRORS);						

		//config
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_SAVE_CONFIG);						
		
		//digitalLo(&OdriveData.flashSave);         
        
	} 	
}


void canSendGimbalUpdate(void){
 

	//1KHZ
	//����λ������
	ODSendInputPosData(CAN1,AXIS1_ID,MSG_SET_INPUT_POS,8,&OdriveData,axis1,&ODSendData);	
	
	//�����ٶ�����
	//ODSendInputVelData(CAN1,AXIS0_ID,MSG_SET_INPUT_VEL,8,&OdriveData,axis0,&ODSendData);		

	//������������
//	ODSendInputCurData(CAN1,AXIS0_ID,MSG_SET_INPUT_TORQUE,4,&OdriveData,axis0,&ODSendData);	
//	ODSendInputCurData(CAN1,AXIS1_ID,MSG_SET_INPUT_TORQUE,4,&OdriveData,axis0,&ODSendData);	
	
	
	//��ȡ�����ź�
	OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_ODRIVE_HEARTBEAT);
	OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_ODRIVE_HEARTBEAT);	
	
	
	//250Hz  ��ѭ���͡�
	if(!((canSendData.loops + 1) % 2)){																										
			
	
		
		//��ȡ���ߵ�ѹ������
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_VBUS_VOLTAGE);						
		//��ȡ�¶�
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_TEMP);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_GET_TEMP);

		//��ȡ�������ֵ
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_MOTOR_ERROR);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_GET_MOTOR_ERROR);

		//��ȡ����������ֵ
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_ENCODER_ERROR);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_GET_ENCODER_ERROR);

		
		//��ȡ������CPRֵ
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_ENCODER_COUNT);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_GET_ENCODER_COUNT);
		
		//��ȡλ��/�ٶ�ֵ
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_ENCODER_ESTIMATES);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_GET_ENCODER_ESTIMATES);

		
    }
	
	//125Hz  ��ѭ���͡�	
	if(!((canSendData.loops + 1) % 4)){		

		//���õ��״̬
		ODRequestedState();
		
		//flash
		ODFlashSave();	


	}		

	
	

}


void canSendUpdate(void){
	canSendGimbalUpdate();

	digitalIncreasing(&canSendData.loops);
}

void canSendInit(void){								//CAN���ͳ�ʼ��
  
    driver_can1_init(CAN1,BSP_GPIOD0,BSP_GPIOD1,4,0);
	
	
    driver_can2_init(CAN2,BSP_GPIOB5,BSP_GPIOB6,3,0);    
}



void canSendRelax(void){

	memset((void *)&canSendData.can1_0x141, 0, sizeof(canSendData.can1_0x141));
	memset((void *)&canSendData.can1_0x142, 0, sizeof(canSendData.can1_0x142));
}


/*
***************************************************
��������driver_can1_init
���ܣ�RM���CAN1��ʼ��
��ڲ�����	rm_canx��ʹ�õ�CANͨ��
					rm_canx_rx��RM CAN��������
					rm_canx_tx��RM CAN��������
					Preemption��CAN�ж���ռ���ȼ�
					Sub��CAN�жϴ����ȼ�
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void driver_can1_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub){
	can1.CANx = rm_canx;
	can1.CANx_RX = rm_canx_rx;
	can1.CANx_TX = rm_canx_tx;
	if(rm_canx == CAN1){
		can1.CAN_FilterInitStructure = CAN1_FilterInitStructure;
	}
	else if(rm_canx == CAN2){
		can1.CAN_FilterInitStructure = CAN2_FilterInitStructure;
	}
	BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal,Preemption,Sub);
	
	//BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal,Preemption,Sub);
	
} 
/*
***************************************************
��������driver_can1_init
���ܣ�RM���CAN2��ʼ��
��ڲ�����	rm_canx��ʹ�õ�CANͨ��
					rm_canx_rx��RM CAN��������
					rm_canx_tx��RM CAN��������
					Preemption��CAN�ж���ռ���ȼ�
					Sub��CAN�жϴ����ȼ�
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void driver_can2_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub){
	can2.CANx = rm_canx;
	can2.CANx_RX = rm_canx_rx;
	can2.CANx_TX = rm_canx_tx;
	if(rm_canx == CAN1){
		can2.CAN_FilterInitStructure = CAN1_FilterInitStructure;
	}
	else if(rm_canx == CAN2){
		can2.CAN_FilterInitStructure = CAN2_FilterInitStructure;
	}
	BSP_CAN_Mode_Init(&can2,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal,Preemption,Sub);
	
	//BSP_CAN_Mode_Init(&can2,CAN_SJW_4tq,CAN_BS2_4tq,CAN_BS1_16tq,4,CAN_Mode_Normal,Preemption,Sub);
	
	
}
