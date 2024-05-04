#include "Driver_RMMotor.h"

#define CHASSIS_EC60

/*******************************�ⲿʹ�õı���*********************************/
motorCanDataRecv_t	 fricWheelData[2];											//Ӣ��Ħ����
motorCanDataRecv_t	 wheelData[4];													//���̵��
motorCanDataRecv_t	 pokeData;															//����
motorCanDataRecv_t	 lidData;																//����
motorCanDataRecv_t	 turntableData;  											  //Ӣ�۲���ת��
motorCanDataRecv_t	 bigPokeData;     										  //�����������
rmmotorCanDataRecv_t pitchMotorData,yawMotorData; 
currentCanDataRecv_t currentDate;
motorMeasureData_t   motorViewPitData,motorViewYawData;
motorMeasureData_t   motorGM3510PitData;

/****************************���������͵������*********************************/
motorCanDataRecv_t	 pawMotorData[2];												//����צ��
motorCanDataRecv_t	 deformMotorData[2];										//���̺�����λ���
motorCanDataRecv_t	 liftMotorData[2];											//����̧������
rmmotorTarData_t 	 pawMotorTarData[2];
/*****************************************************************************/                 

/*
***************************************************
��������driver_gimbal_readdata
���ܣ���ȡ��̨�������
��ڲ�����	can_rx_msg��CAN��������ָ�����
					gimbal_data�������̨������յ����ݵ�ָ�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void gimbal_readdata(CanRxMsg *can_rx_msg,gimbalCanDataRecv_t *gimbal_data){
	gimbal_data->encoderAngle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
	gimbal_data->realcurrent 	= (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
	gimbal_data->current = (int16_t)(can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
}
/*
***************************************************
��������chassis_motor_readdata
���ܣ���ȡ���̵������
��ڲ�����	can_rx_msg��CAN��������ָ�����
					wheelData����ŵ��̵�����յ����ݵ�ָ�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/ 
void rmmotor_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *motorData){
	motorData->rawangle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
	motorData->speed 	 = (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
	motorData->currunt  = (( int16_t)can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
	motorData->temperature = ( int8_t)can_rx_msg->Data[6];
}

void gimbal_motor6020_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *motor6020Data){
	motor6020Data->rawangle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
	motor6020Data->speed 	 = (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
	motor6020Data->currunt  = (( int16_t)can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
	motor6020Data->temperature = ( int8_t)can_rx_msg->Data[6];
}

void rmmotor_senddata(CAN_TypeDef *CANx, u32 ID_CAN, motorSerialNumber_t *rmmotorCanData){
	u8 mbox;
  u16 i=0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = (uint8_t)(rmmotorCanData->currunt1>>8);
	txMessage.Data[1] = (uint8_t) rmmotorCanData->currunt1;
	txMessage.Data[2] = (uint8_t)(rmmotorCanData->currunt2>>8);
	txMessage.Data[3] = (uint8_t) rmmotorCanData->currunt2;
	txMessage.Data[4] = (uint8_t)(rmmotorCanData->currunt3>>8);
	txMessage.Data[5] = (uint8_t) rmmotorCanData->currunt3;
	txMessage.Data[6] = (uint8_t)(rmmotorCanData->currunt4>>8);
	txMessage.Data[7] = (uint8_t) rmmotorCanData->currunt4;
	
	        
  mbox= CAN_Transmit(CANx, &txMessage);   
	
	//�ȴ����ͽ���
  while(CAN_TransmitStatus(CANx, mbox)==CAN_TxStatus_Failed){
		i++;	
		if(i>=0xFFF)
		break;
	}
}

int16_t getRelativePos(int16_t rawEcd, int16_t centerOffset,rmmotorCanDataRecv_t *gimbalMotor){
  int16_t tmp = 0;

	  if (centerOffset >= 4096){
		if (rawEcd > centerOffset - 4096)
		  tmp = rawEcd - centerOffset;
		else
		  tmp = rawEcd + 8192 - centerOffset;
	  }
	  else{
		if (rawEcd > centerOffset + 4096)
		  tmp = rawEcd - 8192 - centerOffset;
		else
		  tmp = rawEcd - centerOffset;
	  }
  return tmp;
}

/*
***************************************************
��������gimbal_readData
���ܣ���ȡ��̨�������
��ڲ�����	can_rx_msg��CAN��������ָ�����
					gimbal_data�������̨������յ����ݵ�ָ�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void gimbal_readData(CanRxMsg *can_rx_msg,rmmotorCanDataRecv_t *gimbal_data){                                       

		gimbal_data->RMD_L_7015Data.molotovDataSheet.motorEncoder = ((uint16_t)can_rx_msg->Data[7] << 8) | can_rx_msg->Data[6];
		gimbal_data->RMD_L_7015Data.molotovDataSheet.motorSpeed   = ((int16_t)can_rx_msg->Data[5] << 8) | can_rx_msg->Data[4];
		gimbal_data->RMD_L_7015Data.molotovDataSheet.temperature  = (int8_t)can_rx_msg->Data[1];
		gimbal_data->RMD_L_7015Data.molotovDataSheet.iq			  = ((int16_t)can_rx_msg->Data[3] << 8) | can_rx_msg->Data[2];
}

/*
***************************************************
��������gimbal_readSlaveData
���ܣ�����̨������ݽṹ����д������
��ڲ�����	dataNumber : ���ݶ�Ӧ����ţ�����鿴ͷ�ļ���
          writeData��Ҫд�������
					gimbal_data�������̨������յ����ݵ�ָ�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void gimbal_readSlaveData(uint8_t dataNumber,vs16 writeData,rmmotorCanDataRecv_t *gimbal_data){                                       

			switch(dataNumber){
				case CODEBOARD_VALUE:    gimbal_data -> GM6020Data . rawangle = writeData;break;
				case REALTORQUE_CURRENT: gimbal_data -> GM6020Data . currunt = writeData;break;
				case TURN_SPEED:         gimbal_data -> GM6020Data . speed = writeData;break;
				case TEMPER:             gimbal_data -> GM6020Data . temperature = writeData;break;
			}

	}



/*
***************************************************
gimbal_chooseData
���ܣ���̨�������ѡ��
��ڲ�����	dataNumber : ���ݶ�Ӧ����ţ�����鿴ͷ�ļ���
					gimbal_data�������̨������յ����ݵ�ָ�����
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
vs16 gimbal_chooseData(uint8_t dataNumber,rmmotorCanDataRecv_t *gimbal_data){                                       
	vs16 dataChoose;
			switch(dataNumber){
				
//				case CODEBOARD_VALUE:    dataChoose = gimbal_data -> GM6020Data . rawangle;break;
//				case REALTORQUE_CURRENT: dataChoose = gimbal_data -> GM6020Data . currunt;break;
//				case TURN_SPEED:         dataChoose = gimbal_data -> GM6020Data . speed;break;
//				case TEMPER:             dataChoose = gimbal_data -> GM6020Data . temperature;break;				
							
				case CODEBOARD_VALUE: dataChoose = gimbal_data->RMD_L_7015Data.molotovDataSheet.motorEncoder;break;
				case TORQUE_CURRENT: dataChoose = gimbal_data->RMD_L_7015Data.molotovDataSheet.iq;break;
			
			}
    return  dataChoose;
}









