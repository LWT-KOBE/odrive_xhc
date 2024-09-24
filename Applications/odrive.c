#include "board.h"

OdriveStruct_t OdriveData; 
ODCanDataRecv_t OdReceivedData;
uint8_t Flag = 0;
float buf[10];
////////////////////////////ODRIVE�����壨M0����CAN����(STM32F407VET6) �����ʣ�1000K  ʹ��120���ն˵���/////////////////////////
/*********************************����Ϊ���������*****************************************************************************/

/*
***************************************************
��������OdriveSendData
���ܣ��������CAN����_����֡
��ڲ�����	CANx��CAN1 orCAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive�ĸ�������  
			len������֡���� 
			CanSendData��CAN���͵����ݽṹ
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע�� �ú������͵�������֡���м�
***************************************************
*/

void OdriveSendData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,ODCANSendStruct_t* CanSendData) {
	CanTxMsg *txMessage;
	uint8_t mbox;
	uint8_t count;
	uint16_t i = 0;
	txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
	
	//CAN ID ��ǰ��λ����ID����odrive������Ϊ0x001��������λ�ǿ���������� MSG_GET_ENCODER_ERROR��	
	txMessage->StdId = (ID_CAN<<5)+CMD_CAN;	
	txMessage->IDE = CAN_Id_Standard;
	txMessage->RTR = CAN_RTR_Data;
	txMessage->DLC = len;
	for (count = 0; count < len; count++) {
		txMessage->Data[count] = (uint8_t)CanSendData->data[count];
	}
	mbox = CAN_Transmit(CANx, txMessage);
	while (CAN_TransmitStatus(CANx,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
	aqFree(txMessage,8,sizeof(CanTxMsg));
}


/*
***************************************************
��������OdriveSend_RemoteCmd
���ܣ��������CAN����_Զ��֡֡
��ڲ�����	CANx��CAN1 orCAN2
			CMD_CAN��odrive�ĸ�������  
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע�� �ú������͵���Զ��֡���м�
***************************************************
*/
void OdriveSend_RemoteCmd(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN) {
	CanTxMsg *txMessage;
	uint8_t mbox;
	uint16_t i = 0;
	txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
	
	//CAN ID ��ǰ��λ����ID����odrive������Ϊ0x001��������λ�ǿ���������� MSG_GET_ENCODER_ERROR��
	txMessage->StdId = (ID_CAN<<5)+CMD_CAN;

	//txMessage->ExtId=0x12; 	 // ������չ��ʾ����29λ�� 
	txMessage->ExtId=0x12; 	 // ������չ��ʾ����29λ��
	
	txMessage->IDE = CAN_Id_Standard;
	//txMessage->RTR = CAN_RTR_Remote;
	txMessage->RTR = CAN_RTR_Data;
	mbox = CAN_Transmit(CANx, txMessage);
	while (CAN_TransmitStatus(CANx,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
	aqFree(txMessage,8,sizeof(CanTxMsg));
}


/*
***************************************************
��������ODSendInputPosData
���ܣ����λ�ñջ�����_����֡
��ڲ�����	CANx��CAN1 or CAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive��λ�ñջ����MSG_SET_INPUT_POS ��
			len������֡���� 
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����͵�λ�ÿ��ƽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	 
***************************************************
*/
void ODSendInputPosData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData) {

	CanSendData->data[0] = Spetsnaz->SetPos[axis].u8_temp[0];
	CanSendData->data[1] = Spetsnaz->SetPos[axis].u8_temp[1];
	CanSendData->data[2] = Spetsnaz->SetPos[axis].u8_temp[2];
	CanSendData->data[3] = Spetsnaz->SetPos[axis].u8_temp[3];	

	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
	
	
}



/*
***************************************************
��������ODSendPos_gainData
���ܣ����λ����������_����֡
��ڲ�����	CANx��CAN1 or CAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive��λ���������MSG_SET_INPUT_POS ��
			len������֡���� 
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����͵�λ��������ƽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	 
***************************************************
*/
void ODSendPos_gainData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData) {

	CanSendData->data[0] = Spetsnaz->Pos_gain[axis].u8_temp[0];
	CanSendData->data[1] = Spetsnaz->Pos_gain[axis].u8_temp[1];
	CanSendData->data[2] = Spetsnaz->Pos_gain[axis].u8_temp[2];
	CanSendData->data[3] = Spetsnaz->Pos_gain[axis].u8_temp[3];	

	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
}


/*
***************************************************
��������ODSendInputVelData
���ܣ�����ٶȱջ�����_����֡
��ڲ�����	CANx��CAN1 orCAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive���ٶȱջ����MSG_SET_INPUT_VEL ��
			len������֡���� 
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����͵��ٶȿ��ƽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	 
***************************************************
*/
void ODSendInputVelData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData) {

	CanSendData->data[0] = Spetsnaz->SetVel[axis].u8_temp[0];
	CanSendData->data[1] = Spetsnaz->SetVel[axis].u8_temp[1];
	CanSendData->data[2] = Spetsnaz->SetVel[axis].u8_temp[2];
	CanSendData->data[3] = Spetsnaz->SetVel[axis].u8_temp[3];
	

	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
	
	
}

/*
***************************************************
��������ODSendVel_gainData
���ܣ�����ٶȻ���������_����֡
��ڲ�����	CANx��CAN1 or CAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive���ٶȻ��������MSG_SET_VEL_GAIN ��
			len������֡���� 
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����͵�λ��������ƽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	 
***************************************************
*/
void ODSendVel_gainsData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData){

	CanSendData->data[0] = Spetsnaz->Vel_gain[axis].u8_temp[0];
	CanSendData->data[1] = Spetsnaz->Vel_gain[axis].u8_temp[1];
	CanSendData->data[2] = Spetsnaz->Vel_gain[axis].u8_temp[2];
	CanSendData->data[3] = Spetsnaz->Vel_gain[axis].u8_temp[3];

	CanSendData->data[4] = Spetsnaz->Vel_integrator_gain[axis].u8_temp[0];
	CanSendData->data[5] = Spetsnaz->Vel_integrator_gain[axis].u8_temp[1];
	CanSendData->data[6] = Spetsnaz->Vel_integrator_gain[axis].u8_temp[2];
	CanSendData->data[7] = Spetsnaz->Vel_integrator_gain[axis].u8_temp[3];

	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
}


/*
***************************************************
��������ODSendInputCurData
���ܣ�������رջ�����_����֡
��ڲ�����	CANx��CAN1 orCAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive�����رջ����MSG_SET_INPUT_TORQUE ��
			len������֡���� 
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����͵����ؿ��ƽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	 
***************************************************
*/
void ODSendInputCurData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData) {
	
	
	CanSendData->data[0] = Spetsnaz->SetCur[axis].u8_temp[0];
	CanSendData->data[1] = Spetsnaz->SetCur[axis].u8_temp[1];
	CanSendData->data[2] = Spetsnaz->SetCur[axis].u8_temp[2];
	CanSendData->data[3] = Spetsnaz->SetCur[axis].u8_temp[3];

	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
	
	
}


/*
***************************************************
��������ODSET_CONTROL_MODE
���ܣ����״̬��������_����֡
��ڲ�����	CANx��CAN1 orCAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive��״̬�����������MSG_SET_CONTROLLER_MODES ��
			len������֡����
			CONTROL_MODE���������ģʽ
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����͵ĵ�������ṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע�� uint8_t len�����ݷ�Χ
	 
    enum {
        CONTROL_MODE_CURRENT		 = 0,//��������ģʽ����ֱ�ӿ���
        CONTROL_MODE_CURRENT_RAMP	 = 1,//��������ģʽ��������
        CONTROL_MODE_VELOCITY		 = 2,//�ٶȿ���ģʽ����ֱ�ӿ���
        CONTROL_MODE_VELOCITY_RAMP	 = 3,//�ٶȿ���ģʽ��������
        CONTROL_MODE_POSITION		 = 4,//λ�ÿ���ģʽ����ֱ�ӿ���
        CONTROL_MODE_POSITION_TRAP	 = 5,//λ�ÿ���ģʽ��������
		CONTROL_MODE_POSITION_FILTER = 6,//λ���˲�����ģʽ
    }; 

***************************************************
*/
void ODSET_CONTROL_MODE(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData) {
	
	uint8_t CONTROL_MODE = 5;
	
	CONTROL_MODE = Spetsnaz->ControlMode[axis];
	switch (CONTROL_MODE){
		
		//ֱ������ģʽ
		case CONTROL_MODE_CURRENT:
			CanSendData->data[0] = 0x01;
			CanSendData->data[1] = 0;
			CanSendData->data[2] = 0;
			CanSendData->data[3] = 0;
	
			CanSendData->data[4] = 0x01;
			CanSendData->data[5] = 0;
			CanSendData->data[6] = 0;
			CanSendData->data[7] = 0;
		break;
		
		//��������ģʽ
		case CONTROL_MODE_CURRENT_RAMP:
			CanSendData->data[0] = 0x01;
			CanSendData->data[1] = 0;
			CanSendData->data[2] = 0;
			CanSendData->data[3] = 0;
	
			CanSendData->data[4] = 0x06;
			CanSendData->data[5] = 0;
			CanSendData->data[6] = 0;
			CanSendData->data[7] = 0;
		break;
		
		//ֱ���ٶ�ģʽ
		case CONTROL_MODE_VELOCITY:
			CanSendData->data[0] = 0x02;
			CanSendData->data[1] = 0;
			CanSendData->data[2] = 0;
			CanSendData->data[3] = 0;
	
			CanSendData->data[4] = 0x01;
			CanSendData->data[5] = 0;
			CanSendData->data[6] = 0;
			CanSendData->data[7] = 0;
		break;
		
		//�ٶ�����ģʽ
		case CONTROL_MODE_VELOCITY_RAMP:
			CanSendData->data[0] = 0x02;
			CanSendData->data[1] = 0;
			CanSendData->data[2] = 0;
			CanSendData->data[3] = 0;
	
			CanSendData->data[4] = 0x02;
			CanSendData->data[5] = 0;
			CanSendData->data[6] = 0;
			CanSendData->data[7] = 0;
		break;
		
		//ֱ��λ��ģʽ
		case CONTROL_MODE_POSITION:
			CanSendData->data[0] = 0x03;
			CanSendData->data[1] = 0;
			CanSendData->data[2] = 0;
			CanSendData->data[3] = 0;
	
			CanSendData->data[4] = 0x01;
			CanSendData->data[5] = 0;
			CanSendData->data[6] = 0;
			CanSendData->data[7] = 0;
		break;
		
		//����λ��ģʽ
		case CONTROL_MODE_POSITION_TRAP:
			CanSendData->data[0] = 0x03;
			CanSendData->data[1] = 0;
			CanSendData->data[2] = 0;
			CanSendData->data[3] = 0;
	
			CanSendData->data[4] = 0x05;
			CanSendData->data[5] = 0;
			CanSendData->data[6] = 0;
			CanSendData->data[7] = 0;
		break;
		
		//λ���˲�����ģʽ
		case CONTROL_MODE_POSITION_FILTER:
			CanSendData->data[0] = 0x03;
			CanSendData->data[1] = 0;
			CanSendData->data[2] = 0;
			CanSendData->data[3] = 0;
	
			CanSendData->data[4] = 0x03;
			CanSendData->data[5] = 0;
			CanSendData->data[6] = 0;
			CanSendData->data[7] = 0;
		break;
	}


	
	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
	
	
}



/*
***************************************************
��������ODReadHeartBeatData
���ܣ���ȡ�����ź�_����֡
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����������źŽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	
	�����źŵķ���Ƶ��Ϊ100HZ��ODRIVE�ˣ�,�������źŷ��ص�����0-1���͵��ߣ���ODRIVE�ĵ����������100��ʾ�����ڷ���ǰ������100����������0.01f
	2-3λ��ODRIVE�ĵ��λ�ã���100��ʾλ���ڷ���ǰ������100����������0.01f
	4-5λ��ODRIVE�ĵ���ٶȣ���1000��ʾ�ٶ��ڷ���ǰ������1000����������0.01f
	��6λ���������Ϣ��axes[0].error_����U8�ͣ�8���ֽ�
	
	enum {
			ERROR_NONE		= 1,
			ERROR_INVALID_STATE	= 2,
			ERROR_WATCHDOG_TIMER_EXPIRED		= 3,
			ERROR_MIN_ENDSTOP_PRESSED	= 4,
			ERROR_MAX_ENDSTOP_PRESSED		= 5,
			ERROR_ESTOP_REQUESTED	= 6,
			ERROR_HOMING_WITHOUT_ENDSTOP = 7,
			ERROR_OVER_TEMP = 8,
			ERROR_UNKNOWN_POSITION = 9,
		
		};


	��7λ�ǿ���״̬��U8�ͣ�8���ֽ�
	
        enum AxisState {
            AXIS_STATE_UNDEFINED             = 0,
            AXIS_STATE_IDLE                  = 1,
            AXIS_STATE_STARTUP_SEQUENCE      = 2,
            AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
            AXIS_STATE_MOTOR_CALIBRATION     = 4,
            AXIS_STATE_ENCODER_INDEX_SEARCH  = 6,
            AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
            AXIS_STATE_CLOSED_LOOP_CONTROL   = 8,
            AXIS_STATE_LOCKIN_SPIN           = 9,
            AXIS_STATE_ENCODER_DIR_FIND      = 10,
            AXIS_STATE_HOMING                = 11,
            AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
            AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
        };

***************************************************
*/

void ODReadHeartBeatData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
//	Spetsnaz->heartbeat_Cur[axis] = (float)(s16)((CanRevData->Data[1]<<8)|CanRevData->Data[0])/100.0f;
//	Spetsnaz->heartbeat_Pos[axis] = (float)(s16)((CanRevData->Data[3]<<8)|CanRevData->Data[2])/100.0f;
//	Spetsnaz->heartbeat_Vel[axis] = (float)(s16)((CanRevData->Data[5]<<8)|CanRevData->Data[4])/100.0f;	
//	Spetsnaz->heartbeat_AxisError[axis] = CanRevData->Data[6];
//	Spetsnaz->heartbeat_Current_State[axis] = CanRevData->Data[7];
	
	Spetsnaz->heartbeat_AxisError[axis].u8_temp[0] = CanRevData->Data[0]; 
	Spetsnaz->heartbeat_AxisError[axis].u8_temp[1] = CanRevData->Data[1]; 
	Spetsnaz->heartbeat_AxisError[axis].u8_temp[2] = CanRevData->Data[2]; 
	Spetsnaz->heartbeat_AxisError[axis].u8_temp[3] = CanRevData->Data[3]; 
	
	Spetsnaz->heartbeat_current_state_[axis] = CanRevData->Data[4];
	Spetsnaz->heartbeat_motorFlags[axis] = CanRevData->Data[5];
	Spetsnaz->heartbeat_encoderFlags[axis] = CanRevData->Data[6];
	Spetsnaz->heartbeat_controllerFlags[axis] = CanRevData->Data[7];
	

}

/*
***************************************************
��������ODReadVbusData
���ܣ���ȡĸ�ߵ�ѹ_����֡
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz������ĸ�ߵ�ѹ��ĸ�ߵ��������ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	
	���ص�����ǰ��λ���͵��ߣ���ODRIVE��ĸ�ߵ�ѹ���ݣ�����λ���ص���ODRIVE��ĸ�ߵ������ݣ�����FLOAT��
	�����õ���һ���������������ת���������뿴��
	//����������ת������
	typedef union{
		u8 		u8_temp[4];
		float float_temp;
		s32 	s32_temp;
		u32		u32_temp;
	} formatTrans32Struct_t;
***************************************************
*/
void ODReadVbusData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	
	Spetsnaz->vbus_voltage[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->vbus_voltage[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->vbus_voltage[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->vbus_voltage[axis].u8_temp[3] = CanRevData->Data[3];
	
	Spetsnaz->ibus[axis].u8_temp[0] = CanRevData->Data[4];
	Spetsnaz->ibus[axis].u8_temp[1] = CanRevData->Data[5];
	Spetsnaz->ibus[axis].u8_temp[2] = CanRevData->Data[6];
	Spetsnaz->ibus[axis].u8_temp[3] = CanRevData->Data[7];
}

/*
***************************************************
��������ODReadTempData
���ܣ���ȡ������¶�_����֡
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����յ���¶ȵ����ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	
	���ص�����ǰ��λ���͵��ߣ���ODRIVE�ĵ���¶����ݣ���FLOAT��
	�����õ���һ���������������ת���������뿴��
	//����������ת������
	typedef union{
		u8 		u8_temp[4];
		float float_temp;
		s32 	s32_temp;
		u32		u32_temp;
	} formatTrans32Struct_t;
***************************************************
*/void ODReadTempData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
	
	//�����¶�
	Spetsnaz->temperature[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->temperature[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->temperature[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->temperature[axis].u8_temp[3] = CanRevData->Data[3];
	
	//����¶�
	Spetsnaz->motor_temperature[axis].u8_temp[0] = CanRevData->Data[4];
	Spetsnaz->motor_temperature[axis].u8_temp[1] = CanRevData->Data[5];
	Spetsnaz->motor_temperature[axis].u8_temp[2] = CanRevData->Data[6];
	Spetsnaz->motor_temperature[axis].u8_temp[3] = CanRevData->Data[7];
}

/*
***************************************************
��������ODReadMotorErrorData
���ܣ���ȡ���������Ϣ_����֡
��ڲ�����	
			CMD_CAN��odrive�ĵ��������Ϣ��ȡ���MSG_GET_MOTOR_ERROR ��
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz����ȡ���������Ϣ�ṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��

	OdReceivedData.MotorError.u32_temp ��������Ӧֵ

        enum Error {
            ERROR_NONE                       = 0x00000000,
            ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001,
            ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002,
            ERROR_DRV_FAULT                  = 0x00000008,
            ERROR_CONTROL_DEADLINE_MISSED    = 0x00000010,
            ERROR_MODULATION_MAGNITUDE       = 0x00000080,
            ERROR_CURRENT_SENSE_SATURATION   = 0x00000400,
            ERROR_CURRENT_LIMIT_VIOLATION    = 0x00001000,
            ERROR_MODULATION_IS_NAN          = 0x00010000,
            ERROR_MOTOR_THERMISTOR_OVER_TEMP = 0x00020000,
            ERROR_FET_THERMISTOR_OVER_TEMP   = 0x00040000,
            ERROR_TIMER_UPDATE_MISSED        = 0x00080000,
            ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = 0x00100000,
            ERROR_CONTROLLER_FAILED          = 0x00200000,
            ERROR_I_BUS_OUT_OF_RANGE         = 0x00400000,
            ERROR_BRAKE_RESISTOR_DISARMED    = 0x00800000,
            ERROR_SYSTEM_LEVEL               = 0x01000000,
            ERROR_BAD_TIMING                 = 0x02000000,
            ERROR_UNKNOWN_PHASE_ESTIMATE     = 0x04000000,
            ERROR_UNKNOWN_PHASE_VEL          = 0x08000000,
            ERROR_UNKNOWN_TORQUE             = 0x10000000,
            ERROR_UNKNOWN_CURRENT_COMMAND    = 0x20000000,
            ERROR_UNKNOWN_CURRENT_MEASUREMENT = 0x40000000,
            ERROR_UNKNOWN_VBUS_VOLTAGE       = 0x80000000,
            ERROR_UNKNOWN_VOLTAGE_COMMAND    = 0x100000000,
            ERROR_UNKNOWN_GAINS              = 0x200000000,
            ERROR_CONTROLLER_INITIALIZING    = 0x400000000,
            ERROR_UNBALANCED_PHASES          = 0x800000000,
        };

***************************************************
*/

void ODReadMotorErrorData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	Spetsnaz->MotorError[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->MotorError[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->MotorError[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->MotorError[axis].u8_temp[3] = CanRevData->Data[3];
	
}



/*
***************************************************
��������ODReadMotorErrorData
���ܣ���ȡ������������Ϣ_����֡
��ڲ�����	
			CMD_CAN��odrive�ı�����������Ϣ��ȡ���MSG_GET_ENCODER_ERROR ��
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz����ȡ������������Ϣ�ṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��

	OdReceivedData.EncoderError.u32_temp �����������Ӧֵ



        enum Error {
            ERROR_NONE                       = 0x00000000,
            ERROR_UNSTABLE_GAIN              = 0x00000001,
            ERROR_CPR_POLEPAIRS_MISMATCH     = 0x00000002,
            ERROR_NO_RESPONSE                = 0x00000004,
            ERROR_UNSUPPORTED_ENCODER_MODE   = 0x00000008,
            ERROR_ILLEGAL_HALL_STATE         = 0x00000010,
            ERROR_INDEX_NOT_FOUND_YET        = 0x00000020,
            ERROR_ABS_SPI_TIMEOUT            = 0x00000040,
            ERROR_ABS_SPI_COM_FAIL           = 0x00000080,
            ERROR_ABS_SPI_NOT_READY          = 0x00000100,
            ERROR_HALL_NOT_CALIBRATED_YET    = 0x00000200,
        };


***************************************************
*/
void ODReadEncodeErrorData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	Spetsnaz->EncoderError[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->EncoderError[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->EncoderError[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->EncoderError[axis].u8_temp[3] = CanRevData->Data[3];
	
}


/*
***************************************************
��������ODReadEncoderCountData
���ܣ���ȡ��������SHADOW��CPR_����֡
��ڲ�����	
			CMD_CAN��odrive�ı�����������Ϣ��ȡ���MSG_GET_ENCODER_COUNT ��
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz����ȡ������CPR���ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��

***************************************************
*/

void ODReadEncoderCountData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	Spetsnaz->shadow_count[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->shadow_count[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->shadow_count[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->shadow_count[axis].u8_temp[3] = CanRevData->Data[3];
	
	
	Spetsnaz->count_in_cpr[axis].u8_temp[0] = CanRevData->Data[4];	
	Spetsnaz->count_in_cpr[axis].u8_temp[1] = CanRevData->Data[5];	
	Spetsnaz->count_in_cpr[axis].u8_temp[2] = CanRevData->Data[6];	
	Spetsnaz->count_in_cpr[axis].u8_temp[3] = CanRevData->Data[7];	

}

/*
***************************************************
��������ODReadEncodeEstimatesData
���ܣ���ȡ���λ�ú��ٶ�_����֡
��ڲ�����	
			CMD_CAN��odrive�ı�����������Ϣ��ȡ���MSG_GET_ENCODER_ESTIMATES ��
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz����ȡ���λ�ú��ٶ����ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��

***************************************************
*/
void ODReadEncodeEstimatesData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	Spetsnaz->pos_estimate[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->pos_estimate[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->pos_estimate[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->pos_estimate[axis].u8_temp[3] = CanRevData->Data[3];
	
	
	Spetsnaz->vel_estimate[axis].u8_temp[0] = CanRevData->Data[4];	
	Spetsnaz->vel_estimate[axis].u8_temp[1] = CanRevData->Data[5];	
	Spetsnaz->vel_estimate[axis].u8_temp[2] = CanRevData->Data[6];	
	Spetsnaz->vel_estimate[axis].u8_temp[3] = CanRevData->Data[7];	

}

//��ȡ����������͵������Ϳ���ģʽ
void ODReadMotorPolePairs(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	//CAN�źŷ��͵ĵ�������Ϣ��ǿ�ƽ�S16תFloat�����ҳ�10000��(��Ϊ���͵�ʱ��������10000��)	
	Spetsnaz->motor_phase_resistance[axis] = (float)(s16)((CanRevData->Data[1]<<8)|CanRevData->Data[0])/10000.0f;
	
	//CAN�źŷ��͵ĵ�������Ϣ��ǿ�ƽ�S16תFloat����������1000000��(��Ϊ���ֵʵ����̫С��)	
	Spetsnaz->motor_phase_inductance[axis] = (float)(s16)((CanRevData->Data[3]<<8)|CanRevData->Data[2]);

	
	//��ȡ����ļ�����
	Spetsnaz->motor_pole_pairs[axis] = CanRevData->Data[4];	
	//��ȡ����Ŀ���ģʽ
	//Spetsnaz->control_mode[axis] = CanRevData->Data[5];	


}

//��ȡ�������ƺ��ٶ�����
void ODReadLimitData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	Spetsnaz->vel_limit[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->vel_limit[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->vel_limit[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->vel_limit[axis].u8_temp[3] = CanRevData->Data[3];
	
	
	Spetsnaz->current_limit[axis].u8_temp[0] = CanRevData->Data[4];	
	Spetsnaz->current_limit[axis].u8_temp[1] = CanRevData->Data[5];	
	Spetsnaz->current_limit[axis].u8_temp[2] = CanRevData->Data[6];	
	Spetsnaz->current_limit[axis].u8_temp[3] = CanRevData->Data[7];	


}

//���͵������ƺ��ٶ�����
void ODSendLimitData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData) {

	CanSendData->data[0] = Spetsnaz->vel_limit[axis].u8_temp[0];
	CanSendData->data[1] = Spetsnaz->vel_limit[axis].u8_temp[1];
	CanSendData->data[2] = Spetsnaz->vel_limit[axis].u8_temp[2];
	CanSendData->data[3] = Spetsnaz->vel_limit[axis].u8_temp[3];

	CanSendData->data[4] = Spetsnaz->current_limit[axis].u8_temp[0];
	CanSendData->data[5] = Spetsnaz->current_limit[axis].u8_temp[1];
	CanSendData->data[6] = Spetsnaz->current_limit[axis].u8_temp[2];
	CanSendData->data[7] = Spetsnaz->current_limit[axis].u8_temp[3];	
		
	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
	
	
}


/*
***************************************************
��������ODReadPos_gainData
���ܣ���ȡ����λ�û�����
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz������ĸ�ߵ�ѹ��ĸ�ߵ��������ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	
	���ص�����ǰ��λ���͵��ߣ���ODRIVE��ĸ�ߵ�ѹ���ݣ�����λ���ص���ODRIVE��ĸ�ߵ������ݣ�����FLOAT��
	�����õ���һ���������������ת���������뿴��
	//����������ת������
	typedef union{
		u8 		u8_temp[4];
		float float_temp;
		s32 	s32_temp;
		u32		u32_temp;
	} formatTrans32Struct_t;
***************************************************
*/
void ODReadPos_gainData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	
	Spetsnaz->Pos_gain[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->Pos_gain[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->Pos_gain[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->Pos_gain[axis].u8_temp[3] = CanRevData->Data[3];
}

/*
***************************************************
��������ODReadVel_gainsData
���ܣ���ȡ�ٶȻ�����ֵ
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz������ĸ�ߵ�ѹ��ĸ�ߵ��������ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	
	���ص�����ǰ��λ���͵��ߣ���ODRIVE��ĸ�ߵ�ѹ���ݣ�����λ���ص���ODRIVE��ĸ�ߵ������ݣ�����FLOAT��
	�����õ���һ���������������ת���������뿴��
	//����������ת������
	typedef union{
		u8 		u8_temp[4];
		float float_temp;
		s32 	s32_temp;
		u32		u32_temp;
	} formatTrans32Struct_t;
***************************************************
*/
void ODReadVel_gainsData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	
	Spetsnaz->Vel_gain[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->Vel_gain[axis].u8_temp[1] = CanRevData->Data[1];		
	Spetsnaz->Vel_gain[axis].u8_temp[2] = CanRevData->Data[2];		
	Spetsnaz->Vel_gain[axis].u8_temp[3] = CanRevData->Data[3];	
	
	Spetsnaz->Vel_integrator_gain[axis].u8_temp[0] = CanRevData->Data[4];	
	Spetsnaz->Vel_integrator_gain[axis].u8_temp[1] = CanRevData->Data[5];		
	Spetsnaz->Vel_integrator_gain[axis].u8_temp[2] = CanRevData->Data[6];		
	Spetsnaz->Vel_integrator_gain[axis].u8_temp[3] = CanRevData->Data[7];
}

/*
***************************************************
��������ODReadVel_gainsData
���ܣ���ȡ�ٶȻ�����ֵ
��ڲ�����	
			CanRevData��CAN���յ����ݽṹ
			Spetsnaz������ĸ�ߵ�ѹ��ĸ�ߵ��������ݽṹ��
			num: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	
	���ص�����ǰ��λ���͵��ߣ���ODRIVE��ĸ�ߵ�ѹ���ݣ�����λ���ص���ODRIVE��ĸ�ߵ������ݣ�����FLOAT��
	�����õ���һ���������������ת���������뿴��
	//����������ת������
	typedef union{
		u8 		u8_temp[4];
		float float_temp;
		s32 	s32_temp;
		u32		u32_temp;
	} formatTrans32Struct_t;
***************************************************
*/
void CG_Data(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t num) {
		
	
	Spetsnaz->CG[num].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->CG[num].u8_temp[1] = CanRevData->Data[1];
	Spetsnaz->CG[num].u8_temp[2] = CanRevData->Data[2];
	Spetsnaz->CG[num].u8_temp[3] = CanRevData->Data[3];
	
	
}


/*
***************************************************
��������set_axis_requested_state
���ܣ����״̬��������_����֡
��ڲ�����	CANx��CAN1 orCAN2
			ID_CAN��CANID ���0����1��ID��ַ �涨�� AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN��odrive��״̬�����������MSG_SET_CONTROLLER_MODES ��
			len������֡����
			CONTROL_MODE���������ģʽ
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz�����͵ĵ�������ṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע�� uint8_t len�����ݷ�Χ
	 
    enum {
        AXIS_STATE_UNDEFINED             = 0,
		AXIS_STATE_IDLE                  = 1,
		AXIS_STATE_STARTUP_SEQUENCE      = 2,
		AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
		AXIS_STATE_MOTOR_CALIBRATION     = 4,
		AXIS_STATE_ENCODER_INDEX_SEARCH  = 6,
		AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,
		AXIS_STATE_CLOSED_LOOP_CONTROL   = 8,
		AXIS_STATE_LOCKIN_SPIN           = 9,
		AXIS_STATE_ENCODER_DIR_FIND      = 10,
		AXIS_STATE_HOMING                = 11,
		AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12,
		AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13,
    }; 

***************************************************
*/
void set_axis_requested_state(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData) {
	
	CanSendData->data[0] = Spetsnaz->AxisState[axis];
	CanSendData->data[1] = 0;
	CanSendData->data[2] = 0;
	CanSendData->data[3] = 0;

	
	OdriveSendData(CANx,ID_CAN,CMD_CAN,len,CanSendData); 
}

/*
***************************************************
��������ODReadIq_measured
���ܣ���ȡ����Iq����ֵ,�����
��ڲ�����	
			CanSendData��CAN���͵����ݽṹ
			Spetsnaz������ĸ�ߵ�ѹ��ĸ�ߵ��������ݽṹ��
			axis: ѡ����0����1����� �涨 �� axis0=0  axis1=1 
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
	
	���ص�����ǰ��λ���͵��ߣ���ODRIVE��ĸ�ߵ�ѹ���ݣ�����λ���ص���ODRIVE��ĸ�ߵ������ݣ�����FLOAT��
	�����õ���һ���������������ת���������뿴��
	//����������ת������
	typedef union{
		u8 		u8_temp[4];
		float float_temp;
		s32 	s32_temp;
		u32		u32_temp;
	} formatTrans32Struct_t;
***************************************************
*/
void ODReadIq_measured(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis) {
		
	
	Spetsnaz->Iq_measured[axis].u8_temp[0] = CanRevData->Data[4];	
	Spetsnaz->Iq_measured[axis].u8_temp[1] = CanRevData->Data[5];	
	Spetsnaz->Iq_measured[axis].u8_temp[2] = CanRevData->Data[6];	
	Spetsnaz->Iq_measured[axis].u8_temp[3] = CanRevData->Data[7];
}

/*********************************����Ϊ���������*****************************************************************************/



/*********************************����ΪӦ�ò����*****************************************************************************/
ODCANSendStruct_t ODSendData;
static BSP_CAN_TypeDef can1;
static BSP_CAN_TypeDef can2;

/*
***************************************************
��������driver_can1_init
���ܣ����CAN1��ʼ��
��ڲ�����	rm_canx��ʹ�õ�CANͨ��
					rm_canx_rx��RM CAN��������
					rm_canx_tx��RM CAN��������
					Preemption��CAN�ж���ռ���ȼ�
					Sub��CAN�жϴ����ȼ�
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע�� ������Ϊ1M
***************************************************
*/
//��������CANbps=APB1����Ƶ��45M/3/(1+5+9))=1000k bps,3Ϊ����ϵ��,5ΪBS2ʱ�䵥Ԫ,9ΪBS1ʱ�䵥Ԫ
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
	//1M������
	//BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal,Preemption,Sub);
	
	//500K������
	BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,6,CAN_Mode_Normal,Preemption,Sub);
	
	//250K������
	//BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,12,CAN_Mode_Normal,Preemption,Sub);
	
	//125K������
	//BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,24,CAN_Mode_Normal,Preemption,Sub);
}

/*
***************************************************
��������driver_can2_init
���ܣ����CAN2��ʼ��
��ڲ�����	rm_canx��ʹ�õ�CANͨ��
					rm_canx_rx��RM CAN��������
					rm_canx_tx��RM CAN��������
					Preemption��CAN�ж���ռ���ȼ�
					Sub��CAN�жϴ����ȼ�
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע�� ������Ϊ1M
***************************************************
*/
//��������CANbps=APB1����Ƶ��45M/3/(1+5+9))=1000k bps,3Ϊ����ϵ��,5ΪBS2ʱ�䵥Ԫ,9ΪBS1ʱ�䵥Ԫ
void driver_can2_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub){
	can2.CANx = rm_canx;
	can2.CANx_RX = rm_canx_rx;
	can2.CANx_TX = rm_canx_tx;
	if(rm_canx == CAN1){
		can1.CAN_FilterInitStructure = CAN1_FilterInitStructure;
	}
	else if(rm_canx == CAN2){
		
	}
	can2.CAN_FilterInitStructure = CAN2_FilterInitStructure;
	//1M������
	//BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal,Preemption,Sub);
	
	//250K������
	//BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,12,CAN_Mode_Normal,Preemption,Sub);
	
	//125K������
	BSP_CAN_Mode_Init(&can2,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,24,CAN_Mode_Normal,Preemption,Sub);
}

////     Frame
//// nodeID | CMD
//// 6 bits | 5 bits
//CanRxMsg can1_rx_msg;
//u32 rxbuf3;
//void CAN1_RX0_IRQHandler(void){
//	//CanRxMsg can1_rx_msg;
//	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
//		// ����жϱ�־�ͱ�־λ
//		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
//		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
//		
//		// �ӽ��� FIFO �ж�ȡ��Ϣ		
//		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
//		
//		// �洢���յ��ı�׼ ID
//		rxbuf3=can1_rx_msg.StdId;
//		// �����������
//		
//		digitalIncreasing(&OdriveData.OdError.errorCount);

//		/*********�������Զ��岿��**********/
//		switch(can1_rx_msg.StdId>>5){         
//				case AXIS0_ID:
//					//Ŀǰ���մ�������ָ������֧�ֵ�31
//					switch(can1_rx_msg.StdId&0x01F){ 

//						case MSG_ODRIVE_HEARTBEAT:
//							ODReadHeartBeatData(&can1_rx_msg,&OdReceivedData,axis0);
//						break;							
//						
//						case MSG_GET_VBUS_VOLTAGE:
//								ODReadVbusData(&can1_rx_msg,&OdReceivedData,axis0);
//						break;	
//						
////						case MSG_GET_TEMP:
////								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis0);
//											
//						break;								
//					
//						case MSG_GET_MOTOR_ERROR:
//								ODReadMotorErrorData(&can1_rx_msg,&OdReceivedData,axis0);			
//						break;
//						
//						case MSG_GET_ENCODER_COUNT:
//								ODReadEncoderCountData(&can1_rx_msg,&OdReceivedData,axis0);			
//						break;						
//						
//						case MSG_GET_ENCODER_ESTIMATES:
//								ODReadEncodeEstimatesData(&can1_rx_msg,&OdReceivedData,axis0);			
//						break;								
//		
//						case MSG_GET_ENCODER_ERROR:
//								ODReadEncodeErrorData(&can1_rx_msg,&OdReceivedData,axis0);			
//						break;						
//							
//						
//						case MSG_SET_LIMITS:
//								ODReadLimitData(&can1_rx_msg,&OdReceivedData,axis0);			
//						break;

//						case MSG_GET_POS_GAIN:
//								ODReadPos_gainData(&can1_rx_msg,&OdReceivedData,axis0);
//								
//						break;
//						
//						case MSG_GET_VEL_GAINS:
//								ODReadVel_gainsData(&can1_rx_msg,&OdReceivedData,axis0);
//						break;
//						
//						case MSG_GET_MOTOR_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis0);
//						break;
//						
//					default:	break;																			

//					
//					}						
//						
//				break;			
//			
//				case AXIS1_ID:							
//					switch(can1_rx_msg.StdId & 0x01F){ 

//						case MSG_ODRIVE_HEARTBEAT:
//							ODReadHeartBeatData(&can1_rx_msg,&OdReceivedData,axis1);
//						break;							
//						
//						case MSG_GET_VBUS_VOLTAGE:
//								ODReadVbusData(&can1_rx_msg,&OdReceivedData,axis1);
//											
//						break;	
//						
////						case MSG_GET_TEMP:
////								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
//											
//						break;								
//					
//						case MSG_GET_MOTOR_ERROR:
//								ODReadMotorErrorData(&can1_rx_msg,&OdReceivedData,axis1);			
//						break;
//						
//						case MSG_GET_ENCODER_COUNT:
//								ODReadEncoderCountData(&can1_rx_msg,&OdReceivedData,axis1);			
//						break;						
//						
//						case MSG_GET_ENCODER_ESTIMATES:
//								ODReadEncodeEstimatesData(&can1_rx_msg,&OdReceivedData,axis1);			
//						break;								
//		
//						case MSG_GET_ENCODER_ERROR:
//								ODReadEncodeErrorData(&can1_rx_msg,&OdReceivedData,axis1);			
//						break;						
//							
//						
//						case MSG_SET_LIMITS:
//								ODReadLimitData(&can1_rx_msg,&OdReceivedData,axis1);			
//						break;

//						case MSG_GET_VEL_GAINS:
//								ODReadVel_gainsData(&can1_rx_msg,&OdReceivedData,axis1);
//						break;
//						
//						case MSG_GET_POS_GAIN:
//								ODReadPos_gainData(&can1_rx_msg,&OdReceivedData,axis1);
//								
//						break;
//						
//						case MSG_GET_MOTOR_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
//						break;
//						
//					default:	break;			


//																							

//					
//					}
//					break;
//					
//					case AXIS2_ID:							
//					switch(can1_rx_msg.StdId & 0x01F){ 

//						case MSG_ODRIVE_HEARTBEAT:
//							ODReadHeartBeatData(&can1_rx_msg,&OdReceivedData,axis2);
//						break;							
//						
//						case MSG_GET_VBUS_VOLTAGE:
//								ODReadVbusData(&can1_rx_msg,&OdReceivedData,axis2);
//											
//						break;	
//						
////						case MSG_GET_TEMP:
////								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
//											
//						break;								
//					
//						case MSG_GET_MOTOR_ERROR:
//								ODReadMotorErrorData(&can1_rx_msg,&OdReceivedData,axis2);			
//						break;
//						
//						case MSG_GET_ENCODER_COUNT:
//								ODReadEncoderCountData(&can1_rx_msg,&OdReceivedData,axis2);			
//						break;						
//						
//						case MSG_GET_ENCODER_ESTIMATES:
//								ODReadEncodeEstimatesData(&can1_rx_msg,&OdReceivedData,axis2);			
//						break;								
//		
//						case MSG_GET_ENCODER_ERROR:
//								ODReadEncodeErrorData(&can1_rx_msg,&OdReceivedData,axis2);			
//						break;						
//							
//						
//						case MSG_SET_LIMITS:
//								ODReadLimitData(&can1_rx_msg,&OdReceivedData,axis2);			
//						break;

//						case MSG_GET_VEL_GAINS:
//								ODReadVel_gainsData(&can1_rx_msg,&OdReceivedData,axis2);
//						break;
//						
//						case MSG_GET_POS_GAIN:
//								ODReadPos_gainData(&can1_rx_msg,&OdReceivedData,axis2);
//								
//						break;
//						
//						case MSG_GET_MOTOR_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis2);
//						break;
//						
//					default:	break;			


//																						
//					}
//					break;

//					case AXIS3_ID:							
//					switch(can1_rx_msg.StdId & 0x01F){ 

//						case MSG_ODRIVE_HEARTBEAT:
//							ODReadHeartBeatData(&can1_rx_msg,&OdReceivedData,axis3);
//						break;							
//						
//						case MSG_GET_VBUS_VOLTAGE:
//								ODReadVbusData(&can1_rx_msg,&OdReceivedData,axis3);
//											
//						break;	
//						
////						case MSG_GET_TEMP:
////								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
//											
//						break;								
//					
//						case MSG_GET_MOTOR_ERROR:
//								ODReadMotorErrorData(&can1_rx_msg,&OdReceivedData,axis3);			
//						break;
//						
//						case MSG_GET_ENCODER_COUNT:
//								ODReadEncoderCountData(&can1_rx_msg,&OdReceivedData,axis3);			
//						break;						
//						
//						case MSG_GET_ENCODER_ESTIMATES:
//								ODReadEncodeEstimatesData(&can1_rx_msg,&OdReceivedData,axis3);			
//						break;								
//		
//						case MSG_GET_ENCODER_ERROR:
//								ODReadEncodeErrorData(&can1_rx_msg,&OdReceivedData,axis3);			
//						break;						
//							
//						
//						case MSG_SET_LIMITS:
//								ODReadLimitData(&can1_rx_msg,&OdReceivedData,axis3);			
//						break;

//						case MSG_GET_VEL_GAINS:
//								ODReadVel_gainsData(&can1_rx_msg,&OdReceivedData,axis3);
//						break;
//						
//						case MSG_GET_POS_GAIN:
//								ODReadPos_gainData(&can1_rx_msg,&OdReceivedData,axis3);
//								
//						break;
//						
//						case MSG_GET_MOTOR_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis3);
//						break;
//						
//					default:	break;			


//																							

//					
//					}
//						
//				break;					
//				
//				
//			default:	break;		

//        }


//	}
//}


///*
//***************************************************
//��������CAN1_TX_IRQHandler
//���ܣ�CAN1�����ж�
//��ע��
//***************************************************
//*/
//void CAN1_TX_IRQHandler(void){
//	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
//		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

//		/*********�������Զ��岿��**********/
//        OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_MOTOR_ERROR);
//        
//	}
//}



/*
***************************************************
��������CAN2_RX0_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************
*/
u32 rxbuf2;
void CAN2_RX0_IRQHandler(void){
	CanRxMsg can2_rx_msg;
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN2, CAN_IT_FF0);
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0);		
		CAN_Receive(CAN2, CAN_FIFO0, &can2_rx_msg);
		
		rxbuf2=can2_rx_msg.StdId;	
		digitalIncreasing(&OdriveData.OdError.errorCount);
		/*********�������Զ��岿��**********/
			
		switch(can2_rx_msg.StdId>>5){         
				case AXIS0_ID:
					//Ŀǰ���մ�������ָ������֧�ֵ�31
					switch(can2_rx_msg.StdId&0x01F){ 

						case MSG_ODRIVE_HEARTBEAT:
							ODReadHeartBeatData(&can2_rx_msg,&OdReceivedData,axis0);
						break;							
						
						case MSG_GET_VBUS_VOLTAGE:
								ODReadVbusData(&can2_rx_msg,&OdReceivedData,axis0);
						break;	
						
//						case MSG_GET_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis0);
											
						break;								
					
						case MSG_GET_MOTOR_ERROR:
								ODReadMotorErrorData(&can2_rx_msg,&OdReceivedData,axis0);			
						break;
						
						case MSG_GET_ENCODER_COUNT:
								ODReadEncoderCountData(&can2_rx_msg,&OdReceivedData,axis0);			
						break;						
						
						case MSG_GET_ENCODER_ESTIMATES:
								ODReadEncodeEstimatesData(&can2_rx_msg,&OdReceivedData,axis0);			
						break;								
		
						case MSG_GET_ENCODER_ERROR:
								ODReadEncodeErrorData(&can2_rx_msg,&OdReceivedData,axis0);			
						break;						
							
						
						case MSG_SET_LIMITS:
								ODReadLimitData(&can2_rx_msg,&OdReceivedData,axis0);			
						break;

						case MSG_GET_POS_GAIN:
								ODReadPos_gainData(&can2_rx_msg,&OdReceivedData,axis0);
								
						break;
						
						case MSG_GET_VEL_GAINS:
								ODReadVel_gainsData(&can2_rx_msg,&OdReceivedData,axis0);
						break;
						
						case MSG_GET_MOTOR_TEMP:
								ODReadTempData(&can2_rx_msg,&OdReceivedData,axis0);
						break;
						
						case MSG_GET_IQ:
								ODReadIq_measured(&can2_rx_msg,&OdReceivedData,axis0);
						break;
						
					default:	break;																			

					
					}						
						
				break;			
			
				case AXIS1_ID:							
					switch(can2_rx_msg.StdId & 0x01F){ 

						case MSG_ODRIVE_HEARTBEAT:
							ODReadHeartBeatData(&can2_rx_msg,&OdReceivedData,axis1);
						break;							
						
						case MSG_GET_VBUS_VOLTAGE:
								ODReadVbusData(&can2_rx_msg,&OdReceivedData,axis1);
											
						break;	
						
//						case MSG_GET_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
											
						break;								
					
						case MSG_GET_MOTOR_ERROR:
								ODReadMotorErrorData(&can2_rx_msg,&OdReceivedData,axis1);			
						break;
						
						case MSG_GET_ENCODER_COUNT:
								ODReadEncoderCountData(&can2_rx_msg,&OdReceivedData,axis1);			
						break;						
						
						case MSG_GET_ENCODER_ESTIMATES:
								ODReadEncodeEstimatesData(&can2_rx_msg,&OdReceivedData,axis1);			
						break;								
		
						case MSG_GET_ENCODER_ERROR:
								ODReadEncodeErrorData(&can2_rx_msg,&OdReceivedData,axis1);			
						break;						
							
						
						case MSG_SET_LIMITS:
								ODReadLimitData(&can2_rx_msg,&OdReceivedData,axis1);			
						break;

						case MSG_GET_VEL_GAINS:
								ODReadVel_gainsData(&can2_rx_msg,&OdReceivedData,axis1);
						break;
						
						case MSG_GET_POS_GAIN:
								ODReadPos_gainData(&can2_rx_msg,&OdReceivedData,axis1);
								
						break;
						
						case MSG_GET_MOTOR_TEMP:
								ODReadTempData(&can2_rx_msg,&OdReceivedData,axis1);
						break;
						
						case MSG_GET_IQ:
								ODReadIq_measured(&can2_rx_msg,&OdReceivedData,axis1);
						break;
						
					default:	break;			


																							

					
					}
					break;
					
					case AXIS2_ID:							
					switch(can2_rx_msg.StdId & 0x01F){ 

						case MSG_ODRIVE_HEARTBEAT:
							ODReadHeartBeatData(&can2_rx_msg,&OdReceivedData,axis2);
						break;							
						
						case MSG_GET_VBUS_VOLTAGE:
								ODReadVbusData(&can2_rx_msg,&OdReceivedData,axis2);
											
						break;	
						
//						case MSG_GET_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
											
						break;								
					
						case MSG_GET_MOTOR_ERROR:
								ODReadMotorErrorData(&can2_rx_msg,&OdReceivedData,axis2);			
						break;
						
						case MSG_GET_ENCODER_COUNT:
								ODReadEncoderCountData(&can2_rx_msg,&OdReceivedData,axis2);			
						break;						
						
						case MSG_GET_ENCODER_ESTIMATES:
								ODReadEncodeEstimatesData(&can2_rx_msg,&OdReceivedData,axis2);			
						break;								
		
						case MSG_GET_ENCODER_ERROR:
								ODReadEncodeErrorData(&can2_rx_msg,&OdReceivedData,axis2);			
						break;						
							
						
						case MSG_SET_LIMITS:
								ODReadLimitData(&can2_rx_msg,&OdReceivedData,axis2);			
						break;

						case MSG_GET_VEL_GAINS:
								ODReadVel_gainsData(&can2_rx_msg,&OdReceivedData,axis2);
						break;
						
						case MSG_GET_POS_GAIN:
								ODReadPos_gainData(&can2_rx_msg,&OdReceivedData,axis2);
								
						break;
						
						case MSG_GET_MOTOR_TEMP:
								ODReadTempData(&can2_rx_msg,&OdReceivedData,axis2);
						break;
						
						case MSG_GET_IQ:
								ODReadIq_measured(&can2_rx_msg,&OdReceivedData,axis2);
						break;
						
					default:	break;			


																						
					}
					break;

					case AXIS3_ID:							
					switch(can2_rx_msg.StdId & 0x01F){ 

						case MSG_ODRIVE_HEARTBEAT:
							ODReadHeartBeatData(&can2_rx_msg,&OdReceivedData,axis3);
						break;							
						
						case MSG_GET_VBUS_VOLTAGE:
								ODReadVbusData(&can2_rx_msg,&OdReceivedData,axis3);
											
						break;	
						
//						case MSG_GET_TEMP:
//								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
											
						break;								
					
						case MSG_GET_MOTOR_ERROR:
								ODReadMotorErrorData(&can2_rx_msg,&OdReceivedData,axis3);			
						break;
						
						case MSG_GET_ENCODER_COUNT:
								ODReadEncoderCountData(&can2_rx_msg,&OdReceivedData,axis3);			
						break;						
						
						case MSG_GET_ENCODER_ESTIMATES:
								ODReadEncodeEstimatesData(&can2_rx_msg,&OdReceivedData,axis3);			
						break;								
		
						case MSG_GET_ENCODER_ERROR:
								ODReadEncodeErrorData(&can2_rx_msg,&OdReceivedData,axis3);			
						break;						
							
						
						case MSG_SET_LIMITS:
								ODReadLimitData(&can2_rx_msg,&OdReceivedData,axis3);			
						break;

						case MSG_GET_VEL_GAINS:
								ODReadVel_gainsData(&can2_rx_msg,&OdReceivedData,axis3);
						break;
						
						case MSG_GET_POS_GAIN:
								ODReadPos_gainData(&can2_rx_msg,&OdReceivedData,axis3);
								
						break;
						
						case MSG_GET_MOTOR_TEMP:
								ODReadTempData(&can2_rx_msg,&OdReceivedData,axis3);
						break;
						
						case MSG_GET_IQ:
								ODReadIq_measured(&can2_rx_msg,&OdReceivedData,axis3);
						break;
						
					default:	break;			


																							

					
					}
						
				break;					
				
				
			default:	break;		

        }
		

	}
}


/*
***************************************************
��������CAN2_TX_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************
*/
void CAN2_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);

		/*********�������Զ��岿��**********/
        
        
	}
}




void ODRequestedState(void){		
    //�����Ҫ��״̬�������в�����ֱ�ӽ�OdriveData.flashSave����һ�μ���    
	if(OdriveData.RequestedStateFlag&&(OdriveData.AxisState[axis0] == AXIS_STATE_IDLE)){                                              
       //��⵽��Ҫ�ı�״̬����		
		
		#if USE_CAN1
		ODSET_CONTROL_MODE(CAN1,AXIS0_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis0,&ODSendData);
		ODSET_CONTROL_MODE(CAN1,AXIS1_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis1,&ODSendData);
		#endif
		
		#if USE_CAN2
		ODSET_CONTROL_MODE(CAN2,AXIS0_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis0,&ODSendData);
		ODSET_CONTROL_MODE(CAN2,AXIS1_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis1,&ODSendData);
		#endif
		
		

		digitalLo(&OdriveData.RequestedStateFlag);         
        
	}

	if(OdriveData.RequestedStateFlag1&&(OdriveData.AxisState[axis0] == AXIS_STATE_IDLE)){                                              
       //��⵽��Ҫ�ı�״̬����		
		
		#if USE_CAN1
		ODSET_CONTROL_MODE(CAN1,AXIS2_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis2,&ODSendData);
		ODSET_CONTROL_MODE(CAN1,AXIS3_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis3,&ODSendData);
		#endif
		
		#if USE_CAN2
		ODSET_CONTROL_MODE(CAN2,AXIS2_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis2,&ODSendData);
		ODSET_CONTROL_MODE(CAN2,AXIS3_ID,MSG_SET_CONTROLLER_MODES,8,&OdriveData,axis3,&ODSendData);
		#endif
		

		digitalLo(&OdriveData.RequestedStateFlag1);         
        
	}

}

//�������
void ODFlashSave(void){
	//������1�����2����	
	if(OdriveData.flashSaveFlag){                                              						
		//config
		#if USE_CAN1
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_SAVE_CONFIG);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_SAVE_CONFIG);
		#endif
		
		#if USE_CAN2
		OdriveSend_RemoteCmd(CAN2,AXIS0_ID,MSG_SAVE_CONFIG);
		OdriveSend_RemoteCmd(CAN2,AXIS1_ID,MSG_SAVE_CONFIG);
		#endif
		
		digitalLo(&OdriveData.flashSaveFlag);         
        
	} 
	
	//������3�����4����
	if(OdriveData.flashSaveFlag1){
		#if USE_CAN1
		OdriveSend_RemoteCmd(CAN1,AXIS2_ID,MSG_SAVE_CONFIG);
		OdriveSend_RemoteCmd(CAN1,AXIS3_ID,MSG_SAVE_CONFIG);
		#endif
		
		#if USE_CAN2
		OdriveSend_RemoteCmd(CAN2,AXIS2_ID,MSG_SAVE_CONFIG);
		OdriveSend_RemoteCmd(CAN2,AXIS3_ID,MSG_SAVE_CONFIG);
		#endif
		
		digitalLo(&OdriveData.flashSaveFlag1);         
        
	} 
}

//�޸��ٶȻ��������
void ODChangeVel_gain(void){
	//�޸ĵ��1�����2���ٶȻ�����	
	if(OdriveData.Vel_gainFlag){
		#if USE_CAN1
		ODSendVel_gainsData(CAN1,AXIS0_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis0,&ODSendData);
		ODSendVel_gainsData(CAN1,AXIS1_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis1,&ODSendData);
		#endif
		
		#if USE_CAN2
		ODSendVel_gainsData(CAN2,AXIS0_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis0,&ODSendData);
		ODSendVel_gainsData(CAN2,AXIS1_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis1,&ODSendData);
		#endif
		
		digitalLo(&OdriveData.Vel_gainFlag);         
	}
	//�޸ĵ��3�����4���ٶȻ�����
	if(OdriveData.Vel_gainFlag1){
		#if USE_CAN1
		ODSendVel_gainsData(CAN1,AXIS2_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis2,&ODSendData);
		ODSendVel_gainsData(CAN1,AXIS3_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis3,&ODSendData);
		#endif
		
		#if USE_CAN2
		ODSendVel_gainsData(CAN2,AXIS2_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis2,&ODSendData);
		ODSendVel_gainsData(CAN2,AXIS3_ID,MSG_SET_VEL_GAINS,8,&OdriveData,axis3,&ODSendData);
		#endif
		digitalLo(&OdriveData.Vel_gainFlag1);         
        
	}
	
}

//�޸�λ�û����������
void ODChangePos_gain(void){
	//�޸ĵ��1�����2��λ�û�����
	if(OdriveData.Pos_gainFlag){                                              						
		//config
		#if USE_CAN1
		ODSendPos_gainData(CAN1,AXIS0_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis0,&ODSendData);
		ODSendPos_gainData(CAN1,AXIS1_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis1,&ODSendData);
		#endif
		
		#if USE_CAN2
		ODSendPos_gainData(CAN2,AXIS0_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis0,&ODSendData);
		ODSendPos_gainData(CAN2,AXIS1_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis1,&ODSendData);
		#endif
		
		digitalLo(&OdriveData.Pos_gainFlag);         
	}
	//�޸ĵ��3�����4��λ�û�����
	if(OdriveData.Pos_gainFlag1){
		#if USE_CAN1
		ODSendPos_gainData(CAN1,AXIS2_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis2,&ODSendData);
		ODSendPos_gainData(CAN1,AXIS3_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis3,&ODSendData);
		#endif
		
		#if USE_CAN2
		ODSendPos_gainData(CAN2,AXIS2_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis2,&ODSendData);
		ODSendPos_gainData(CAN2,AXIS3_ID,MSG_SET_POS_GAIN,4,&OdriveData,axis3,&ODSendData);
		#endif
		digitalLo(&OdriveData.Pos_gainFlag1);         
        
	}
	
}



//����������
void ODClearError(void){		
    //�����Ҫ��״̬�������в�����ֱ�ӽ�OdriveData.clearerror��һ�μ���
	//������1�����2����    
	if(OdriveData.clearerrorFlag){
		#if	USE_CAN1
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_CLEAR_ERRORS);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_CLEAR_ERRORS);
		#endif
		
		#if USE_CAN2
		OdriveSend_RemoteCmd(CAN2,AXIS0_ID,MSG_CLEAR_ERRORS);
		OdriveSend_RemoteCmd(CAN2,AXIS1_ID,MSG_CLEAR_ERRORS);
		#endif
		digitalLo(&OdriveData.clearerrorFlag);         
        
	}
	//������3�����4����
	if(OdriveData.clearerrorFlag1){
		
		#if USE_CAN1
		OdriveSend_RemoteCmd(CAN1,AXIS2_ID,MSG_CLEAR_ERRORS);
		OdriveSend_RemoteCmd(CAN1,AXIS3_ID,MSG_CLEAR_ERRORS);
		#endif
		
		#if USE_CAN2
		OdriveSend_RemoteCmd(CAN2,AXIS2_ID,MSG_CLEAR_ERRORS);
		OdriveSend_RemoteCmd(CAN2,AXIS3_ID,MSG_CLEAR_ERRORS);
		#endif
		digitalLo(&OdriveData.clearerrorFlag1);         
        
	}
}


//����odrive
void ODRESET_ODRIVE(void){
	//�������1�����2
	if(OdriveData.RebotFlag){                                              						
		//config
		#if USE_CAN1
		OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_RESET_ODRIVE);
		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_RESET_ODRIVE);
		#endif
		
		#if USE_CAN2
		OdriveSend_RemoteCmd(CAN2,AXIS0_ID,MSG_RESET_ODRIVE);
		OdriveSend_RemoteCmd(CAN2,AXIS1_ID,MSG_RESET_ODRIVE);
		#endif
		digitalLo(&OdriveData.RebotFlag);         
        
	}
	//�������3�����4
	if(OdriveData.RebotFlag1){                                              						
		//config
		#if USE_CAN1
		OdriveSend_RemoteCmd(CAN1,AXIS2_ID,MSG_RESET_ODRIVE);
		OdriveSend_RemoteCmd(CAN1,AXIS3_ID,MSG_RESET_ODRIVE);
		#endif
		
		#if USE_CAN2
		OdriveSend_RemoteCmd(CAN2,AXIS2_ID,MSG_RESET_ODRIVE);
		OdriveSend_RemoteCmd(CAN2,AXIS3_ID,MSG_RESET_ODRIVE);
		#endif
        digitalLo(&OdriveData.RebotFlag1);
	}
}


//���õ������״̬
void ODSetMotorState(void){		
    //�����Ҫ�Ե������ģʽ���в�����ֱ�ӽ�OdriveData.ControlModeFlag��һ�μ���,���ҵ������ģʽΪCMD_MENU
	if((OdriveData.ControlModeFlag)){                                              
		//�޸ĵ��1�����2״̬
		#if USE_CAN1
		set_axis_requested_state(CAN1,AXIS0_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis0,&ODSendData);
		set_axis_requested_state(CAN1,AXIS1_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis1,&ODSendData);
		#endif

		#if USE_CAN2
		set_axis_requested_state(CAN2,AXIS0_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis0,&ODSendData);
		set_axis_requested_state(CAN2,AXIS1_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis1,&ODSendData);
		#endif
		digitalLo(&OdriveData.ControlModeFlag);         
        
	}

	if((OdriveData.ControlModeFlag1)){                                              
		//�޸ĵ��3�����4״̬
		#if USE_CAN1
		set_axis_requested_state(CAN1,AXIS2_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis2,&ODSendData);
		set_axis_requested_state(CAN1,AXIS3_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis3,&ODSendData);
		#endif
		
		#if USE_CAN2
		set_axis_requested_state(CAN2,AXIS2_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis2,&ODSendData);
		set_axis_requested_state(CAN2,AXIS3_ID,MSG_SET_AXIS_REQUESTED_STATE,4,&OdriveData,axis3,&ODSendData);
		#endif
		digitalLo(&OdriveData.ControlModeFlag1);         
        
	}
}

//���õ������״̬
void ODSetLimit(void){		
    //�����Ҫ�Ե���ٶȵ������ƽ��в�����ֱ�ӽ�OdriveData.SetLimitFlag��һ�μ���,���ҵ������ģʽΪAXIS_STATE_IDLE
	if((OdriveData.SetLimitFlag)&&(OdriveData.AxisState[axis0] == AXIS_STATE_IDLE)&&(OdriveData.AxisState[axis1] == AXIS_STATE_IDLE)){                                              
			
		#if USE_CAN1
		ODSendLimitData(CAN1,AXIS0_ID,MSG_SET_LIMITS,8,&OdriveData,axis0,&ODSendData);
		#endif

		#if USE_CAN2
		ODSendLimitData(CAN2,AXIS0_ID,MSG_SET_LIMITS,8,&OdriveData,axis0,&ODSendData);
		ODSendLimitData(CAN2,AXIS1_ID,MSG_SET_LIMITS,8,&OdriveData,axis1,&ODSendData);
		#endif
		digitalLo(&OdriveData.SetLimitFlag);         
        
	}
	
	if((OdriveData.SetLimitFlag1)&&(OdriveData.AxisState[axis2] == AXIS_STATE_IDLE)&&(OdriveData.AxisState[axis3] == AXIS_STATE_IDLE)){                                              
			
		#if USE_CAN1
		ODSendLimitData(CAN1,AXIS2_ID,MSG_SET_LIMITS,8,&OdriveData,axis2,&ODSendData);
		#endif

		#if USE_CAN2
		ODSendLimitData(CAN2,AXIS2_ID,MSG_SET_LIMITS,8,&OdriveData,axis2,&ODSendData);
		ODSendLimitData(CAN2,AXIS3_ID,MSG_SET_LIMITS,8,&OdriveData,axis3,&ODSendData);
		#endif
		digitalLo(&OdriveData.SetLimitFlag1);         
        
	}
	
}



void OdriveGlobalInit(void){
	
	//��ʼ��CAN1
    driver_can1_init(CAN1,BSP_GPIOB8,BSP_GPIOB9,4,0);//405
	//driver_can1_init(CAN1,BSP_GPIOD0,BSP_GPIOD1,4,0);//407
	//driver_can2_init(CAN2,BSP_GPIOB12,BSP_GPIOB13,4,0);
	
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	
	//�����ʼ����	
	OdriveData.AxisState[axis0]   = AXIS_STATE_CLOSED_LOOP_CONTROL;//ʹ�ܵ��1			
	OdriveData.ControlMode[axis0] = CONTROL_MODE_VELOCITY_RAMP;//�ٶ�����ģʽ
	OdriveData.current_limit[axis0].float_temp = 15.0f;
	OdriveData.vel_limit[axis0].float_temp = 35.0f;
	
	OdriveData.AxisState[axis1]   = AXIS_STATE_CLOSED_LOOP_CONTROL;//ʹ�ܵ��2			
	OdriveData.ControlMode[axis1] = CONTROL_MODE_VELOCITY_RAMP;//�ٶ�����ģʽ
	OdriveData.current_limit[axis1].float_temp = 15.0f;
	OdriveData.vel_limit[axis1].float_temp = 35.0f;
	
	OdriveData.AxisState[axis2]   = AXIS_STATE_CLOSED_LOOP_CONTROL;//ʹ�ܵ��3			
	OdriveData.ControlMode[axis2] = CONTROL_MODE_VELOCITY_RAMP;//�ٶ�����ģʽ
	OdriveData.current_limit[axis2].float_temp = 15.0f;
	OdriveData.vel_limit[axis2].float_temp = 35.0f;
	
	OdriveData.AxisState[axis3]   = AXIS_STATE_CLOSED_LOOP_CONTROL;//ʹ�ܵ��4			
	OdriveData.ControlMode[axis3] = CONTROL_MODE_VELOCITY_RAMP;//�ٶ�����ģʽ
	OdriveData.current_limit[axis3].float_temp = 15.0f;
	OdriveData.vel_limit[axis3].float_temp = 35.0f;
	
}

//ODrive���Ƹ�������
void odrivelUpdateTask(void *Parameters){
	//��ȡ��ǰ�������е�ʱ��
	TickType_t xLastWakeTime = xTaskGetTickCount();
	//����״̬
	digitalLo(&OdriveData.dataInitFlag);
	while(true){
		
		vTaskDelayUntil(&xLastWakeTime,ODRIVE_NORMAL_PERIOD);
        //��ֹ�ظ���ʼ��
		if(!OdriveData.dataInitFlag){	
            //���п���ȫ����ʼ��            
			OdriveGlobalInit();
			
			digitalHi(&OdriveData.dataInitFlag);
			
		}
		
	//���������ڱջ�״̬����������/λ��/�ٶȿ���ָ��  Ƶ��100HZ
	if(EW.change == 0){
	if(OdriveData.AxisState[axis0] == AXIS_STATE_CLOSED_LOOP_CONTROL){
					
					switch(OdriveData.ControlMode[axis0]){ 
		
						case CONTROL_MODE_CURRENT:
						case CONTROL_MODE_CURRENT_RAMP:

							 //ͨ�����������������
							//OdriveData.SetCur[0].float_temp;	
						
							//������������
							ODSendInputCurData(CAN1,AXIS0_ID,MSG_SET_INPUT_TORQUE,4,&OdriveData,axis0,&ODSendData);	
							ODSendInputCurData(CAN1,AXIS1_ID,MSG_SET_INPUT_TORQUE,4,&OdriveData,axis0,&ODSendData);
						break;	
						
						case CONTROL_MODE_VELOCITY:
						case CONTROL_MODE_VELOCITY_RAMP:
							
							//ͨ��������������ٶ�
							//OdriveData.SetVel[0].float_temp;
						
							//�����ٶ�����
//							if(flag == 1){
								
								if(NFC.flag == 0){
									#if USE_CAN1
									ODSendInputVelData(CAN1,AXIS0_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
									ODSendInputVelData(CAN1,AXIS2_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
									#endif
									
									#if USE_CAN2
									//ODSendInputVelData(CAN1,AXIS0_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
									ODSendInputVelData(CAN2,AXIS0_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
									ODSendInputVelData(CAN2,AXIS2_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
									#endif
								}
								NFC.flag++;
//								ODSendInputVelData(CAN2,AXIS1_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis1,&ODSendData);
//								ODSendInputVelData(CAN2,AXIS0_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
//								flag = 0;
//							}
						break;
						case CONTROL_MODE_POSITION:
						case CONTROL_MODE_POSITION_TRAP:

							//ͨ�������������λ��						
							//OdriveData.SetPos[0].float_temp;
						
							//����λ������
							ODSendInputPosData(CAN1,AXIS0_ID,MSG_SET_INPUT_POS,4,&OdriveData,axis0,&ODSendData);
							//ODSendInputPosData(CAN1,AXIS1_ID,MSG_SET_INPUT_POS,4,&OdriveData,axis0,&ODSendData);
						
						break;																		
					default:	break;																								
					}			
		
	}
	//���������ڱջ�״̬����������/λ��/�ٶȿ���ָ��  Ƶ��100HZ
	if(OdriveData.AxisState[axis1] == AXIS_STATE_CLOSED_LOOP_CONTROL){

					switch(OdriveData.ControlMode[axis1]){ 
		
						case CONTROL_MODE_CURRENT:
						case CONTROL_MODE_CURRENT_RAMP:

							 //ͨ�����������������
							//OdriveData.SetCur[0].float_temp;	
						
							//������������
							ODSendInputCurData(CAN1,AXIS1_ID,MSG_SET_INPUT_TORQUE,4,&OdriveData,axis1,&ODSendData);	
						break;	
						
						case CONTROL_MODE_VELOCITY:
						case CONTROL_MODE_VELOCITY_RAMP:
							
							//ͨ��������������ٶ�
							//OdriveData.SetVel[0].float_temp;
						
							//�����ٶ�����
//								if(flag == 1){
								
								if(NFC.flag == 2){
									#if USE_CAN1
									ODSendInputVelData(CAN1,AXIS1_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis1,&ODSendData);
									ODSendInputVelData(CAN1,AXIS3_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis1,&ODSendData);
									#endif
									
									#if USE_CAN2
									ODSendInputVelData(CAN2,AXIS1_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis1,&ODSendData);
									ODSendInputVelData(CAN2,AXIS3_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis1,&ODSendData);
									#endif
									NFC.flag = 0;
								}
								
								//ODSendInputVelData(CAN1,AXIS0_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
								
//								ODSendInputVelData(CAN2,AXIS1_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis1,&ODSendData);
//								ODSendInputVelData(CAN2,AXIS0_ID,MSG_SET_INPUT_VEL,4,&OdriveData,axis0,&ODSendData);
//								flag = 0;
//							}
								
						
						break;							
						case CONTROL_MODE_POSITION:
						case CONTROL_MODE_POSITION_TRAP:

							//ͨ�������������λ��						
							//OdriveData.SetPos[0].float_temp;
						
							//����λ������
							//ODSendInputPosData(CAN1,AXIS1_ID,MSG_SET_INPUT_POS,4,&OdriveData,axis1,&ODSendData);	
						
						break;																		
					default:	break;																								
					}			
		
	}
}
	
	//ͨ����������޸�λ�û��������
	//OdriveData.Pos_gain[0].float_temp = test;
	//����λ�û���������޸�����
	
	//10Hz  ��ѭ���͡�
	if(!((OdriveData.loops + 1) % 100)){	

		ODSetMotorState();		
		//����״̬��Ϣ
		ODRequestedState();		
////	//�������
		ODFlashSave();
////	//�������
		ODClearError();
		
		//����odrive
		ODRESET_ODRIVE();
		
//		//���õ������/�ٶ�����
		ODSetLimit();
//		


		
		//��ȡ������CPRֵ
//		OdriveSend_RemoteCmd(CAN1,AXIS1_ID,MSG_GET_ENCODER_COUNT);
		
		//�޸��ٶȻ�����
		ODChangeVel_gain();
		
		//�޸�λ�û�����
		ODChangePos_gain();
		
	}
        vofa_sendData(OdReceivedData.pos_estimate[0].float_temp,OdReceivedData.pos_estimate[1].float_temp,(OdReceivedData.vel_estimate[0].float_temp * 0.2199f),(OdReceivedData.vel_estimate[1].float_temp * 0.2199f),EW.Current_Mileage,OdReceivedData.ibus[1].float_temp,OdReceivedData.vbus_voltage[0].float_temp,OdReceivedData.Pos_gain[0].float_temp,OdReceivedData.Vel_gain[0].float_temp,OdReceivedData.Vel_integrator_gain[0].float_temp,OdReceivedData.Pos_gain[1].float_temp,OdReceivedData.Vel_gain[1].float_temp,OdReceivedData.Vel_integrator_gain[1].float_temp,MBSpeed / 100.0f);
		//Vofa_sendData(SM_Recv.vel_estimate[0].s32_temp / 60.0f);
		//printf("�ٶ�=%d\r\n",SM_Recv.vel_estimate[0].s32_temp);
		//vofa_sendData(SM_Recv.vel_estimate[0].s32_temp,SM_Recv.pos_estimate[0].s32_temp,(OdReceivedData.vel_estimate[0].float_temp * 0.2199f),(OdReceivedData.vel_estimate[1].float_temp * 0.2199f),EW.Current_Mileage,OdReceivedData.ibus[0].float_temp,OdReceivedData.vbus_voltage[0].float_temp,OdReceivedData.Pos_gain[0].float_temp,OdReceivedData.Vel_gain[0].float_temp,OdReceivedData.Vel_integrator_gain[0].float_temp,OdReceivedData.Pos_gain[1].float_temp,OdReceivedData.Vel_gain[1].float_temp,OdReceivedData.Vel_integrator_gain[1].float_temp,Motor_SpeedB_Goal.target);
		digitalIncreasing(&OdriveData.loops);        

	}
}

void OdriveInit(void){
	getsupervisorData()->taskEvent[ODRIVE_TASK] = xTaskCreate(odrivelUpdateTask,"ODRIVE",ODRIVE_STACK_SIZE,NULL,ODRIVE_PRIORITY,&OdriveData.xHandleTask);
    
}




