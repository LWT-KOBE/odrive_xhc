#include "board.h"

wiredControlStruct_t wiredControlData;

slaveSensorStruct_t slaveSensorData;

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�		 //
//			���ض�����������ɿ��Ը����⣬		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

void wiredTransTypeSwitch(uint16_t state, uint8_t value){
	if(value)
		wiredControlData.transType |= state;
	else
		wiredControlData.transType &= ~state;
}


//�ӻ����͵�����
void wiredSendData(USART_TypeDef *USARTx){ 
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	uint8_t index = 0;
	//֡ͷ������4��1kHz
	array[index_ptr++] = MAIN_CONTROL_BEGIN;
	array[index_ptr++] = MAIN_CONTROL_ADDRESS + wiredControlData.transType;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
	
	//������״̬��־λ������3��1kHz
	supervisrCheckModule();
	array[index_ptr++] = getsupervisorData()->modularState;

	
	//�Ƕ����ݣ�����14��500Hz
	if(wiredControlData.transType & TRANS_ADD_ANGLE){
		wiredTransTypeSwitch(TRANS_ADD_ANGLE, DISABLE);
		
		//32λ���ݣ�����ֻ�ܷ�8λ���ݣ��ʽ����Ĵ�forѭ��
		for(index = 0; index < 4; index++){ 
			array[index_ptr++] = imuSensorData.pitch.u8_temp[index];
		}		
		for(index = 0; index < 4; index++){
			array[index_ptr++] = imuSensorData.roll.u8_temp[index];
		}
		for(index = 0; index < 4; index++){
			array[index_ptr++] = imuSensorData.yaw.u8_temp[index];	
		}		

		
	}
	
	//�����ǵ����ݣ��������ٶȣ����ٶȣ��¶ȣ�����28��500Hz
	if(wiredControlData.transType & TRANS_ADD_IMU_SENSOR){
		//wiredTransTypeSwitch(TRANS_ADD_IMU_SENSOR, DISABLE);    
		
		//���ٶ�
		for(index = 0; index < 4; index++){  //32λ
			array[index_ptr++] = imuSensorData.normalizedGyo[0].u8_temp[index];
		}		
		for(index = 0; index < 4; index++){//32λ
			array[index_ptr++] = imuSensorData.normalizedGyo[1].u8_temp[index];
		}
		for(index = 0; index < 4; index++){//32λ
			array[index_ptr++] = imuSensorData.normalizedGyo[2].u8_temp[index];
		}

		//���ٶ�
		for(index = 0; index < 4; index++){  //32λ
			array[index_ptr++] = imuSensorData.normalizedAcc[0].u8_temp[index];
		}		
		for(index = 0; index < 4; index++){//32λ
			array[index_ptr++] = imuSensorData.normalizedAcc[1].u8_temp[index];
		}
		for(index = 0; index < 4; index++){//32λ
			array[index_ptr++] = imuSensorData.normalizedAcc[2].u8_temp[index];
		}		
	
		//�¶�
		for(index = 0; index < 4; index++){  //32λ
			array[index_ptr++] = imuSensorData.temp.u8_temp[index];
		}	

		
	}	
	
	//���������ݣ�����12��500Hz
	if(wiredControlData.transType & TRANS_ADD_MAG){
		//wiredTransTypeSwitch(TRANS_ADD_MAG, DISABLE);
		
		for(index = 0; index < 4; index++){  //32λ
			array[index_ptr++] = imuSensorData.normalizedMag[0].u8_temp[index];
		}		
		for(index = 0; index < 4; index++){//32λ
			array[index_ptr++] = imuSensorData.normalizedMag[1].u8_temp[index];
		}
		for(index = 0; index < 4; index++){//32λ
			array[index_ptr++] = imuSensorData.normalizedMag[2].u8_temp[index];
		}
	}	

		
	//��װУ��λ
	array[2] = index_ptr + 2;
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
	BSP_USART_DMA_SendData(USARTx, array, (index_ptr + 2));
}



//��ȡ�ӻ���IMU״̬��Ϣ
void modularDataDecoding(uint8_t array){

	wiredTransTypeSwitch(MODULAR_IMU_LOSS, array & 0x01);
	wiredTransTypeSwitch(MODULAR_IMU_CALI, array & 0x02);
}

//��ȡ�ӻ�IMU����
static void slaveImuDataRead(uint8_t *array){
	uint8_t index_ptr = 4;
	uint8_t index = 0;
	
	//�ӻ�������״̬��־λ������3��1kHz
	modularDataDecoding(array[index_ptr++]);
	
	//�ӻ��Ƕ����ݣ�����14��500Hz
	if(array[1] & TRANS_ADD_ANGLE){
		for(index=0;index < 4;index++)
			slaveSensorData.pitch.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.roll.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.yaw.u8_temp[index] = array[index_ptr++];

	
		
	}
	
	//�ӻ������ǵ����ݣ��������ٶȣ����ٶȣ��¶ȣ�����28��500Hz
	if(array[1] & TRANS_ADD_IMU_SENSOR){
		
		//���ٶ�		
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedGyo[0].u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedGyo[1].u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedGyo[2].u8_temp[index] = array[index_ptr++];

		//���ٶ�		
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedAcc[0].u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedAcc[1].u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedAcc[2].u8_temp[index] = array[index_ptr++];		
		
		//�¶�
		for(index=0;index < 4;index++)
			slaveSensorData.temp.u8_temp[index] = array[index_ptr++];			
		
	
		
	}	
	
	//�ӻ����������ݣ�����12��500Hz	
	if(array[1] & TRANS_ADD_MAG){

		
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedMag[0].u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedMag[1].u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			slaveSensorData.normalizedMag[2].u8_temp[index] = array[index_ptr++];	
	}


	
	
}

//������ȡ�ӻ�������������
void slaveSensorRead(uint8_t *array){
	if(array[0] == MAIN_CONTROL_BEGIN && (array[1] & MAIN_CONTROL_ADDRESS) ){
		if(!Verify_CRC8_Check_Sum(array, 4) && !Verify_CRC16_Check_Sum(array, array[2])){					
			// У��ʧ���򲻵�������
			//��ʱ�����в���
		}
		else{
			//��ȡ�ӻ�imu����
			slaveImuDataRead(array);
		}
	}
}

//�ӻ����ڳ�ʼ��
void driver_SlaveInit(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,
												u32 baudRate,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef SLAVE_USART;
	SLAVE_USART.USARTx = USARTx;
	SLAVE_USART.USART_RX = USART_RX;
	SLAVE_USART.USART_TX = USART_TX;
	SLAVE_USART.USART_InitStructure.USART_BaudRate = baudRate;
	SLAVE_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	SLAVE_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	SLAVE_USART.USART_InitStructure.USART_Parity = USART_Parity_No;
	SLAVE_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	SLAVE_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	BSP_USART_Init(&SLAVE_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&SLAVE_USART);
	BSP_USART_TX_DMA_Init(&SLAVE_USART);
}

//�������ڳ�ʼ��
void driver_MasterInit(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,
												u32 baudRate,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef MASTER_USART;
	MASTER_USART.USARTx = USARTx;
	MASTER_USART.USART_RX = USART_RX;
	MASTER_USART.USART_TX = USART_TX;
	MASTER_USART.USART_InitStructure.USART_BaudRate = baudRate;
	MASTER_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	MASTER_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	MASTER_USART.USART_InitStructure.USART_Parity = USART_Parity_No;
	MASTER_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	MASTER_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	BSP_USART_Init(&MASTER_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&MASTER_USART);
	BSP_USART_TX_DMA_Init(&MASTER_USART);
}









void wiredControlInit(void){
	
#ifdef IMU_SLAVE
	
	driver_SlaveInit(MASTER_USARTX, MASTER_USARTX_RX_PIN, MASTER_USARTX_TX_PIN,\
					 MASTER_USART_BAUD_RATE, MASTER_USART_PRE_PRIORITY, MASTER_USART_SUB_PRIORITY);	
	//SlaveBleInit_1();
#else
	driver_MasterInit(SLAVE_USARTX, SLAVE_USARTX_RX_PIN, SLAVE_USARTX_TX_PIN,\
					 SLAVE_USART_BAUD_RATE, SLAVE_USART_PRE_PRIORITY, SLAVE_USART_SUB_PRIORITY);	
	
	//MasterBleInit();
	
#endif
	
}
