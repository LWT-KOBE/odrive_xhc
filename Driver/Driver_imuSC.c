#include "Driver_imuSC.h"
#include "cansend.h"
#include "bsp.h"
#include <string.h>
#include <math.h>
#include "driver.h"

imuBroadcastStruct_t imuBroadcast;
imuBroadcastStruct_t *getScimuData(){
	return &imuBroadcast;
}

/**************************************************/
static void imuSCInit(void);

/**************************************************/

deviceInitClass IMUIintClass = {
	imuSCInit,
};


float ByteToFloat(uint8_t *src, int index) {
	formatTrans32Struct_t value;
	for (uint8_t i = 0; i < 4; i++)
		value.u8_temp[i] = src[index + i];
	return value.float_temp;
}

int16_t ByteToInt16(uint8_t *src, int index) {
	formatTrans16Struct_t value;
	for (uint8_t i = 0; i < 2; i++)
		value.u8_temp[i] = src[index + i];
	return value.s16_temp;
}

uint16_t ByteToUint16(uint8_t *src, int index) {
	formatTrans16Struct_t value;
	for (uint8_t i = 0; i < 2; i++)
		value.u8_temp[i] = src[index + i];
	return value.u16_temp;
}


//can���ͺ���
uint8_t canTransferPack(CAN_TypeDef *CANx,uint32_t ID_CAN, uint8_t *array) {
    uint8_t mbox;
    uint16_t i = 0;     
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
    
    for(uint8_t i = 0; i < 8; i++) {
        txMessage.Data[i] = (uint8_t)array[i];
    }
    
    mbox = CAN_Transmit(CANx, &txMessage);   
	
	//�ȴ����ͽ���
    while(CAN_TransmitStatus(CANx, mbox) == CAN_TxStatus_Failed) {
        i++;	
		if(i >= 0xFFF)
			return 1;
	}
    return 0;
}

//��ȡ�ض�λ������
int8_t imuReadConfigRegister(uint16_t reg) {
    uint8_t group = (uint8_t)((reg & 0xF00) >> 8);
    uint8_t regBit = (uint8_t)(reg & 0xFF);
    uint8_t shift = 0;
    //ѭ���ҵ���С����Чλ������¼λ�ƴ�������cmd�������0
    if(regBit > 0) {
        for(; shift < 8 ; shift ++) {
            //�Ҳ�û����λ��1�������λ
            if(regBit & 0x01) {
                break;
            }
            regBit = regBit >> 1;
        }
    }
    else {
        return -1;
    }
    //��cmdֵ��λ
    regBit = (uint8_t)(reg & 0xFF);
    return (imuBroadcast.config.setting[group] & regBit) >> shift;
}

//д���ض�λ������
int8_t imuWirteConfigRegister(imuConfigStruct_t *config, uint16_t reg, uint8_t data) {
    uint8_t group = (uint8_t)((reg & 0xF00) >> 8);
    uint8_t regBit = (uint8_t)(reg & 0xFF);
    uint8_t shift = 0;
    //ѭ���ҵ���С����Чλ����¼λ�ƴ�������cmd�������0
    if(regBit > 0) {
        for(; shift < 8 ; shift ++) {
            //�Ҳ�û����λ��1�������λ
            if(regBit & 0x01) {
                break;
            }
            regBit = regBit >> 1;
        }
    }
    else {
        return -1;
    }
    //��cmdֵ��λ
    regBit = (uint8_t)(reg & 0xFF);
    //��ָ��λ���㲢��ֵ
    uint8_t value = config->setting[group] & (~regBit);
    value |= (data << shift);
    config->setting[group] = value;
    return 0;
}

//�����ֽڵ�����ת�����ֽڵ�λ
void BytesToMultipleByte(uint8_t *intput, uint8_t length, void *output, uint8_t *add) {
    formatTrans32Struct_t *data_32bit;
    formatTrans16Struct_t *data_16bit;
    if(length == 4) {
        data_32bit = output;
        for(uint8_t i = 0; i < 4; i++) {
            data_32bit->u8_temp[i] = intput[(*add)++];
        }
    }
    else if(length == 2) {
        data_16bit = output;
        for(uint8_t i = 0; i < 2; i++) {
            data_16bit->u8_temp[i] = intput[(*add)++];
        }
    }
}

//��USART��ȡһ��������Ϣ
void imuUsartGetOnceParameter(uint8_t *array) {
    uint8_t index_ptr = 0;
    array[index_ptr++] = 0xAD;
	array[index_ptr++] = 0x01;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(UART7, array, (index_ptr + 2));
}

//USART����д��1���Ĵ�����������Ϣ
void imuUsartSetting(uint8_t *array, uint8_t reg, uint8_t data) {
    uint8_t index_ptr = 0;
    array[index_ptr++] = 0xAD;
	array[index_ptr++] = reg;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    array[index_ptr++] = data;
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(UART7, array, (index_ptr + 2));
}

//USARTд��ȫ���Ĵ�����������Ϣ
void imuUsartAllSetting(uint8_t *array, uint8_t *data) {
    uint8_t index_ptr = 0;
    array[index_ptr++] = 0xAD;
	array[index_ptr++] = 0xBF;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    for(uint8_t j = 0; j < 8; j++) {
        array[index_ptr++] = data[j];
    }
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(UART7, array, (index_ptr + 2));
}

//USART���������ݽ��պ���
static void imuUsartSensorReceive(uint8_t *src) {
    uint8_t ptr = 7;
	if (0x01 == (src[2] & 0x01)) {
		uint8_t cellLength = (uint8_t)powf(2, (src[ptr] & 0x03));
		uint8_t structLength = (src[ptr] & 0x3C) >> 2;
		bool type = (src[ptr++] & 0xC0) >> 6;
		if (cellLength == 4) {
			for (uint8_t i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				imuBroadcast.gyro[i] = ByteToFloat(src, ptr);
			}
		}
		else if (cellLength == 2) {
			for (uint8_t i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				if (type) {
					imuBroadcast.gyro[i] = ByteToInt16(src, ptr) * 0.001f;
				}
				else {
					imuBroadcast.gyro[i] = ByteToInt16(src, ptr) * 0.1f;
				}
			}
		}
	}
	if (0x02 == (src[2] & 0x02)) {
		uint8_t cellLength = (uint8_t)powf(2, (src[ptr] & 0x03));
		uint8_t structLength = (src[ptr] & 0x3C) >> 2;
		bool type = (src[ptr++] & 0xC0) >> 6;
		if (cellLength == 4) {
			for (uint8_t i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				imuBroadcast.accel[i] = ByteToFloat(src, ptr);
			}
		}
		else if (cellLength == 2) {
			for (uint8_t i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				imuBroadcast.accel[i] = ByteToInt16(src, ptr) * 0.01f;
			}
		}
	}
	if (0x04 == (src[2] & 0x02)) {
		uint8_t cellLength = (uint8_t)powf(2, (src[ptr] & 0x03));
		uint8_t structLength = (src[ptr++] & 0x3C) >> 2;
		if (cellLength == 4) {
			for (uint8_t i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				imuBroadcast.mag[i] = ByteToFloat(src, ptr);
			}
		}
		else if (cellLength == 2) {
			for (uint8_t i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				imuBroadcast.mag[i] = ByteToInt16(src, ptr) * 0.01f;
			}
		}
	}
	if (0x08 == (src[2] & 0x08)) {
		uint8_t cellLength = (uint8_t)powf(2, (src[ptr] & 0x03));
		uint8_t structLength = (src[ptr++] & 0x3C) >> 2;
		if (cellLength == 4) {
			imuBroadcast.presure = ByteToFloat(src, ptr);
		}
	}
}

//USART��̬���ݽ��պ���
static void imuUsartAttitudeReceive(uint8_t *src) {
    uint8_t ptr = 7;
	uint8_t cellLength = powf(2, (src[ptr] & 0x03));
	uint8_t structLength = (src[ptr] & 0x3C) >> 2;
	bool type = (src[ptr++] & 0xC0) >> 6;
	if (type) {
		if (cellLength == 4) {
			for (int i = 0; i < MAX(structLength, 4); i++, ptr += cellLength) {
				imuBroadcast.q[i] = ByteToFloat(src, ptr);
			}
		}
		else if (cellLength == 2) {
			for (int i = 0; i < MAX(structLength, 4); i++, ptr += cellLength) {
				imuBroadcast.q[i] = ByteToInt16(src, ptr) * 0.0001f;
			}
		}
	}
	else {
		if (cellLength == 4) {
			for (int i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				imuBroadcast.euler[i] = ByteToFloat(src, ptr);
			}
		}
		else if (cellLength == 2) {
			for (int i = 0; i < MAX(structLength, 3); i++, ptr += cellLength) {
				imuBroadcast.euler[i] = ByteToInt16(src, ptr) * 0.02f;
			}
		}
	}
}

//USART�������ݽ��պ���
static void imuUsartSettingReceive(uint8_t *src) {
    uint8_t ptr = 7;
    uint8_t cellLength = powf(2, (src[ptr] & 0x03));
    uint8_t structLength = (src[ptr] & 0x3C) >> 2;
    imuBroadcast.config.configGroup = (src[ptr++] & 0xC0) >> 6;
    if (cellLength * structLength == 24) {
        for(uint8_t j = 0; j < cellLength * structLength; j++) {
            *(imuBroadcast.config.setting + j) = src[ptr + j];
        }
    }
}

//USARTת�����ݻ��溯��
static void imuUsartForwardReceive(uint8_t *src) {
    buffefLoopStruct_t *forwardContent;
    uint8_t length = src[3] - 9;
    if((src[2] & 0xF0) == 0x40) {
        //CANת����usart
        forwardContent = &imuBroadcast.forwardContent[CAN_TRANS_USART];
    }
    else {
        //VCPת����usart
        forwardContent = &imuBroadcast.forwardContent[VCP_TRANS_USART];
    }
    if(forwardContent != NULL) {
        for(uint8_t i = 0; i < length; i++) {
            forwardContent->buffer[(uint8_t)(forwardContent->tail + i)] = src[7 + i];
        }
        forwardContent->tail += length;
    }
}

//���ڴ����ж��еĽ��պ���
void imuUsartIspFunction(uint8_t *array, uint16_t len) {
    for(uint16_t i = 0 ; i < len; i++){
        imuBroadcast.bufferLoop.buffer[(uint8_t)(imuBroadcast.bufferLoop.tail + i)] = array[i];
    }
    imuBroadcast.bufferLoop.tail += len;
}

//USART���պ���
void imuUsartReceive(void) { 
    buffefLoopStruct_t *bufferLoop = &imuBroadcast.bufferLoop;
    while((uint8_t)(bufferLoop->tail - bufferLoop->header) > 7) {
        //�����ǰ����ͷ��ֵ��Ϊ0x3F����ǰѰ�ҵ�0x3F
		while(((uint8_t)(bufferLoop->tail - bufferLoop->header) > 7) && (bufferLoop->buffer[bufferLoop->header] != 0x3F)) {
			digitalIncreasing(&bufferLoop->header);
		}
        if((uint8_t)(bufferLoop->tail - bufferLoop->header) > 7) {
            //��У��CRC8�Ƿ�ͨ��
            uint8_t *crc8Check = malloc(7 * sizeof(uint8_t));
            for (uint8_t i = 0; i < 7; i++) {
                crc8Check[i] = bufferLoop->buffer[(uint8_t)(bufferLoop->header + i)];
            }
            //CRC8У�飬��ͨ��ֱ��������
            if (Verify_CRC8_Check_Sum(crc8Check, 7)) {
                uint8_t bufferLength = bufferLoop->buffer[(uint8_t)(bufferLoop->header + 3)];
                //ʣ�೤�ȱ����㹻ȡ�굱ǰ��,��������ֱ�ӽ�����ǰѭ��
                if ((uint8_t)(bufferLoop->tail - bufferLoop->header) < bufferLength){
                    break;
				}
                //������ʱ�����ڴ�
                uint8_t *bufferDecode = malloc(bufferLength * sizeof(uint8_t));
                //��������ȡ������
                for(uint8_t i = 0; i < bufferLength; i++) {
                    bufferDecode[i] = bufferLoop->buffer[(uint8_t)(bufferLoop->header + i)];
                }
                //16λCRCУ��
                if(Verify_CRC16_Check_Sum(bufferDecode, bufferLength)) {
                    //У��ͨ������ϳ���
                    bufferLoop->header += bufferLength;
                    switch (bufferDecode[2] & 0xF0) {
                    //����������
                    case 0x00:
                        imuUsartSensorReceive(bufferDecode);
                        break;
                    //������Ϣ����
                    case 0x10:
                        imuUsartSettingReceive(bufferDecode);
                        break;
                    //��̬����
                    case 0x20:
                        imuUsartAttitudeReceive(bufferDecode);
                        break;
                    //ת������
                    case 0x40:
                    case 0x80:
                        imuUsartForwardReceive(bufferDecode);
                        break;
                    }
                }
                else {
                    digitalIncreasing(&bufferLoop->header);
				}
                free(bufferDecode);
            }
            else {
                digitalIncreasing(&bufferLoop->header);
            }
            free(crc8Check);
        }
        
    }
}
 
//��CANָ����ַ��ȡһ��������Ϣ
void imuCanGetOnceParameter(CAN_TypeDef *CANx) {
    uint8_t array[8] = {0};
    array[0] = 0x01;
    canTransferPack(CANx,SETTING_ID + 1, array);
}

//CAN���պ���
void imuCanReceive(CanRxMsg *can_rx_msg) {
    switch(can_rx_msg->StdId) {
        case BRAODCAST_RECEIVE_ID : {
            formatTrans16Struct_t gyro_16bit[3];
            for(uint8_t i = 0; i < 3; i++) {
                for(uint8_t j = 0; j < 2; j++) {
                    gyro_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                }
                if(imuReadConfigRegister(IMU_GYRO_TYPE_RW)) {
                    imuBroadcast.gyro[i] = (float)gyro_16bit[i].s16_temp * 0.001f;
                }
                else {
                    imuBroadcast.gyro[i] = (float)gyro_16bit[i].s16_temp * 0.1f;
                }
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 1) : {
            formatTrans16Struct_t accel_16bit[3];
            for(uint8_t i = 0; i < 3; i++) {
                for(uint8_t j = 0; j < 2; j++) {
                    accel_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                }
                imuBroadcast.accel[i] = (float)accel_16bit[i].s16_temp * 0.01f;
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 2) : {
            formatTrans16Struct_t mag_16bit[3];
            for(uint8_t i = 0; i < 3; i++) {
                for(uint8_t j = 0; j < 2; j++) {
                    mag_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                }
                imuBroadcast.mag[i] = (float)mag_16bit[i].s16_temp * 0.01f;
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 3) : {
            formatTrans32Struct_t pres_32bit;
            for(uint8_t j = 0; j < 4; j++) {
                pres_32bit.u8_temp[j] = can_rx_msg->Data[j];
            }
            imuBroadcast.presure = pres_32bit.float_temp;
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 4) : {
            if(imuReadConfigRegister(IMU_ATTITUDE_TYPE_RW)) {
                formatTrans16Struct_t q_16bit[4];
                for(uint8_t i = 0; i < 4; i++) {
                    for(uint8_t j = 0; j < 2; j++) {
                        q_16bit[i].u8_temp[j] = can_rx_msg->Data[i * 2 + j];
                    }
                    imuBroadcast.q[i] = (float)q_16bit[i].s16_temp * 0.0001f;
                }
            }
            else {
                formatTrans32Struct_t euler_32bit[2];
                for(uint8_t i = 0; i < 2; i++) {
                    for(uint8_t j = 0; j < 4; j++) {
                        euler_32bit[i].u8_temp[j] = can_rx_msg->Data[i * 4 + j];
                    }
                    imuBroadcast.euler[i] = euler_32bit[i].float_temp;
                }
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 5) : {
            formatTrans32Struct_t euler_32bit;
            formatTrans16Struct_t CNTR_16bit;
            if(imuReadConfigRegister(IMU_ATTITUDE_TYPE_RW)) {
                for(uint8_t j = 0; j < 2; j++) {
                    CNTR_16bit.u8_temp[j] = can_rx_msg->Data[j];
                }
                imuBroadcast.CNTR = CNTR_16bit.u16_temp;
            }
            else {
                for(uint8_t j = 0; j < 4; j++) {
                    euler_32bit.u8_temp[j] = can_rx_msg->Data[j];
                }
                imuBroadcast.euler[2] = euler_32bit.float_temp;
                for(uint8_t j = 0; j < 2; j++) {
                    CNTR_16bit.u8_temp[j] = can_rx_msg->Data[4 + j];
                }
                imuBroadcast.CNTR = CNTR_16bit.u16_temp;
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x06) : {
            for(uint8_t j = 0; j < 8; j++) {
                imuBroadcast.config.setting[j] = can_rx_msg->Data[j];
            }
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x07) : {
            
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x08) :
        case (BRAODCAST_RECEIVE_ID + 0x09) : {
            buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[USART_TRANS_CAN];
            forwardContent->buffer[(uint8_t)(forwardContent->tail++)] = can_rx_msg->StdId - BRAODCAST_RECEIVE_ID;
            for(uint8_t i = 0 ; i < 8; i++) {
                forwardContent->buffer[(uint8_t)(forwardContent->tail + i)] = can_rx_msg->Data[i];
            }
            forwardContent->tail += 8;
            break;
        }
        case (BRAODCAST_RECEIVE_ID + 0x0A) :
        case (BRAODCAST_RECEIVE_ID + 0x0B) : {
            buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[VCP_TRANS_CAN];
            forwardContent->buffer[(uint8_t)(forwardContent->tail++)] = can_rx_msg->StdId - BRAODCAST_RECEIVE_ID;
            for(uint8_t i = 0 ; i < 8; i++) {
                forwardContent->buffer[(uint8_t)(forwardContent->tail + i)] = can_rx_msg->Data[i];
            }
            forwardContent->tail += 8;
            break;
        }
    }
}

//���ձ��ĳ�ʼ��
void imuBroadcastInit(void) {
    memset(&imuBroadcast, 0, sizeof(imuBroadcastStruct_t));
    imuConfigStruct_t writeConfig;
    memset(&writeConfig, 0, sizeof(imuConfigStruct_t));
    //���ٶȲ�����̬�ں�
    imuWirteConfigRegister(&writeConfig, IMU_ACCEL_FUSION_RW, ENABLE);
	//YAW���������
    imuWirteConfigRegister(&writeConfig, IMU_YAW_CONtTINUOUS_RW, ENABLE);
//	//���ڲ�����115200
	imuWirteConfigRegister(&writeConfig, IMU_USART_BAUDRATE_RW, 3);
//    //����USART����
    imuWirteConfigRegister(&writeConfig, IMU_UASRT_BROADCAST_RW, ENABLE);
//    //������̬�������
    imuWirteConfigRegister(&writeConfig, IMU_GYRO_OUTPUT_RW, ENABLE);
//    //��̬�����������Ϊŷ����
    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_TYPE_RW, 0);
//	//��̬���Ƶ��500Hz
	imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_ODR_RW, 0);
//    //��̬����32λ���
    imuWirteConfigRegister(&writeConfig, IMU_ATTITUDE_LENGTH_RW, OUTPUT_32BIT);
//    //������Ϊicm42605
    imuWirteConfigRegister(&writeConfig, IMU_IMU_TYPE_RW, IMU_TYPE_ICM42605);
	
	//USART�������õ�ģ��
	imuUsartAllSetting(Array_UART7_TX,writeConfig.setting);
	imuUsartGetOnceParameter(Array_UART7_TX);
//    //CAN�������õ�ģ��
//    canTransferPack(CAN1,SETTING_ID, writeConfig.setting);
	
}


//��ȡת����Ϣ�ĳ���
uint8_t getForwardReceiveLength(FORWARD_ENUM group) {
    buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[group];
    return (uint8_t)(forwardContent->tail - forwardContent->header);
}

//��ȡת����Ϣ����
bool getDataFromForward(uint8_t *dst, FORWARD_ENUM group) {
    buffefLoopStruct_t *forwardContent = &imuBroadcast.forwardContent[group];
    if(getForwardReceiveLength(group) > 0) {
        uint8_t length = getForwardReceiveLength(group);
        for(uint8_t i = 0; i < length; i ++) {
            dst[i] = forwardContent->buffer[forwardContent->header++];
        }
        return true;
    }
    else {
        return false;
    }
}

void imuUsartForwardTest(uint8_t *array) {
    uint8_t index_ptr = 0;
    array[index_ptr++] = 0xAD;
	array[index_ptr++] = 0x03;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
    
    for(uint8_t i = 0; i < 16; i++) {
        array[index_ptr++] = i + 1;
    }
    
    //��װ����
	array[2] = index_ptr + 2;
    //��װУ��λ
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
    //��������
	BSP_USART_DMA_SendData(USART2, array, (index_ptr + 2));
}


/*
***************************************************
��������Driver_DTU_Init
���ܣ�����������ʼ��
��ڲ�����	I can understand it at one glance.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
static void Driver_DTU_Init(USART_TypeDef *USARTx,\
							BSP_GPIOSource_TypeDef *USART_RX,\
                            BSP_GPIOSource_TypeDef *USART_TX,\
							uint8_t PreemptionPriority,uint8_t SubPriority){
	BSP_USART_TypeDef WIRELESS_USART;
	WIRELESS_USART.USARTx = USARTx;
	WIRELESS_USART.USART_RX = USART_RX;
	WIRELESS_USART.USART_TX = USART_TX;
	//������Ϊ500000
	WIRELESS_USART.USART_InitStructure.USART_BaudRate = 460800;	
	//�ֳ�Ϊ8λ���ݸ�ʽ
	WIRELESS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//һ��ֹͣλ
	WIRELESS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//��У��λ
	WIRELESS_USART.USART_InitStructure.USART_Parity = USART_Parity_No;
	//����/����ģʽ
	WIRELESS_USART.USART_InitStructure.USART_Mode =USART_Mode_Tx | USART_Mode_Rx;
	//��Ӳ������������
	WIRELESS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,		
	
	BSP_USART_Init(&WIRELESS_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&WIRELESS_USART);																				
}


/****************************************************
��������imuSCInit
���ܣ����������Ǵ��ڵ��ó�ʼ��
��ڲ�������.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
****************************************************/
static void imuSCInit(void){
	Driver_DTU_Init(IMUSC_USARTX,\
	IMUSC_USARTX_RX_PIN,\
    IMUSC_USARTX_TX_PIN,\
	IMUSC_USART_PreemptionPriority,\
	IMUSC_USART_SubPriority);
}



