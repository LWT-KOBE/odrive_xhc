#ifndef __IMU_SC_H
#define __IMU_SC_H
#include "stm32f4xx.h"
#include "util.h"
#include <stdbool.h>

#define SETTING_ID  0x100
#define BRAODCAST_RECEIVE_ID    0x110

/**********************************DTU��ʼ������****************************************/
//DTU���ں�
#define IMUSC_USARTX								UART7
//DTU��������
#define IMUSC_USARTX_RX_PIN						BSP_GPIOE8	
//DTU��������
#define IMUSC_USARTX_TX_PIN						BSP_GPIOE7	
//WIRELESS_USART�ж���ռ���ȼ�
#define IMUSC_USART_PreemptionPriority 			3					
//WIRELESS_USART�ж���Ӧ���ȼ�
#define IMUSC_USART_SubPriority 					0	



//0xC0
//���ٶȲ�����̬�ں�
#define IMU_ACCEL_FUSION_RW 0x0001
//�����Ʋ�����̬�ں�
#define IMU_MAG_FUSION_RW   0x0002
//YAW���������
#define IMU_YAW_CONtTINUOUS_RW  0x0004
//���ڱ������
#define IMU_UASRT_BROADCAST_RW  0x0008
//CAN�������
#define IMU_CAN_BROADCAST_RW    0x0010
//VCP�������
#define IMU_VCP_BROADCAST_RW    0x0020
//���ڲ�����
#define IMU_USART_BAUDRATE_RW   0x00C0

//0xC1
//���ٶ����
#define IMU_GYRO_OUTPUT_RW  0x0101
//���ٶ����
#define IMU_ACCEL_OUTPUT_RW 0x0102
//���������
#define IMU_MAG_OUTPUT_RW   0x0104
//��ѹ�����
#define IMU_PRES_OUTPUT_RW  0x0108
//��̬���
#define IMU_TEMP_OUTPUT_RW  0x0110
#define IMU_ATTITUDE_OUTPUT_RW  0x0120
#define IMU_TIMESTAMP_OUTPUT_RW 0x0140

//0xC2
//���ٶ��������
#define IMU_GYRO_TYPE_RW    	0x0201
//���ٶ��������
#define IMU_ACCEL_TYPE_RW   	0x0202
//��̬�������
#define IMU_ATTITUDE_TYPE_RW    0x0204
//�߶��������
#define IMU_HEIGHT_TYPE_RW  	0x0208
//���ٶ����λ��
#define IMU_ACCEL_LENGTH_RW 	0x0210
//���ٶ����λ��
#define IMU_GYRO_LENGTH_RW  	0x0220
//���������λ��
#define IMU_MAG_LENGTH_RW   	0x0240
//��̬���λ��
#define IMU_ATTITUDE_LENGTH_RW  0x0280

//0xC3
//IMU����
#define IMU_IMU_TYPE_RW 0x030F
//VCP��CAN����
#define IMU_VCP_FORWARD_CAN_RW  0x0310
//VCP�ʹ��ڻ���
#define IMU_VCP_FORWARD_USART_RW    0x0320
//���ں�CAN����
#define IMU_USART_FORWARD_CAN_RW    0x0340

//0xC4
//���ٶ����Ƶ��
#define IMU_GYRO_ODR_RW 0x0407
//���ٶ����Ƶ��
#define IMU_ACCEL_ODR_RW    0x0438

//0xC5
//��̬���Ƶ��
#define IMU_ATTITUDE_ODR_RW 0x0507
//���������Ƶ��
#define IMU_MAG_ODR_RW  0x0518
//��ѹ�����Ƶ��
#define IMU_PRES_ODR_RW 0x0560

//0xC6
//���ٶ�����
#define IMU_GYRO_FSR_RW 0x060F
//���ٶ�����
#define IMU_ACCEL_FSR_RW    0x06F0

//0xC7
//�����λ
#define IMU_SOFTWARE_RESET_W    0x0701
//�ָ�Ĭ��ֵ
#define IMU_RESUME_TO_DEFAULT_W 0x0702
//��������
#define IMU_SAVE_CONFIG_W   0x0704
//�Ƿ���¹̼�
#define IMU_APP_UPDATE_W    0x0708
//������У׼
#define IMU_GYRO_CALI_W 0x0710
//���ٶȼ�У׼
#define IMU_ACCEL_CALI_W    0x0720
//������У׼
#define IMU_MAG_CALI_W  0x0740

//���λ��
enum {
    OUTPUT_16BIT = 0,
    OUTPUT_32BIT,
};

//IMU����
enum {
    IMU_TYPE_ADIS16470 = 0,
    IMU_TYPE_ICM20602,
    IMU_TYPE_ICM42605,
    IMU_TYPE_LSM6DSRX,
    IMU_TYPE_BMI088,
};

//���ڻ�������
typedef struct {
	uint8_t header;
	uint8_t	tail;
	uint8_t buffer[256];
} buffefLoopStruct_t;

//IMU����
typedef struct {
	//0xC0 ~ 0xC7
    uint8_t setting[8];
    float installCorrect[3];
    uint16_t receiveID;
    uint16_t broadcastID;
    uint8_t configGroup;
    float accelBias[3];
    float magBias[3];
    float gyroBias[3];
} imuConfigStruct_t;

//���ͷ�ʽ
typedef enum {
    CAN_TRANS_USART = 0,
    USART_TRANS_CAN,
    VCP_TRANS_CAN,
    VCP_TRANS_USART,
    TRANS_LIST
} FORWARD_ENUM;

//��ȡ��Ϣ
typedef struct {
    float gyro[3];
    float accel[3];
    float mag[3];
    float presure;
    float euler[3];
    float q[4];
    uint16_t CNTR;
    imuConfigStruct_t config;
    buffefLoopStruct_t bufferLoop;
    buffefLoopStruct_t forwardContent[TRANS_LIST];
} imuBroadcastStruct_t; 

imuBroadcastStruct_t *getScimuData(void);
uint8_t canTransferPack(CAN_TypeDef *CANx,uint32_t ID_CAN, uint8_t *array);
void imuUsartGetOnceParameter(uint8_t *array);
void imuUsartIspFunction(uint8_t *array, uint16_t len);
void imuUsartReceive(void);
void imuCanGetOnceParameter(CAN_TypeDef *CANx);
void imuCanReceive(CanRxMsg *can_rx_msg);
void imuBroadcastInit(void);
uint8_t getForwardReceiveLength(FORWARD_ENUM group);
bool getDataFromForward(uint8_t *dst, FORWARD_ENUM group);
void imuUsartForwardTest(uint8_t *array);
int8_t imuReadConfigRegister(uint16_t reg);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

#endif
