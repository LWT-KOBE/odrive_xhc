#ifndef SPL06_01_H
#define SPL06_01_H

#include "BSP.h"
#include "std_lib.h"
#include "util.h"
#include "d_imu.h"


//�Ĵ�������
#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG_REG 0x06 //��ѹ������������
#define TMP_CFG_REG 0x07 //�¶Ȳ����ٶ�����
#define MEAS_CFG_REG 0x08 //���������봫��������
#define CFG_REG 0x09 //�ж�/FIFO/SPI����������
#define INT_STS_REG 0X0A //�ж�״̬��־λ
#define FIFO_STS_REG 0X0B //FIFO״̬
#define RESET_REG 0X0C
#define ID_REG 0x0D
#define COEF_REG 0x10


#define PRESSURE_RATE_1_TIMES 0 //������ 
#define PRESSURE_RATE_2_TIMES 1
#define PRESSURE_RATE_4_TIMES 2
#define PRESSURE_RATE_8_TIMES 3
#define PRESSURE_RATE_16_TIMES 4
#define PRESSURE_RATE_32_TIMES 5
#define PRESSURE_RATE_64_TIMES 6
#define PRESSURE_RATE_128_TIMES 7

#define TEMPERATURE_RATE_1_TIMES 0 //������
#define TEMPERATURE_RATE_2_TIMES 1
#define TEMPERATURE_RATE_4_TIMES 2
#define TEMPERATURE_RATE_8_TIMES 3
#define TEMPERATURE_RATE_16_TIMES 4
#define TEMPERATURE_RATE_32_TIMES 5
#define TEMPERATURE_RATE_64_TIMES 6
#define TEMPERATURE_RATE_128_TIMES 7
#define TEMPERATURE_RATE_TMP_EXT_INTERNAL 0  //���ɵ�·�ϵ��¶ȼ�
#define TEMPERATURE_RATE_TMP_EXT_EXTERNAL 1  //������MEMS��ѹоƬ���¶ȼ�


#define MEAS_CFG_COEF_RDY 0X80 // �������ڲ�У׼ֵ�ɶ�����������
#define MEAS_CFG_SENSOR_RDY 0X40 // �������ѳ�ʼ����ɣ���������
#define MEAS_CFG_TMP_RDY 0x20 //�¶�ֵ�Ѿ�׼�����������Խ��ж�ȡ���ñ�־λ��ȡ���Զ���0
#define MEAS_CFG_PRS_RDY 0x10 //��ѹֵ�Ѿ�׼�����������Խ��ж�ȡ���ñ�־λ
#define MEAS_CFG_MEAS_CTR_STANDBY 0 //ģʽ���� ����ģʽ
#define MEAS_CFG_MEAS_CTR_COMMAND_PRS 0x01 //ģʽ���� ����ģʽ��������ѹ�ɼ�
#define MEAS_CFG_MEAS_CTR_COMMAND_TMP 0x02 //ģʽ���� ����ģʽ�������¶Ȳɼ�
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PRS 0x05 //ģʽ���� ��̨ģʽֻ��ȡ��ѹֵ
#define MEAS_CFG_MEAS_CTR_BACKGROUND_TMP 0X06 //ģʽ���� ��̨ģʽֻ��ȡ�¶�ֵ
#define MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP 0X07 //ģʽ���� ��̨ģʽͬʱ��ȡ�¶�ֵ����ѹֵ

#define CFG_INT_LEVEL_ACTIVE_LOW 0//�жϵ͵�ƽ��Ч
#define CFG_INT_LEVEL_ACTIVE_HIGH 1//�жϸߵ�ƽ��Ч
#define CFG_INT_FIFO 0X40    //��FIFO��ʹ���ж� 
#define CFG_INT_PRS 0X20    //����ѹ�ƶ�ȡ���ʹ���ж� 
#define CFG_INT_TMP 0X10    //���¶ȶ�ȡ���ʹ���ж� 
#define CFG_T_SHIFT 0X08    //�������ݱ����ǣ����Խ�����һ�ʲɼ�
#define CFG_P_SHIFT 0X04    //�������ݱ����ǣ����Խ�����һ�ʲɼ�
#define CFG_FIF 0X02    //ʹ��FIFO
#define CFG_SPI_3_WIRE 1    //3��SPI
#define CFG_SPI_4_WIRE 0    //4��SPI

#define INT_STS_FIFO_FULL  0X04 //FIFO���ж�״̬
#define INT_STS_FIFO_TMP   0X02  //�¶Ȳ�����ɱ�־λ
#define INT_STS_FIFO_PRS  0X01  //��ѹ������ɱ�־λ

#define FIFO_STS_FULL  0X02 //FIFO��
#define FIFO_STS_EMPTY   0X01 //FIFO����

#define RESET_FIFO_FLUSH 0X80 //FIFO��0
#define RESET_SOFT 0X09//�����λ

#define PRODUCT_ID 0X10//��ƷID



#define s32 int32
#define int16 short
#define int32 int
#define uint8 unsigned char
	

#define SPL06_CS_H GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPL06_CS_L GPIO_ResetBits(GPIOA, GPIO_Pin_4)




//extern spl06Struct_t Spl06Data;


unsigned char spl0601_init(void);
void Drv_Spl0601_Read ( void );

float user_spl0601_get_presure(void);
float user_spl0601_get_temperature(void);
#endif

