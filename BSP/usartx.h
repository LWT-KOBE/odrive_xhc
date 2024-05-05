#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
#include "JY60.h"
#define 	RECV_BUF_SIZE 	400

#define 	SEND_BUF_SIZE 	200

#define FRAME_HEADER      0X7B //Frame_header //֡ͷ
#define FRAME_TAIL        0X7D //Frame_tail   //֡β
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****���ڴ�������Ǽ��ٶȼ��������ݵĽṹ��*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2���ֽ�
	short Y_data; //2 bytes //2���ֽ�
	short Z_data; //2 bytes //2���ֽ�
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******���ڷ������ݵĽṹ��*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1���ֽ�
		short X_speed;	            //2 bytes //2���ֽ�
		short Y_speed;              //2 bytes //2���ֽ�
		short Z_speed;              //2 bytes //2���ֽ�
		short Power_Voltage;        //2 bytes //2���ֽ�
		Mpu6050_Data Accelerometer; //6 bytes //6���ֽ�
		Mpu6050_Data Gyroscope;     //6 bytes //6���ֽ�	
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1���ֽ�
		float X_speed;	            //4 bytes //4���ֽ�
		float Y_speed;              //4 bytes //4���ֽ�
		float Z_speed;              //4 bytes //4���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Control_Str;
}RECEIVE_DATA;



void data_transition(void);
void USART1_SEND(void);
void USART3_SEND(void);
void CAN_SEND(void);
void uart1_init(u32 bound);
void uart4_init(u32 bound);
void uart3_init(u32 bound);
void uart5_init(u32 bound);
int USART1_IRQHandler(void);
int UART4_IRQHandler(void);
int USART3_IRQHandler(void);
void Serial1Data(uint8_t ucData);
//float XYZ_Target_Speed_transition(u8 High,u8 Low);
float Vz_to_Akm_Angle(float Vx, float Vz);
void usart1_send(u8 data);
void usart4_send(u8 data);

void usart3_send(u8 data);
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);

void uart2_init(u32 bound);
void UART2_Put_Char(unsigned char DataToSend);
void UART2_Put_String(unsigned char *Str);

void u1_SendByte(uint8_t Byte);
void u1_SendArray(uint8_t *Array, uint16_t Length);
void u1_SendHalfWord(uint16_t ch);
#if Mec
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C,float Encoder_D);
#elif Omni
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C);
#endif


#endif

