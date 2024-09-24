#include "BSP_eeprom.h"
#include <stdlib.h>
#include <string.h>
#include "application.h"
#include "stm32f4xx.h"
void EEPROM_GPIO_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APBxPeriph_EEPROM_IO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = EEPROM_SCL_PIN;			//SCL
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	GPIO_Init(EEPROM_IO, &GPIO_InitStructure);//��ʼ��GPIO		

	GPIO_InitStructure.GPIO_Pin = EEPROM_SDA_PIN;			//SDA	OD
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	GPIO_Init(EEPROM_IO, &GPIO_InitStructure);//��ʼ��GPIO			
	
	
	
//д����	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				//WP PA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO		


//	GPIO_InitStructure.GPIO_Pin = EEPROM_WP_PIN;			//WP 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(EEPROM_IO, &GPIO_InitStructure);
//	GPIO_SetBits(EEPROM_IO, EEPROM_WP_PIN); //д����
	
}


void EEPROM_delay(void)
{
	u8 i = 255; //��������Ż��ٶ� ����������͵�5����д��
	while(i)
	{
		i--;
	}
}

bool EEPROM_Start(void)
{
	SET_EEPROM_SDA;
	SET_EEPROM_SCL;
	delay_us(3);
	if(!SDA_read)return false; //SDA��Ϊ�͵�ƽ������æ,�˳�
	RESET_EEPROM_SDA;
	delay_us(3);
	if(SDA_read) return false; //SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	RESET_EEPROM_SDA;
	delay_us(3);
	return true;
}

void EEPROM_Stop(void)
{
	RESET_EEPROM_SCL;
	delay_us(3);
	RESET_EEPROM_SDA;
	delay_us(3);
	SET_EEPROM_SCL;
	delay_us(3);
	SET_EEPROM_SDA;
	delay_us(3);
}

void EEPROM_Ack(void)
{
	RESET_EEPROM_SCL;
	delay_us(3);
	RESET_EEPROM_SDA;
	delay_us(3);
	SET_EEPROM_SCL;
	delay_us(3);
	RESET_EEPROM_SCL;
	delay_us(3);
}

void EEPROM_NoAck(void)
{
	RESET_EEPROM_SCL;
	delay_us(3);
	SET_EEPROM_SDA;
	delay_us(3);
	SET_EEPROM_SCL;
	delay_us(3);
	RESET_EEPROM_SCL;
	delay_us(3);
}

bool EEPROM_WaitAck(void)   //����Ϊ:=1��ACK,=0��ACK
{
	RESET_EEPROM_SCL;
	delay_us(3);
	SET_EEPROM_SDA;
	delay_us(3);
	SET_EEPROM_SCL;
	delay_us(3);
	if(SDA_read)
	{
		RESET_EEPROM_SCL;
		return false;
		//      return TRUE;
	}
	RESET_EEPROM_SCL;
	return true;
}

void EEPROM_SendByte(u8 SendByte) //���ݴӸ�λ����λ//
{
	u8 i = 8;
	while(i--)
	{
		RESET_EEPROM_SCL;
		delay_us(3);
		
		if(SendByte & 0x80)
			SET_EEPROM_SDA;
		else
			RESET_EEPROM_SDA;
		SendByte <<= 1;
		delay_us(3);
		SET_EEPROM_SCL;
		delay_us(3);
	}
	RESET_EEPROM_SCL;
}

u8 EEPROM_ReceiveByte(void)  //���ݴӸ�λ����λ//
{
	u8 i = 8;
	u8 ReceiveByte = 0;

	SET_EEPROM_SDA;
	while(i--)
	{
		ReceiveByte <<= 1;
		RESET_EEPROM_SCL;
		delay_us(3);
		SET_EEPROM_SCL;
		delay_us(3);
		if(SDA_read)
		{
			ReceiveByte |= 0x01;
		}
	}
	RESET_EEPROM_SCL;
	return ReceiveByte;
}

bool EEPROM_Write_Byte(u8 WriteAddr, u8 WriteData)
{
//	GPIO_ResetBits(EEPROM_IO, EEPROM_WP_PIN);
	if (!EEPROM_Start()) return false;
	EEPROM_SendByte(EEPROM_SLAVE_ADDRESS7);//����������ַ
	if (!EEPROM_WaitAck())
	{
		EEPROM_Stop();
		return false;
	}
	EEPROM_SendByte(WriteAddr);   //���ö��ڵ�ַ
	EEPROM_WaitAck();

	EEPROM_SendByte(WriteData);
	EEPROM_WaitAck();
	EEPROM_Stop();
	//ע�⣺��Ϊ����Ҫ�ȴ�EEPROMд�꣬���Բ��ò�ѯ����ʱ��ʽ(10ms)
	//Systick_Delay_1ms(10);
//	delay_ms(10);
//	GPIO_SetBits(EEPROM_IO, EEPROM_WP_PIN);
	return true;
}



bool EEPROM_Write2(u8 WriteAddr, u8 WriteData)
{
	if (!EEPROM_Start()) return false;
	EEPROM_SendByte(0x10);//����������ַ+�ε�ַ
	if (!EEPROM_WaitAck())
	{
		EEPROM_Stop();
		return false;
	}
	EEPROM_SendByte(WriteAddr);   //���ö��ڵ�ַ
	EEPROM_WaitAck();

	EEPROM_SendByte(WriteData);
	EEPROM_WaitAck();
	EEPROM_Stop();
	//ע�⣺��Ϊ����Ҫ�ȴ�EEPROMд�꣬���Բ��ò�ѯ����ʱ��ʽ(10ms)
	//Systick_Delay_1ms(10);
	return true;
}

void EepromWriteFloat(uint8_t addr_, float dat_)
{

	uint8_t buf[4] = {0, 0, 0, 0};
	uint8_t i = 0;

	memcpy(&buf[0], &dat_, 4);
	for(i = 0; i < 4; i++)
	{
		EEPROM_Write_Byte((addr_ + i), buf[i]); //д��EEPROM��1���ֽ�
		delay_ms(2);
	}

}

//����1������
u8 EEPROM_Read_Byte(u8 WriteAddr)
{
	u8 tempDat = 0;
	if (!EEPROM_Start()) return false;

	EEPROM_SendByte(EEPROM_SLAVE_ADDRESS7);//����������ַ+�ε�ַ

	if (!EEPROM_WaitAck())
	{
		EEPROM_Stop();
		return false;
	}

	EEPROM_SendByte(WriteAddr);   //���õ���ʼ��ַ
	EEPROM_WaitAck();
	EEPROM_Start();
	EEPROM_SendByte(EEPROM_SLAVE_ADDRESS7 | 0x01);
	EEPROM_WaitAck();
	tempDat = EEPROM_ReceiveByte();
	EEPROM_NoAck();

	EEPROM_Stop();
	return tempDat;
}

u16 EEPROM_Read_u16(u8 WriteAddr)
{
	return ((EEPROM_Read_Byte(WriteAddr) << 8) + (EEPROM_Read_Byte(WriteAddr + 1) & 0xff));
}

//����1������
u8 EEPROM_Read2(u8 WriteAddr)
{
	u8 tempDat = 0;


	if (!EEPROM_Start()) return false;

	EEPROM_SendByte(0x10);//����������ַ+�ε�ַ

	if (!EEPROM_WaitAck())
	{
		EEPROM_Stop();
		return false;
	}

	EEPROM_SendByte(WriteAddr);   //���õ���ʼ��ַ
	EEPROM_WaitAck();
	EEPROM_Start();
	EEPROM_SendByte(0x10 | 0x01);
	EEPROM_WaitAck();
	tempDat = EEPROM_ReceiveByte();
	EEPROM_NoAck();

	EEPROM_Stop();

	return tempDat;
}
///////////////////////////////////////////////

bool EEPROM_Write_u16(u8 WriteAddr, u16 WriteData)
{
	EEPROM_Write_Byte(WriteAddr + 1, WriteData & 0xff);
	delay_ms(5);
	EEPROM_Write_Byte(WriteAddr, WriteData >> 8);
	return true;
}

float EepromReadFloat(u8 WriteAddr)
{
	u8 datbuf[4];
	u8 i = 0;
	u8 len = 4;
	float data_ = 0;
	memset(&datbuf[0], 0, 4);//�������
	for(i = 0; i < len; i++)
	{
		datbuf[i] = EEPROM_Read_Byte(WriteAddr + i);
		delay_ms(2);
	}
	memcpy(&data_, &datbuf[0], 4);
	return data_;
}


