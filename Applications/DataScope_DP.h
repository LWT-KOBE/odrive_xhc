/**************************************************************************
���ߣ�ƽ��С��֮�� 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
#include "stm32f4xx.h"
#include "FreeRTOS_board.h"
#include "stdbool.h"
#include "bsp.h"
#include "util.h" 
 
extern unsigned char DataScope_OutPut_Buffer[42];	   //������֡���ݻ�����
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 
void DataScope(void);//��λ����ʾ���� 
 
 /**********************************DTU��ʼ������****************************************/
//DTU���ں�
#define WIRELESS_USARTX								USART2
//DTU��������
#define WIRELESS_USARTX_RX_PIN						BSP_GPIOA3	//���ջ���������
//DTU��������
#define WIRELESS_USARTX_TX_PIN						BSP_GPIOA2	//���ջ���������
//WIRELESS_USART�ж���ռ���ȼ�
#define WIRELESS_USART_PreemptionPriority 			3					
//WIRELESS_USART�ж���Ӧ���ȼ�
#define WIRELESS_USART_SubPriority 					0	

 
 
 
#endif 



