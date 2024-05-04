#include "Driver_Openmv.h"
#include "application.h"



 /*OPENMV X �ᷴ������*/
int origin_x,origin_y;
int color=0,shape=0;
int Lm_1=0,Lm_2=0,Lm_3=0;
int size=0;
int conunt=0;
u8 adition_falg=0;			//�ж�λ���Ƿ��Ѱ�λ

void Driver_Openmv_Init(USART_TypeDef* USARTx, BSP_GPIOSource_TypeDef *USART_RX, BSP_GPIOSource_TypeDef *USART_TX, \
														uint32_t baudRate, uint8_t PreemptionPriority, uint8_t SubPriority){
	BSP_USART_TypeDef openmv_USART;
	openmv_USART.USARTx = USARTx;
	openmv_USART.USART_RX = USART_RX;
	openmv_USART.USART_TX = USART_TX;
	openmv_USART.USART_InitStructure.USART_BaudRate = baudRate;							
	openmv_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	openmv_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;			/*һ��ֹͣλ*/
	openmv_USART.USART_InitStructure.USART_Parity = USART_Parity_No;					/*��У��λ*/
	openmv_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	openmv_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&openmv_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&openmv_USART);	
	BSP_USART_TX_DMA_Init(&openmv_USART);	
}
														
void Openmv_Recive(uint8_t *rebuf)
{
	
	static uint16_t state=0;
	
	
	if(state==0&&rebuf[0]==0xb3)
	{
		state = 1;
	}
	 else if((state ==1)&&(rebuf[1]==0xb3))
			state = 2;
	 
	 else if((state ==2)&&rebuf[4]==0x5b)//����һ֡�������
		{
			shape = rebuf[2];
			color = rebuf[3];
			state=0;
		}
		else if((state ==2)&&rebuf[4]==0x5c)
		{	
			origin_x = rebuf[2];
			origin_y = rebuf[3];
			
			state=0;
		}
		
		else if((state ==2)&&rebuf[5]==0x5a)
		{	
			Lm_1 = rebuf[2];
			Lm_2 = rebuf[3];
			Lm_3 = rebuf[4];
						
			state=0;
		}				
				
		if(rebuf[4]!=0x5c)		//��������Ƿ��յ�������ݣ��������100���޽��գ���Ϊ�Ѱ�
		{

			conunt++;
			if(conunt>=20)
				adition_falg=0;					
		}
		else
		{	
		    adition_falg=1;	
			conunt =0;			
		}
	
}

uint8_t temp_data = 0;
uint8_t TFmini_low,TFmini_high = 0;
uint8_t flag = 0;
uint16_t distance;

//TFmini�����ഫ����
void Driver_TFmini_Init(USART_TypeDef* USARTx, BSP_GPIOSource_TypeDef *USART_RX, BSP_GPIOSource_TypeDef *USART_TX, \
														uint32_t baudRate, uint8_t PreemptionPriority, uint8_t SubPriority){
	BSP_USART_TypeDef TFmini_USART;
	TFmini_USART.USARTx = USARTx;
	TFmini_USART.USART_RX = USART_RX;
	TFmini_USART.USART_TX = USART_TX;
	TFmini_USART.USART_InitStructure.USART_BaudRate = baudRate;							
	TFmini_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	TFmini_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;			/*һ��ֹͣλ*/
	TFmini_USART.USART_InitStructure.USART_Parity = USART_Parity_No;					/*��У��λ*/
	TFmini_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	TFmini_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&TFmini_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&TFmini_USART);	
}

void TFmini_Recive(uint8_t *rebuf){
	
		if((rebuf[0] == 0x59) && (flag == 0))
		{
			flag = 1;
		}
		else if((rebuf[1] == 0x59) && (flag == 1))
		{
			flag = 2;
		}
		else if(flag == 2)
		{
			TFmini_low = rebuf[2];
			
			flag = 3;
		}
		else if(flag == 3)
		{
			TFmini_high = rebuf[3];
			
			distance = ((uint16_t)TFmini_high<<8) | ((uint16_t)TFmini_low);
			
			//printf("receive once��  %d cm \r\n",distance);
			
			flag = 0;
		}
	}


