#include "usartx.h"
#include "Odrive.h"

uint8_t flag;
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
#define 	RECV_BUF_SIZE 	400

#define 	SEND_BUF_SIZE 	200
//����洢�����ַ���������
char g_usart1_recv_buf[RECV_BUF_SIZE] = {0};


//����洢�����ַ���������
char g_usart1_send_buf[SEND_BUF_SIZE] = {0};

//��������ַ��ļ���
int g_usart1_recv_cnt = 0;


//����洢�����ַ���������
char g_usart3_recv_buf[RECV_BUF_SIZE] = {0};


//����洢�����ַ���������
char g_usart3_send_buf[SEND_BUF_SIZE] = {0};

//��������ַ��ļ���
int g_usart3_recv_cnt = 0;

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
///�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
#endif



/***************************************************���ڳ�ʼ������************************************************************/

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
�������ܣ�����1��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void uart1_init(u32 bound)
{  	 
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //Enable the gpio clock //ʹ��GPIOʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);	
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);	 
//	
//	// ����PB6Ϊ���ù��ܣ�USART1_TX��
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//    
//    // ����PB7Ϊ���ù��ܣ�USART1_RX��
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
//	
//  //UsartNVIC configuration //UsartNVIC����
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	//Preempt priority //��ռ���ȼ�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
//	//Subpriority //�����ȼ�
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
//	//Enable the IRQ channel //IRQͨ��ʹ��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//  //Initialize the VIC register with the specified parameters 
//	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
//	NVIC_Init(&NVIC_InitStructure);	
//	
//  //USART Initialization Settings ��ʼ������
//	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
//	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //��ʼ������1
//	
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
//	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //ʹ�ܴ���1


	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��

	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1

	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1

	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����


	
}


static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0; 
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
�������ܣ�����2��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 	//Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);	//TX
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);	//RX 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	//Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
	//USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //��ʼ������2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //ʹ�ܴ���2 
}

/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
�������ܣ�����3��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);	//RX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);	//TX 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOB, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //��ʼ������3
	
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
    USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //ʹ�ܴ���3 
}

/**************************************************************************
Function: Serial port 4 initialization
Input   : none
Output  : none
�������ܣ�����4��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart4_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);		//TX
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11 ,GPIO_AF_UART4);	//RX 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure);      //Initialize serial port 2 //��ʼ������2
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(UART4, ENABLE);                     //Enable serial port 2 //ʹ�ܴ���2 
}


/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
�������ܣ�����5��ʼ��
��ڲ�����bound ������
����  ֵ����
**************************************************************************/
void uart5_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);	//RX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_UART5);	//TX 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	//Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
	//USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //��ʼ������5
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //ʹ�ܴ���5 
}


/***************************************************���ڽ����жϲ���**********************************************************/

/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
�������ܣ�����1�����ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���
//int USART1_IRQHandler(void)
//{	
//	u8 Res;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
//	{
//		Res = USART_ReceiveData(USART1);	//��ȡ���յ�������
//		Serial1Data(Res);
//		
//		if((USART_RX_STA&0x8000)==0)//����δ���
//			{
//			if(USART_RX_STA&0x4000)//���յ���0x0d
//				{
//				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
//				else USART_RX_STA|=0x8000;	//��������� 
//				}
//				else //��û�յ�0X0D
//				{	
//				if(Res==0x0d)USART_RX_STA|=0x4000;
//				else
//					{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
//					}		 
//				}
//			}
//		//��ձ�־λ
//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//	} 
//  return 0;
//}


int USART1_IRQHandler(void)
{	
	u8 Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		Res = USART_ReceiveData(USART1);	//��ȡ���յ�������
		Serial1Data(Res);
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
				else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}
		//***************������Ӳ���*************//
			USART_ClearFlag(USART1,USART_FLAG_RXNE);//���RXNE��־λ
			USART_ClearITPendingBit(USART1,USART_FLAG_RXNE);
                                                    //���RXNE�жϱ�־
		//��ձ�־λ
		//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}

		if(USART_GetFlagStatus(USART1, USART_IT_ORE) != RESET)  
                    //��Ҫ��USART_GetFlagStatus���������ORE����ж�
		{
			USART_ClearFlag(USART1,USART_FLAG_ORE);//���ORE��־λ
			USART_ReceiveData(USART1);	           //�������յ�������			
         } 
		  //****************������ӽ���****************//
  return 0;
}



//����2�жϷ�����
void USART2_IRQHandler(void)
{
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    USART_SendData(USART2, TxBuffer[TxCounter++]); 
    USART_ClearITPendingBit(USART2, USART_IT_TXE);
    if(TxCounter == count) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  }
	else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
		CopeSerial2Data((unsigned char)USART2->DR);//��������
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
	
	USART_ClearITPendingBit(USART2,USART_IT_ORE);
}


/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{	u8 res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		res = USART_ReceiveData(USART3);
		Serial3Data(res);
		USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	} 
  return 0;
	
	
	
	
}


/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
�������ܣ�����4�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART4_IRQHandler(void)
{	
	u8 Res;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{	      
		Res = USART_ReceiveData(UART4);	//��ȡ���յ�������
		//USART_SendData(USART1,Res);			//����
		//Serial4Data(Res);
		
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	}
  return 0;	
}


/**************************************************************************
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART5_IRQHandler(void)
{	
	u8 Res;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{	      
		Res = USART_ReceiveData(UART5);	//��ȡ���յ�������
		//USART_SendData(USART1,Res);			//����
		Serial5Data(Res);
		
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	}
  return 0;	
}
/***************************************************���ڷ������ݲ���**********************************************************/

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
�������ܣ�����1��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
�������ܣ�����3��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart3_send(Send_Data.buffer[i]);
	}	 
}



void u1_SendByte(uint8_t Byte)		//����һ���ֽ�����
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/*****************  ����һ��16λ�� **********************/
void u1_SendHalfWord(uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* ȡ���߰�λ */
	temp_h = (ch&0XFF00)>>8;
	/* ȡ���Ͱ�λ */
	temp_l = ch&0XFF;
	
	/* ���͸߰�λ */
	USART_SendData(USART1,temp_h);	
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	
	/* ���͵Ͱ�λ */
	USART_SendData(USART1,temp_l);	
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	
}

void u1_SendArray(uint8_t *Array, uint16_t Length)	//����һ��8λ������
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u1_SendByte(Array[i]);
	}
}

void u1_Send_HalfWordArray(uint16_t *Array, uint16_t Length)	//����һ��16λ������
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u1_SendHalfWord(Array[i]);
	}
}




void u3_SendByte(uint8_t Byte)		//����һ���ֽ�����
{
	USART_SendData(USART3, Byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void u3_SendArray(uint8_t *Array, uint16_t Length)	//����һ��8λ������
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u3_SendByte(Array[i]);
	}
}


/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 4 sends data
Input   : The data to send
Output  : none
�������ܣ�����4��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart4_send(u8 data)
{
	UART4->DR = data;
	while((UART4->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}


void UART2_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}

void UART2_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')UART2_Put_Char(0x0d);
			else if(*Str=='\n')UART2_Put_Char(0x0a);
				else UART2_Put_Char(*Str);
		Str++;
	}
}




/***************************************************���ڽ��շ��ͺ���**********************************************************/


//����1���շ��ʹ�����
void Serial1Data(uint8_t ucData){
	
	
		static unsigned char ucRxBuffer[250];
		static unsigned char ucRxCnt = 0;	
		ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
		
		//������յ��ַ����ַ�������Ķ�Ӧ��λ��
		g_usart1_recv_buf[g_usart1_recv_cnt] =ucData;
		//�����ַ��ļ���
		g_usart1_recv_cnt++;
		
		
			
	
		//�����յ����ϸ�ʽ���ַ��������߽��մﵽ�������ޣ��ͽ����ַ������ݵĴ���
		if(g_usart1_recv_cnt > RECV_BUF_SIZE-1 || g_usart1_recv_buf[g_usart1_recv_cnt-1] == '\n')
		{
			//strncmp�����������Ƚ������ַ���ǰN���ֽ��Ƿ���ͬ��strcmp�����ǱȽ������ַ����Ƿ���ͬ
			/*
				����1��2��Ҫ�Ƚϵ������ַ�����ַ
				����3��Ҫ�Ƚϵ������ַ�����ǰn���ֽ�
				����0�����ʾ�ַ���ǰN���ֽ���ͬ
			*/
			/*
						printf,scanf�����Ǳ�׼������뺯���������Զ��ӱ�׼�豸�н������ݵ���������루stdout,stdin��
						sprintf,sscanf�������ǵ���������Ѿ�ʵ���ض��������Զ������ݴ���ָ�����ڴ�ռ��н�����������루buf��
						����1��Ҫ���/����������ڴ�ռ�
						����2��Ҫ���/������ַ�����ʽ����ʽ������%d�����ͣ�%c���ַ���%s���ַ���
						����3��Ҫ��ʽ�����/����ı���
				*/
			
			
			if(strncmp(g_usart1_recv_buf, "V=\n ", 2) == 0)//�ٶȿ�������
			{
				sscanf(g_usart1_recv_buf, "V=%f\n", &OdriveData.SetVel[0].float_temp);//�ٶ��޸�
				OdriveData.SetVel[1].float_temp = -OdriveData.SetVel[0].float_temp;
				flag = 1;
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			
			if(strncmp(g_usart1_recv_buf, "P=\n ", 2) == 0)//
			{
				
				sscanf(g_usart1_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "Pos\n ", 4) == 0)//
			{
				OdriveData.AxisState[axis0] = CMD_MENU;
				OdriveData.ControlMode[0] = CONTROL_MODE_POSITION_TRAP;
	
				OdriveData.ControlModeFlag = 1;
				OdriveData.RequestedStateFlag = 1;
				//sscanf(g_usart1_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "Vel\n ", 4) == 0)//
			{
				
				OdriveData.AxisState[axis0] = CMD_MENU;
				OdriveData.ControlMode[0] = CONTROL_MODE_VELOCITY_RAMP;
	
				OdriveData.ControlModeFlag = 1;
				OdriveData.RequestedStateFlag = 1;
				//sscanf(g_usart1_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "Set\n ", 4) == 0)//
			{
				OdriveData.AxisState[0] = 27;
				//OdriveData.AxisState[axis0] = CMD_MOTOR;
				OdriveData.RequestedStateFlag = 1;
				//sscanf(g_usart1_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "Pos_gain=\n ", 9) == 0)//λ�û������޸�
			{
				sscanf(g_usart1_recv_buf, "Pos_gain=%f\n", &OdriveData.Pos_gain[0].float_temp);//�����޸�
				OdriveData.Pos_gain[1].float_temp = OdriveData.Pos_gain[0].float_temp;
				//�����޸ı�ʶ��
				flag = 1;
			}
			
			if(strncmp(g_usart1_recv_buf, "Vel_gain=\n ", 9) == 0)//�ٶȻ������޸�
			{
				sscanf(g_usart1_recv_buf, "Vel_gain=%f\n", &OdriveData.Vel_gain[0].float_temp);//�����޸�
				OdriveData.Vel_gain[1].float_temp = OdriveData.Vel_gain[0].float_temp;
				//�����޸ı�ʶ��
				flag = 1;
			}
			
			if(strncmp(g_usart1_recv_buf, "Vel_integrator_gain=\n ", 20) == 0)//�ٶȻ����������޸�
			{
				sscanf(g_usart1_recv_buf, "Vel_integrator_gain=%f\n", &OdriveData.Vel_integrator_gain[0].float_temp);//�����޸�
				OdriveData.Vel_integrator_gain[1].float_temp = OdriveData.Vel_integrator_gain[0].float_temp;
				//�����޸ı�ʶ��
				flag = 1;
			}
			
			if(strncmp(g_usart1_recv_buf, "OYY=\n ", 4) == 0)//�ٶȿ�������
			{
				sscanf(g_usart1_recv_buf, "OYY=%f\n", &Angle_Goal.target);//�ٶ��޸�
				Angle_Goal.finish = 0;
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "MOA=\n ", 4) == 0)//�ٶȿ�������
			{
				balanceData.flag = 0;
				sscanf(g_usart1_recv_buf, "MOA=%f\n", &Motor_SpeedA_Goal.target);//�ٶ��޸�
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "MOB=\n ", 4) == 0)//�ٶȿ�������
			{
				balanceData.flag = 1;
				sscanf(g_usart1_recv_buf, "MOB=%f\n", &Motor_SpeedB_Goal.target);//�ٶ��޸�
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			if(strncmp(g_usart1_recv_buf, "F=\n ", 2) == 0)//�ٶȿ�������
			{
				sscanf(g_usart1_recv_buf, "F=%d\n", &balanceData.flag);//�ٶ��޸�
			}
			
			//����ַ�������
			memset(g_usart1_recv_buf, 0, sizeof(g_usart1_recv_buf));
			
			//����ֵ���
			g_usart1_recv_cnt = 0;
		}
		
		if (ucRxBuffer[0]!=0xFC) //����ͷ���ԣ������¿�ʼѰ��0xFC����ͷ
			{
				if(ucRxBuffer[1]!=0x05){
						ucRxCnt=0;
						return;
				}
			}
		
		if (ucRxCnt<4) {return;}//���ݲ���4�����򷵻�
			else
			{
				Motor_SpeedA_Goal.target = XYZ_Target_Speed_transition(ucRxBuffer[3],ucRxBuffer[2]);
				ucRxCnt=0;//��ջ�����
			}
	
	
}


//����3���մ�����
void Serial3Data(uint8_t ucData){
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0xFC) //����ͷ���ԣ������¿�ʼѰ��0x66����ͷ
	{
		if(ucRxBuffer[1]!=0x05){
				ucRxCnt=0;
				return;
		}
	}
	if (ucRxCnt<4) {return;}//���ݲ���4�����򷵻�
	else
	{
		Motor_SpeedA_Goal.target = XYZ_Target_Speed_transition(ucRxBuffer[3],ucRxBuffer[2]);
		ucRxCnt=0;//��ջ�����
	}
	
	
	//������յ��ַ����ַ�������Ķ�Ӧ��λ��
		g_usart3_recv_buf[g_usart3_recv_cnt] =ucData;
		//�����ַ��ļ���
		g_usart3_recv_cnt++;
		
		
			
	
		//�����յ����ϸ�ʽ���ַ��������߽��մﵽ�������ޣ��ͽ����ַ������ݵĴ���
		if(g_usart3_recv_cnt > RECV_BUF_SIZE-1 || g_usart3_recv_buf[g_usart3_recv_cnt-1] == '\n')
		{
			//strncmp�����������Ƚ������ַ���ǰN���ֽ��Ƿ���ͬ��strcmp�����ǱȽ������ַ����Ƿ���ͬ
			/*
				����1��2��Ҫ�Ƚϵ������ַ�����ַ
				����3��Ҫ�Ƚϵ������ַ�����ǰn���ֽ�
				����0�����ʾ�ַ���ǰN���ֽ���ͬ
			*/
			/*
						printf,scanf�����Ǳ�׼������뺯���������Զ��ӱ�׼�豸�н������ݵ���������루stdout,stdin��
						sprintf,sscanf�������ǵ���������Ѿ�ʵ���ض��������Զ������ݴ���ָ�����ڴ�ռ��н�����������루buf��
						����1��Ҫ���/����������ڴ�ռ�
						����2��Ҫ���/������ַ�����ʽ����ʽ������%d�����ͣ�%c���ַ���%s���ַ���
						����3��Ҫ��ʽ�����/����ı���
				*/
			
			
			if(strncmp(g_usart3_recv_buf, "V=\n ", 2) == 0)//�ٶȿ�������
			{
				sscanf(g_usart3_recv_buf, "V=%f\n", &OdriveData.SetVel[0].float_temp);//�ٶ��޸�
				OdriveData.SetVel[1].float_temp = -OdriveData.SetVel[0].float_temp;
				flag = 1;
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			
			if(strncmp(g_usart3_recv_buf, "P=\n ", 2) == 0)//
			{
				
				sscanf(g_usart3_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart3_recv_buf, "Pos\n ", 4) == 0)//
			{
				OdriveData.AxisState[axis0] = CMD_MENU;
				OdriveData.ControlMode[0] = CONTROL_MODE_POSITION_TRAP;
	
				OdriveData.ControlModeFlag = 1;
				OdriveData.RequestedStateFlag = 1;
				//sscanf(g_usart1_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart3_recv_buf, "Vel\n ", 4) == 0)//
			{
				
				OdriveData.AxisState[axis0] = CMD_MENU;
				OdriveData.ControlMode[0] = CONTROL_MODE_VELOCITY_RAMP;
	
				OdriveData.ControlModeFlag = 1;
				OdriveData.RequestedStateFlag = 1;
				//sscanf(g_usart1_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart3_recv_buf, "Set\n ", 4) == 0)//
			{
				OdriveData.AxisState[0] = 27;
				//OdriveData.AxisState[axis0] = CMD_MOTOR;
				OdriveData.RequestedStateFlag = 1;
				//sscanf(g_usart1_recv_buf, "P=%f\n", &OdriveData.SetPos[0].float_temp);//�ٶ��޸�
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", A);//�ٶ������޸ĺ���ʾ
			}
			
			if(strncmp(g_usart3_recv_buf, "Pos_gain=\n ", 9) == 0)//λ�û������޸�
			{
				sscanf(g_usart3_recv_buf, "Pos_gain=%f\n", &OdriveData.Pos_gain[0].float_temp);//�����޸�
				OdriveData.Pos_gain[1].float_temp = OdriveData.Pos_gain[0].float_temp;
				//�����޸ı�ʶ��
				flag = 1;
			}
			
			if(strncmp(g_usart3_recv_buf, "Vel_gain=\n ", 9) == 0)//�ٶȻ������޸�
			{
				sscanf(g_usart3_recv_buf, "Vel_gain=%f\n", &OdriveData.Vel_gain[0].float_temp);//�����޸�
				OdriveData.Vel_gain[1].float_temp = OdriveData.Vel_gain[0].float_temp;
				//�����޸ı�ʶ��
				flag = 1;
			}
			
			if(strncmp(g_usart3_recv_buf, "Vel_integrator_gain=\n ", 20) == 0)//�ٶȻ����������޸�
			{
				sscanf(g_usart3_recv_buf, "Vel_integrator_gain=%f\n", &OdriveData.Vel_integrator_gain[0].float_temp);//�����޸�
				OdriveData.Vel_integrator_gain[1].float_temp = OdriveData.Vel_integrator_gain[0].float_temp;
				//�����޸ı�ʶ��
				flag = 1;
			}
			
			if(strncmp(g_usart3_recv_buf, "OYY=\n ", 4) == 0)//�ٶȿ�������
			{
				sscanf(g_usart3_recv_buf, "OYY=%f\n", &Angle_Goal.target);//�ٶ��޸�
				Angle_Goal.finish = 0;
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			if(strncmp(g_usart3_recv_buf, "MOA=\n ", 4) == 0)//�ٶȿ�������
			{
				balanceData.flag = 0;
				sscanf(g_usart3_recv_buf, "MOA=%f\n", &Motor_SpeedA_Goal.target);//�ٶ��޸�
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			if(strncmp(g_usart3_recv_buf, "MOB=\n ", 4) == 0)//�ٶȿ�������
			{
				balanceData.flag = 1;
				sscanf(g_usart3_recv_buf, "MOB=%f\n", &Motor_SpeedB_Goal.target);//�ٶ��޸�
				//OdriveData.Vel_gain[0].float_temp += 5;
				//printf("%f\r\n",OdriveData.SetVel[1].float_temp);
				//sprintf(g_usart1_send_buf, "�޸ĺ�Angle=%f\r\n", OdriveData.SetVel[0].float_temp);//�޸ĺ���ʾ
			}
			
			if(strncmp(g_usart3_recv_buf, "F=\n ", 2) == 0)//�ٶȿ�������
			{
				sscanf(g_usart3_recv_buf, "F=%d\n", &balanceData.flag);//�ٶ��޸�
			}
			
			//����ַ�������
			memset(g_usart3_recv_buf, 0, sizeof(g_usart3_recv_buf));
			
			//����ֵ���
			g_usart3_recv_cnt = 0;
		}
	
}


//����4���մ�����
void Serial4Data(uint8_t ucData){
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x66) //����ͷ���ԣ������¿�ʼѰ��0x66����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<65) {return;}//���ݲ���65�����򷵻�
	else
	{
		for(int i = 1;i < 65; i++){
			//buf[i - 1] = ucRxBuffer[i];
		}
		ucRxCnt=0;//��ջ�����
	}
}


//����5���մ�����
void Serial5Data(uint8_t ucData){
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if(ucRxBuffer[0] != 0xAA){//����ͷ���ԣ������¿�ʼѰ��0xAA��0x55����ͷ
		if(ucRxBuffer[1] != 0x55){
			ucRxCnt=0;
			return;
		}
	}
	if (ucRxCnt<4) {return;}//���ݲ���2�����򷵻�
	else
	{
		for(int i = 0;i < 4; i++){
			NFC.NFC_buf[i] = ucRxBuffer[i];
		}
		ucRxCnt=0;//��ջ�����
	}
	
//	static unsigned char ucRxBuffer[250];
//	static unsigned char ucRxCnt = 0;	
//	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
//	if (ucRxCnt<4) {return;}//���ݲ���8�����򷵻�
//	else
//	{
//		for(int i = 0;i < 4; i++){
//			NFC.NFC_buf[i] = ucRxBuffer[i];
//		}
//		ucRxCnt=0;//��ջ�����
//	}
}



/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
    unsigned char check_sum=0,k;

    //Validate the data to be sent
    //��Ҫ���͵����ݽ���У��
    if(Mode==1)
        for(k=0; k<Count_Number; k++)
        {
            check_sum=check_sum^Send_Data.buffer[k];
        }

    //Verify the data received
    //�Խ��յ������ݽ���У��
    if(Mode==0)
        for(k=0; k<Count_Number; k++)
        {
            check_sum=check_sum^Receive_Data.buffer[k];
        }

    //��Ҫ���͵ĳ��������ݽ���У��
    if(Mode==3)
    {
        for(k=0; k<Count_Number; k++)
        {
            //check_sum=check_sum^Distance_Data.buffer[k];
        }
    }
    return check_sum;
}




//���ݴ�����
void Data_Processing(uint8_t ucData){
	
	//������յ���������
	static u8 Count=0;
	
	//Fill the array with serial data
	//����������������
	Receive_Data.buffer[Count]=ucData;
	
	//Ensure that the first data in the array is FRAME_HEADER
	//ȷ�������һ������ΪFRAME_HEADER
	if(ucData == FRAME_HEADER||Count>0)
		Count++;
	else
		Count=0;
	
	if (Count == 11) //Verify the length of the packet //��֤���ݰ��ĳ���
	{
		Count=0; //Prepare for the serial port data to be refill into the array //Ϊ����������������������׼��
		if(Receive_Data.buffer[10] == FRAME_TAIL) //Verify the frame tail of the packet //��֤���ݰ���֡β
		{
			//Data exclusionary or bit check calculation, mode 0 is sent data check
			//�������λУ����㣬ģʽ0�Ƿ�������У��
			if(Receive_Data.buffer[9] ==Check_Sum(9,0))
			{
				//disable_robot_count=0;//��ʱ��������

				//Serial port 1 controls flag position 1, other flag position 0
				//����1���Ʊ�־λ��1��������־λ��0
				//Set_Control_Mode(_USART_Control);

				//Calculate the target speed of three axis from serial data, unit m/s
				//�Ӵ�������������Ŀ���ٶȣ� ��λm/s
//				Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
//				Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
//				Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
			}
		}
	}
		
}


/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 high,u8 low)
{
    //Data conversion intermediate variable
    //����ת�����м����
    short transition = 0;
	float out = 0;

    //����8λ�͵�8λ���ϳ�һ��16λ��short������
    //The high 8 and low 8 bits are integrated into a 16-bit short data
    transition=(int16_t)((high<<8) | low);
    
    return transition/100+(transition%100)*0.01f; //Unit conversion, cm/s->m/s //��λת��, cm/s->m/s
//	out = transition;
//	 out;
}

