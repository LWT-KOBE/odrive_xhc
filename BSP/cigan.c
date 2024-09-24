#include "cigan.h" 
#include "app.h"

void cigan_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIODʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOEʱ��
	
	//GPIOD2��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO

	GPIO_SetBits(GPIOB,GPIO_Pin_3);//
	GPIO_SetBits(GPIOD,GPIO_Pin_2);//
}


void EXTI_Configuration(void)
{
        //�����ⲿ�ж�
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
        
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //ʹ��SYSCFGʱ��

    // �����ж�ԴΪGPIOA��Pin6
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);//speedA
    EXTI_InitStructure.EXTI_Line = EXTI_Line6; //ѡ���ⲿ�ж���·2
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //����Ϊ�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //���ô�����ʽΪ�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; //ʹ���ⲿ�ж���·
    EXTI_Init(&EXTI_InitStructure);
	
    // �����ж�ԴΪGPIOA��Pin6
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);//speedB
    EXTI_InitStructure.EXTI_Line = EXTI_Line1; //ѡ���ⲿ�ж���·2
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //����Ϊ�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //���ô�����ʽΪ�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; //ʹ���ⲿ�ж���·
    EXTI_Init(&EXTI_InitStructure);	

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //ѡ���ж�ͨ��ΪEXTI0_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04; //�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; //ѡ���ж�ͨ��ΪEXTI0_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04; //�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
}


//////�ⲿ�ж�0������� 
void EXTI9_5_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line6)!=RESET)			//��־λ��ֵλ�������жϣ�SPEEDA
	{	
		PulseCntA++;  
		EXTI_ClearITPendingBit(EXTI_Line6); //���LINE0�ϵ��жϱ�־λ 
	}
	
}

void EXTI1_IRQHandler(void)
{

	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)			//��־λ��ֵλ�������жϣ�SPEEDA
	{	
		PulseCntB++; 
		EXTI_ClearITPendingBit(EXTI_Line1); //���LINE0�ϵ��жϱ�־λ 
	}
	
}





