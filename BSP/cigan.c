#include "cigan.h" 


void cigan_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOAʱ��

	//GPIOB4��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO

	GPIO_SetBits(GPIOB,GPIO_Pin_4);//
}


void EXTI_Configuration(void)
{
        //�����ⲿ�ж�
        EXTI_InitTypeDef EXTI_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
        
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //ʹ��SYSCFGʱ��

    // �����ж�ԴΪGPIOA��Pin0
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

    
    EXTI_InitStructure.EXTI_Line = EXTI_Line4; //ѡ���ⲿ�ж���·0
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //����Ϊ�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //���ô�����ʽΪ�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; //ʹ���ⲿ�ж���·
    EXTI_Init(&EXTI_InitStructure);

    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; //ѡ���ж�ͨ��ΪEXTI0_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
}

