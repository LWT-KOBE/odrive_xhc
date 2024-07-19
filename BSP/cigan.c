#include "cigan.h" 


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

    // �����ж�ԴΪGPIOD��Pin2
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

    
    EXTI_InitStructure.EXTI_Line = EXTI_Line2; //ѡ���ⲿ�ж���·2
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //����Ϊ�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //���ô�����ʽΪ�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; //ʹ���ⲿ�ж���·
    EXTI_Init(&EXTI_InitStructure);

    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; //ѡ���ж�ͨ��ΪEXTI0_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //��ռ���ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //�����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
}

