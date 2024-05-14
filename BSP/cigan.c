#include "cigan.h" 


void cigan_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟

	//GPIOB4初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO

	GPIO_SetBits(GPIOB,GPIO_Pin_4);//
}


void EXTI_Configuration(void)
{
        //配置外部中断
        EXTI_InitTypeDef EXTI_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
        
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //使能SYSCFG时钟

    // 配置中断源为GPIOA的Pin0
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

    
    EXTI_InitStructure.EXTI_Line = EXTI_Line4; //选择外部中断线路0
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设置触发方式为下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; //使能外部中断线路
    EXTI_Init(&EXTI_InitStructure);

    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; //选择中断通道为EXTI0_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //抢占优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; //子优先级为1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能中断通道
    NVIC_Init(&NVIC_InitStructure);
}

