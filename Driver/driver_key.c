#include "driver.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ�������ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void KEY_Init(void)
{
    BSP_GPIO_Init(BSP_GPIOA8,GPIO_Mode_IPU);//��������
    BSP_GPIO_Init(BSP_GPIOD12,GPIO_Mode_IPU);//��������
    BSP_GPIO_Init(BSP_GPIOC9,GPIO_Mode_IPU);//��������
    
    BSP_GPIO_EXIT_Init(BSP_GPIOA8,EXTI_Trigger_Falling,0x03,0x03); //�ж�������      
    BSP_GPIO_EXIT_Init(BSP_GPIOC9,EXTI_Trigger_Falling,0x03,0x03); //�ж�������      
    BSP_GPIO_EXIT_Init(BSP_GPIOD12,EXTI_Trigger_Falling,0x03,0x03); //�ж�������      

} 

