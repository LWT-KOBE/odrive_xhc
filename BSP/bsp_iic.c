#include "bsp_iic.h"
#include "driver_ist8310.h"


unsigned char I2C_Err=0;
u32 ulTimeOut_Time ;

/*
***************************************************
��������BSP_I2C_RCC_Init
���ܣ�����I2C����ʱ��
��ڲ�����	BSP_I2Cx��I2C��
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
***************************************************
*/
void BSP_I2C_RCC_Init(I2C_TypeDef *I2Cx){
	if(I2Cx == I2C1)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	else if(I2Cx == I2C2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	else if(I2Cx == I2C3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,ENABLE);

}

/*
***************************************************
��������I2Cx_TO_GPIO_AF
���ܣ���I2C�����GPIO_AF
��ڲ�����	I2Cx��I2C��
����ֵ��GPIO_AF:���õ�I2Cģʽ
Ӧ�÷�Χ���ڲ�����
��ע��
***************************************************
*/
uint8_t I2Cx_TO_GPIO_AF(I2C_TypeDef *I2Cx){
	uint8_t GPIO_AF;
	if(I2Cx == I2C1)
		GPIO_AF = GPIO_AF_I2C1;
	else if(I2Cx == I2C2)	
		GPIO_AF = GPIO_AF_I2C2;
	else if(I2Cx == I2C3)	
		GPIO_AF = GPIO_AF_I2C3;

	return GPIO_AF;
}

void BSP_I2C_Init(BSP_I2C_TypeDef* BSP_I2Cx){
	
	I2C_InitTypeDef I2C_InitStructure; 	
	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_ClocksTypeDef   rcc_clocks;	
	
	/*************��ʼ��I2Cʱ��***************/
	BSP_I2C_RCC_Init(BSP_I2Cx->I2Cx);
	
	/*************��ʼ��I2C����***************/
	
	BSP_GPIO_RCC_Init(BSP_I2Cx->I2C_SCL);	//����GPIO�˿�ʱ��;
	BSP_GPIO_RCC_Init(BSP_I2Cx->I2C_SDA);	//����GPIO�˿�ʱ��;	
	GPIO_InitStructure.GPIO_Pin=BSP_I2Cx->I2C_SCL->Pin | BSP_I2Cx->I2C_SDA->Pin;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //�������ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;    //����Ϊ��©���
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_NOPULL;;   //����������
	GPIO_Init(BSP_I2Cx->I2C_SCL->GPIOx,&GPIO_InitStructure);		
	GPIO_Init(BSP_I2Cx->I2C_SDA->GPIOx,&GPIO_InitStructure);				
	
	/*************�������Ÿ���I2C����***************/
	GPIO_Pin_TO_PinAFConfig(BSP_I2Cx->I2C_SCL ,I2Cx_TO_GPIO_AF(BSP_I2Cx->I2Cx));	
	GPIO_Pin_TO_PinAFConfig(BSP_I2Cx->I2C_SDA,I2Cx_TO_GPIO_AF(BSP_I2Cx->I2Cx));
	
	/**************��ʼ��I2Cxģʽ**************/
	
	 I2C_DeInit(BSP_I2Cx->I2Cx); //������IIC�ĸ����Ĵ����ָ�����λ�Ժ��ֵ	 
	 I2C_InitStructure.I2C_ClockSpeed=400000;  //��׼ģʽ ʱ��Ƶ��Ϊ100KHZ
	 I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;   //ѡ��I2Cģʽ
	 I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	 I2C_InitStructure.I2C_OwnAddress1=0X00;      //��I2C���ڴ�ģʽʱ,����ĵ�ַ
	 I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	 I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	 I2C_Init(BSP_I2Cx->I2Cx, &I2C_InitStructure);	 
	 I2C_AcknowledgeConfig(BSP_I2Cx->I2Cx,ENABLE);   //�ڽ��յ�һ���ֽں󷵻�һ��Ӧ��ACK
	 
	/**************ʹ��I2Cx����*******************/	 
	 I2C_Cmd(BSP_I2Cx->I2Cx,ENABLE);  //��������IICģ��		


	/*��ʱ����*/
    RCC_GetClocksFreq(&rcc_clocks);
    ulTimeOut_Time = (rcc_clocks.SYSCLK_Frequency /10000); 

}


/*
 * BSP_I2C_WriteOneByte
 * ����  ��дһ���ֽڵ�I2C�豸�Ĵ�����
 * ����  ��REG_Address �������ݵ�IIC�豸�Ĵ����ĵ�ַ 
 *        REG_data ��д�������
		  Start_Address IIC�豸Ӳ����ַ
		  BSP_I2Cx Ӳ��i2c����
 * ���  ����
 * ����  ����
 * ����  ���ڲ�����
 */	


void BSP_I2C_WriteOneByte(BSP_I2C_TypeDef* BSP_I2Cx,uint8_t Start_Address, uint8_t REG_Address,uint8_t REG_data){
	
	u32 tmr;

    tmr = ulTimeOut_Time;
    while(I2C_GetFlagStatus(BSP_I2Cx->I2Cx, I2C_FLAG_BUSY));
    while((--tmr)&&I2C_GetFlagStatus(BSP_I2Cx->I2Cx, I2C_FLAG_BUSY));
    if(tmr==0) I2C_Err = 1;
	
	
	
	I2C_GenerateSTART(BSP_I2Cx->I2Cx,ENABLE);
	
	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_MODE_SELECT)));
    if(tmr==0) I2C_Err = 1;

	
	I2C_Send7bitAddress(BSP_I2Cx->I2Cx,Start_Address,I2C_Direction_Transmitter);

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
    if(tmr==0) I2C_Err = 1;

	I2C_SendData(BSP_I2Cx->I2Cx,REG_Address);

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
    if(tmr==0) I2C_Err = 1;

	I2C_SendData(BSP_I2Cx->I2Cx,REG_data);

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
    if(tmr==0) I2C_Err = 1;

	I2C_GenerateSTOP(BSP_I2Cx->I2Cx,ENABLE);	
	
		
	
}

/*
 * ��������BSP_I2C_ReadOneByte
 * ����  ����IIC�豸�Ĵ����ж�ȡһ���ֽ�
 * ����  ��REG_Address ��ȡ���ݵļĴ����ĵ�ַ
		  Start_Address IIC�豸Ӳ����ַ
		  BSP_I2Cx Ӳ��i2c����
 * ���  ����
 * ����  ����
 * ����  ���ڲ����� 
*/

uint8_t BSP_I2C_ReadOneByte(BSP_I2C_TypeDef* BSP_I2Cx,uint8_t Start_Address, uint8_t REG_Address){

	uint8_t REG_data;
	//uint8_t I2C_Hardware_Count=0;

    u32 tmr;

    tmr = ulTimeOut_Time;
    while((--tmr)&&I2C_GetFlagStatus(BSP_I2Cx->I2Cx, I2C_FLAG_BUSY));
    if(tmr==0) I2C_Err = 1;	
	

	while((--tmr)&&(I2C_GetFlagStatus(BSP_I2Cx->I2Cx,I2C_FLAG_BUSY)));
    if(tmr==0) I2C_Err = 1;

	I2C_GenerateSTART(BSP_I2Cx->I2Cx,ENABLE);//��ʼ�ź�

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_MODE_SELECT)));
    if(tmr==0) I2C_Err = 1;

	I2C_Send7bitAddress(BSP_I2Cx->I2Cx,Start_Address,I2C_Direction_Transmitter);//�����豸��ַ+д�ź�

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));//
    if(tmr==0) I2C_Err = 1;

	I2C_Cmd(BSP_I2Cx->I2Cx,ENABLE);

	I2C_SendData(BSP_I2Cx->I2Cx,REG_Address);//���ʹ洢��Ԫ��ַ����0��ʼ

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
    if(tmr==0) I2C_Err = 1;

	I2C_GenerateSTART(BSP_I2Cx->I2Cx,ENABLE);//��ʼ�ź�

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_MODE_SELECT)));
    if(tmr==0) I2C_Err = 1;

	I2C_Send7bitAddress(BSP_I2Cx->I2Cx,Start_Address,I2C_Direction_Receiver);//�����豸��ַ+���ź�

	while((--tmr)&&(!I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
    if(tmr==0) I2C_Err = 1;

	I2C_AcknowledgeConfig(BSP_I2Cx->I2Cx,DISABLE);

	I2C_GenerateSTOP(BSP_I2Cx->I2Cx,ENABLE);

	while((--tmr)&&(!(I2C_CheckEvent(BSP_I2Cx->I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED))));
    if(tmr==0) I2C_Err = 1;

	REG_data=I2C_ReceiveData(BSP_I2Cx->I2Cx);//�����Ĵ�������
	
	//vTaskDelay(1);

	return REG_data;	
	
}


/***************����I2C����*******************/
BSP_I2C_TypeDef BSP_I2C1 = {
	.I2Cx = I2C1,
	.I2C_SCL = BSP_GPIOB8,
	.I2C_SDA = BSP_GPIOB9,

};

BSP_I2C_TypeDef BSP_I2C2 = {
	.I2Cx = I2C2,
	.I2C_SCL = BSP_GPIOB10,
	.I2C_SDA = BSP_GPIOB11,

};

BSP_I2C_TypeDef BSP_I2C3 = {
	.I2Cx = I2C3,
	.I2C_SCL = BSP_GPIOA8,
	.I2C_SDA = BSP_GPIOC9,

};

