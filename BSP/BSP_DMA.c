//#include "BSP_DMA.h"
//#include "BSP_DMA_Define.h"

///*
//***************************************************
//��������BSP_DMA_Init
//���ܣ�DMAx�ĸ�ͨ������
//��ڲ�����	*BSP_DMA_Streamx_chx��DMA������ͨ���ṹ��ָ��
//					par��	�����ַ
//					mar��	�洢����ַ
//					ndtr�����ݴ����� 
//����ֵ����
//Ӧ�÷�Χ���ⲿ����
//��ע��
//***************************************************
//*/
//void BSP_DMA_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr){
//	DMA_InitTypeDef  DMA_InitStructure;
//	//�õ���ǰstream������DMA2����DMA1
//	if((uint32_t)BSP_DMA_Streamx_chx->DMA_Streamx > (uint32_t)DMA2)
//		//DMA2ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); 	
//	else
//		//DMA1ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); 
//	DMA_DeInit(BSP_DMA_Streamx_chx->DMA_Streamx);
//	//�ȴ�DMA������
//	while (DMA_GetCmdStatus(BSP_DMA_Streamx_chx->DMA_Streamx) != DISABLE);
//	/* ���� DMA Stream */
//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = BSP_DMA_Streamx_chx->DMA_channel;  
//	//DMA�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = par;
//	//DMA �洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = mar;
//	//�洢��������ģʽ
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = ndtr; 
//	//���������ģʽ
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//�洢������ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//�������ݳ���:8λ
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	//�洢�����ݳ���:8λ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
//	//ʹ����ͨģʽ
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;/*ѭ��ģʽ*/
//	//�ߵ����ȼ�
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//	//�洢��ͻ�����δ���
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	//����ͻ�����δ���
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//��ʼ��DMA Streams
//	DMA_Init(BSP_DMA_Streamx_chx->DMA_Streamx, &DMA_InitStructure);
//	DMA_Cmd(BSP_DMA_Streamx_chx->DMA_Streamx,ENABLE);//����DMA����

//}

//void TestAdcDMA(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr)
//{
//	DMA_InitTypeDef DMA_InitStructure;
//	//�õ���ǰstream������DMA2����DMA1
//	if((uint32_t)BSP_DMA_Streamx_chx->DMA_Streamx > (uint32_t)DMA2)
//		//DMA2ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); 	
//	else
//		//DMA1ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); 
//	DMA_DeInit(BSP_DMA_Streamx_chx->DMA_Streamx);
//	
//	
//	
//	
//	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);/*DMA2��ʱ��ʹ��*/
//	while(DMA_GetCmdStatus(BSP_DMA_Streamx_chx->DMA_Streamx) != DISABLE);/*�ȴ�DMA��������*/


//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = BSP_DMA_Streamx_chx->DMA_channel;  
//	
//	
//	//DMA�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = par;
//	//DMA �洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = mar;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = ndtr; 
//	
//	//DMA_InitStructure.DMA_BufferSize = 1;/*���ݴ��������Ϊ1*/
//	
//	//DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&AdcValue;/*��ȡ����ַ*/
//	
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;/*��������赽�ڴ�*/
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;/*��ַ������*/
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;/*��ַ������*/
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;/*���ݳ��Ȱ���*/
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;/*���ݳ��Ȱ���*/
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;/*�����ȼ�*/
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;/*ѭ��ģʽ*/
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;/*��ֹFIFO*/
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;/*FIFO��ֵ*/
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;/*���δ���*/
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;/*���δ���*/
//	DMA_Init(BSP_DMA_Streamx_chx->DMA_Streamx,&DMA_InitStructure);/**/
//	DMA_Cmd(BSP_DMA_Streamx_chx->DMA_Streamx,ENABLE);//����DMA����
//	
//	
//	//BSP_DMA_Init(&BSP_DMA_ADC1,(uint32_t)ADC1_BASE+0x4C,(uint32_t)&AdcValue,1);
//	
//}





















///*
//***************************************************
//��������BSP_DMA_DSHOT_Init
//���ܣ�ʹ��DSHOT��DMAx�ĸ�ͨ������
//��ڲ�����	*BSP_DMA_Streamx_chx��DMA������ͨ���ṹ��ָ��
//					par��	�����ַ
//					mar��	�洢����ַ
//					ndtr�����ݴ����� 
//����ֵ����
//Ӧ�÷�Χ���ⲿ����
//��ע��
//***************************************************
//*/
//void BSP_DMA_DSHOT_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr){
//	DMA_InitTypeDef  DMA_InitStructure;
//	//�õ���ǰstream������DMA2����DMA1
//	if((uint32_t)BSP_DMA_Streamx_chx->DMA_Streamx > (uint32_t)DMA2)
//		//DMA2ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); 
//	else
//		//DMA1ʱ��ʹ�� 
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
//	DMA_DeInit(BSP_DMA_Streamx_chx->DMA_Streamx);
//	//�ȴ�DMA������ 
//	while (DMA_GetCmdStatus(BSP_DMA_Streamx_chx->DMA_Streamx) != DISABLE);
//	 /* ���� DMA Stream */
//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = BSP_DMA_Streamx_chx->DMA_channel;  
//	//DMA�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = par;
//	//DMA �洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = mar;
//	//�洢��������ģʽ
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = ndtr; 
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable; 
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	//���������ģʽ
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//�洢������ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//�������ݳ���:16λ
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		
//	//�洢�����ݳ���:16λ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;					
//	//ʹ����ͨģʽ
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
//	//�����ȼ�
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	//�洢��ͻ�����δ���
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	//����ͻ�����δ���
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//��ʼ��DMA Stream
//	DMA_Init(BSP_DMA_Streamx_chx->DMA_Streamx, &DMA_InitStructure);
//	//ʹ��DMA�жϣ����ݴ�����ϴ���	
//	DMA_ITConfig(BSP_DMA_Streamx_chx->DMA_Streamx, DMA_IT_TC, ENABLE);
//}
///*
//***************************************************
//��������BSP_DMA_SK6812_Init
//���ܣ�ʹ��SK6812������DMAx�ĸ�ͨ������
//��ڲ�����	*BSP_DMA_Streamx_chx��DMA������ͨ���ṹ��ָ��
//					par��	�����ַ
//					mar��	�洢����ַ
//					ndtr�����ݴ����� 
//����ֵ����
//Ӧ�÷�Χ���ⲿ����
//��ע��
//***************************************************
//*/
//void BSP_DMA_SK6812_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,uint32_t par,uint32_t mar,uint16_t ndtr){
//  DMA_InitTypeDef  DMA_InitStructure;
//	//�õ���ǰstream������DMA2����DMA1
//	if((uint32_t)BSP_DMA_Streamx_chx->DMA_Streamx > (uint32_t)DMA2)
//		//DMA2ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
//	else
//		//DMA1ʱ��ʹ�� 
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
//	DMA_DeInit(BSP_DMA_Streamx_chx->DMA_Streamx);
//	//�ȴ�DMA������ 
//	while (DMA_GetCmdStatus(BSP_DMA_Streamx_chx->DMA_Streamx) != DISABLE);
//	 /* ���� DMA Stream */
//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = BSP_DMA_Streamx_chx->DMA_channel;  
//	//DMA�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = par;
//	//DMA �洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = mar;
//	//�洢��������ģʽ
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = ndtr; 
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable; 
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	//���������ģʽ
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//�洢������ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//�������ݳ���:32λ
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;		
//	//�洢�����ݳ���:32λ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;					
//	//ʹ����ͨģʽ 
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	//�����ȼ�
//	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	//�洢��ͻ�����δ���
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	//����ͻ�����δ���
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//��ʼ��DMA Stream
//	DMA_Init(BSP_DMA_Streamx_chx->DMA_Streamx, &DMA_InitStructure);
//	//ʹ��DMA�жϣ����ݴ�����ϴ���
//	DMA_ITConfig(BSP_DMA_Streamx_chx->DMA_Streamx, DMA_IT_TC, ENABLE);	
//}
///*
//***************************************************
//��������BSP_DMA_TX_Init
//���ܣ�ʹ�ô��ڷ��͵�DMAx�ĸ�ͨ������
//��ڲ�����	*BSP_DMA_Streamx_chx��DMA������ͨ���ṹ��ָ��
//			*USARTx�����ں�
//			par��	�����ַ
//			ndtr�����ݴ����� 
//����ֵ����
//Ӧ�÷�Χ���ⲿ����
//��ע��
//***************************************************
//*/
//void BSP_DMA_TX_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,USART_TypeDef	*USARTx,uint32_t par,uint16_t ndtr){
//	DMA_InitTypeDef  DMA_InitStructure;
//	//�õ���ǰstream������DMA2����DMA1
//	if((uint32_t)BSP_DMA_Streamx_chx->DMA_Streamx > (uint32_t)DMA2)
//		//DMA2ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); 
//	else
//		//DMA1ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); 
//	DMA_DeInit(BSP_DMA_Streamx_chx->DMA_Streamx);
//	//�ȴ�DMA������
//	while (DMA_GetCmdStatus(BSP_DMA_Streamx_chx->DMA_Streamx) != DISABLE);
//	/* ���� DMA Stream */
//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = BSP_DMA_Streamx_chx->DMA_channel;  
//	//DMA�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = par;
//	//DMA �洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = marArrayTX(USARTx,ndtr);
//	//�洢��������ģʽ
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = 0; 
//	//���������ģʽ
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//�洢������ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//�������ݳ���:8λ
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	//�洢�����ݳ���:8λ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	//ʹ����ͨģʽ
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
//	//�е����ȼ�
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//	//�洢��ͻ�����δ���
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	//����ͻ�����δ���
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//��ʼ��DMA Stream
//	DMA_Init(BSP_DMA_Streamx_chx->DMA_Streamx, &DMA_InitStructure);
//}
///*
//***************************************************
//��������BSP_DMA_RX_Init
//���ܣ�ʹ�ô��ڽ��յ�DMAx�ĸ�ͨ������
//��ڲ�����	*BSP_DMA_Streamx_chx��DMA������ͨ���ṹ��ָ��
//			*USARTx�����ں�
//			par��	�����ַ
//			ndtr�����ݴ����� 
//����ֵ����
//Ӧ�÷�Χ���ⲿ����
//��ע��
//***************************************************
//*/
//void BSP_DMA_RX_Init(BSP_DMA_TypeDef *BSP_DMA_Streamx_chx,USART_TypeDef	*USARTx,uint32_t par,uint16_t ndtr){
//	DMA_InitTypeDef  DMA_InitStructure;
//	//�õ���ǰstream������DMA2����DMA1
//	if((uint32_t)BSP_DMA_Streamx_chx->DMA_Streamx > (uint32_t)DMA2)
//		//DMA2ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); 
//	else
//		//DMA1ʱ��ʹ��
//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE); 
//	DMA_DeInit(BSP_DMA_Streamx_chx->DMA_Streamx);
//	//�ȴ�DMA������ 
//	while (DMA_GetCmdStatus(BSP_DMA_Streamx_chx->DMA_Streamx) != DISABLE);
//	/* ���� DMA Stream */
//	//ͨ��ѡ��
//	DMA_InitStructure.DMA_Channel = BSP_DMA_Streamx_chx->DMA_channel;  
//	//DMA�����ַ
//	DMA_InitStructure.DMA_PeripheralBaseAddr = par;
//	//DMA �洢��0��ַ
//	DMA_InitStructure.DMA_Memory0BaseAddr = marArrayRX(USARTx,ndtr);
//	//���赽�洢��ģʽ
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	//���ݴ�����
//	DMA_InitStructure.DMA_BufferSize = ndtr; 
//	//���������ģʽ
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	//�洢������ģʽ
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	//�������ݳ���:8λ
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	//�洢�����ݳ���:8λ
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	//ʹ����ͨģʽ 
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	//�е����ȼ�
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//	//�洢��ͻ�����δ���
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	//����ͻ�����δ���
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	//��ʼ��DMA Stream
//	DMA_Init(BSP_DMA_Streamx_chx->DMA_Streamx, &DMA_InitStructure);
//}

///*
//DAM1����ӳ�䣺
//-----------------------------------------------------------------------------------------------------------------------------------------
//��������	|	 DMA1_Stream0	|	 DMA1_Stream1	|	 DMA1_Stream2	|	 DMA1_Stream3	|	 DMA1_Stream4	|	 DMA1_Stream5	|	 DMA1_Stream6	|	 DMA1_Stream7	|
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��0	|    SPI3_RX    |								|			SPI3_RX		|			SPI2_RX		|			SPI2_TX		|			SPI3_TX		|								|		 SPI3_TX		|
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��1	|		 I2C1_RX		|								|			TIM7_UP		|								|			TIM7_UP		|    I2C1_RX    |    I2C1_TX    |    I2C1_TX    |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��2	|		 TIM4_CH1	 	|								|  I2S3_EXT_RX  |		 TIM4_CH2	 	|  I2S2_EXT_TX  |  I2S3_EXT_TX  |    TIM4_UP    |    TIM4_CH3   |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��3	|  I2S3_EXT_RX  |	 TIM2_CH3/UP	|		 I2C3_RX		|  I2S2_EXT_RX  |		 I2C3_RX		|    TIM2_CH1   |  TIM2_CH2/CH4 |  TIM2_CH4/UP  |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��4	|		 UART5_RX		|   USART3_RX   |		 UART4_RX		|   USART3_TX   |		 UART4_TX		|   USART2_RX   |   USART2_TX   |    UART5_TX   |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��5	|		 UART8_TX		|    UART7_TX   |	 TIM3_CH4/UP	|    UART7_RX   | TIM3_CH1/TRIG |    TIM3_CH2   |    UART8_RX   |    TIM3_CH3   |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��6	|	 TIM5_CH3/UP	|	TIM5_CH4/TRIG	|		 TIM5_CH1	 	|	TIM5_CH4/TRIG	|		 TIM5_CH2	 	|								|    TIM5_UP    |								|
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��7	|								|    TIM6_UP    |		 I2C2_RX		|		 I2C2_RX		|   USART3_TX   |      DAC1     |      DAC2     |		 I2C2_TX		|
//-----------------------------------------------------------------------------------------------------------------------------------------

//DMA2����ӳ�䣺
//-----------------------------------------------------------------------------------------------------------------------------------------
//��������	|	 DMA2_Stream0	|	 DMA2_Stream1	|	 DMA2_Stream2	|	 DMA2_Stream3	|	 DMA2_Stream4	|	 DMA2_Stream5	|	 DMA2_Stream6	|	 DMA2_Stream7	|
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��0	|     ADC1      |								| TIM8_CH1/2/3  |								|     ADC1      |								| TIM1_CH1/2/3  |								|
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��1	|								|     DCMI      |      ADC2     |      ADC2     |								|    SPI6_TX    |    SPI6_RX    |      DCMI     |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��2	|		  ADC3  	 	|		  ADC3  	 	|								|		 SPI5_RX	 	|    SPI5_TX    |    CRYP_OUT   |    CRYP_IN    |     HASH_IN   |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��3	|    SPI1_RX    |								|		 SPI1_RX		|    SPI1_TX    |								|    SPI1_TX    |								|								|
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��4	|		 SPI4_RX 		|    SPI4_TX    |	  USART1_RX 	|      SDIO     |								|   USART1_RX   |      SDIO     |   USART1_TX   |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��5	|								|   USART6_RX   |	  USART6_RX 	|    SPI4_RX    |    SPI4_TX    |								|   USART6_TX   |   USART6_TX   |
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��6	|	  TIM1_TRIG 	|    TIM1_CH1   |		 TIM1_CH2	 	|	   TIM1_CH1   |TIM1C4/TRIG/COM|    TIM1_UP    |    TIM1_CH3   |								|
//-----------------------------------------------------------------------------------------------------------------------------------------
// ͨ��7	|								|    TIM8_UP    |		 TIM8_CH1  	|		 TIM8_CH2		|		 TIM8_CH3		|    SPI5_RX    |    SPI5_TX    |TIM8C4/TRIG/COM|
//-----------------------------------------------------------------------------------------------------------------------------------------

//*/
