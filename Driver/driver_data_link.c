#include "driver_data_link.h"
#include "driver_ADIS16470.h"

/*--------------------------------------------- 
-----------------------------------------------
//		���棺�����������򣬹��������ش�				   //
//			���ض�����������ɿ��Ը����⣬		  		 //
//	���ಿ�ָ��ģ����빫����������ȡ�ø���Ȩ��  // 
-----------------------------------------------
---------------------------------------------*/

void driver_dataLinkInit(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,
												u32 baudRate,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef DATA_LINK_USART;
	DATA_LINK_USART.USARTx = USARTx;
	DATA_LINK_USART.USART_RX = USART_RX;
	DATA_LINK_USART.USART_TX = USART_TX;
	DATA_LINK_USART.USART_InitStructure.USART_BaudRate = baudRate;
	DATA_LINK_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	DATA_LINK_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	DATA_LINK_USART.USART_InitStructure.USART_Parity = USART_Parity_No;
	DATA_LINK_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	DATA_LINK_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	BSP_USART_Init(&DATA_LINK_USART,PreemptionPriority,SubPriority);
  BSP_USART_RX_DMA_Init(&DATA_LINK_USART);
	BSP_USART_TX_DMA_Init(&DATA_LINK_USART);
}
