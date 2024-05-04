//#include "application.h"
//#include "bsp.h"
//#include "driver.h"

///*
//***************************************************
//��������USART1_IRQHandler
//���ܣ�����1�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//*/
////DMA2-2
////void USART1_IRQHandler(void){																	
////	//�������ݵĳ���
////	uint16_t USART1_len;
////	u8 res = 0;
////	//����Ƿ��ǿ����ж�
////	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET){
////		//��USART_IT_IDLE��־
////		USART1_len = USART1->SR;
////		USART1_len = USART1->DR; 
////		//�ر�DMA
////		DMA_Cmd(DMA2_Stream2,DISABLE); 
////		//��ȡ�������ݳ���		
////		USART1_len = Length_USART1_RX_Buff - DMA2_Stream2->NDTR;
////		//��ձ�־λ		
////		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
////		while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
////		DMA2_Stream2->NDTR = Length_USART1_RX_Buff;
////		DMA_Cmd(DMA2_Stream2, ENABLE);
////		/*********�������Զ��岿��**********/           
////		if(USART1_len){
////		}


////	}
////}



////static unsigned int gresult = 0;
////static bool         fstatus = false;
////static unsigned int gdistance = 0;
////static int          gbytes    = 0;
////static unsigned char gcmd[32] = {0};
//////L1ģ�鴮���жϷ�����
////void USART2_IRQHandler(void)
////{
////    unsigned char ch = 0;
////	  int length       = 0;
////    
////    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
////	  {
////			  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
////        ch = USART_ReceiveData(USART2);
////			  if(g_L1Mod_node.type == 1)//ASCII���������Ϳ�����������
////				{
////					  //Usart_Write_Bytes(DEBUG_USART, &ch, 1);
////				}
////				else if(g_L1Mod_node.type == 2)//HEX���������Ϳ�����������
////				{
////					  if(!fstatus)
////						{
////							  gresult = gresult|ch;
////							  if((gresult&0xffffff) == 0XB46903) //֡ͷ��B4 69 03
////						    {
////									 fstatus = true;//�ҵ�֡ͷ��
////									 gresult = 0;
////						    }
////								else
////								{
////									  gresult = gresult<<8;
////								}
////						}
////						else
////						{
////							 gbytes++;
////							 gdistance = gdistance | ch;
////							 if(gbytes != 4)
////							 {
////								  gdistance = gdistance<<8;
////							 }
////							 else
////							 {
////								   sprintf((char*)gcmd, "D=%f", gdistance/10000.0);
////								   //Usart_Write_Bytes(DEBUG_USART, gcmd, strlen((char*)gcmd));
////								   gbytes    = 0;
////								   gdistance = 0;
////								   fstatus = false;
////							 }
////							 
////						}
////				
////				}
////				else//����
////				{
////					  length                     = g_L1Mod_node.length;
////			      g_L1Mod_node.buf[length++] = ch;
////			      g_L1Mod_node.length        = length % 256;
////				}	  
////	  }
////		if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) == SET) 
////    {
////        USART_ClearFlag(USART2,USART_FLAG_ORE);
////        USART_ReceiveData(USART2);
////    }
////}









///*
//***************************************************
//��������USART2_IRQHandler
//���ܣ�����2�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//*/	
////DMA1-5
////void USART2_IRQHandler(void){
////	//�������ݵĳ���	
////	uint16_t USART2_len;	
////	if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET){
////		//��USART_IT_IDLE��־
////		USART2_len = USART2->SR;
////		USART2_len = USART2->DR; 
////		//�ر�DMA
////		DMA_Cmd(DMA1_Stream5,DISABLE);   
////		//��ձ�־λ		
////		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	
////		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
////		//��ȡ�������ݳ���
////		USART2_len = Length_USART2_RX_Buff - DMA1_Stream5->NDTR;	
////		DMA1_Stream5->NDTR = Length_USART2_RX_Buff;
////		DMA_Cmd(DMA1_Stream5, ENABLE);
////		
////		/*********�������Զ��岿��**********/	
////		if(USART2_len){

////			TFmini_Recive(Array_USART2_RX);
////			
////		}
////	}
////}

///*
//***************************************************
//��������USART3_IRQHandler
//���ܣ�����3�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//*/
////DMA1-1
//void USART3_IRQHandler(void){	
//	//�������ݵĳ���
//	uint16_t USART3_len;	
//	if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET){
//		//��USART_IT_IDLE��־
//		USART3_len = USART3->SR;
//		USART3_len = USART3->DR; 
//		//�ر�DMA
//		DMA_Cmd(DMA1_Stream1,DISABLE);    
//		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
//		//��ձ�־λ		
//		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
//		//��ȡ�������ݳ���
//		USART3_len = Length_USART3_RX_Buff - DMA1_Stream1->NDTR;		
//		DMA1_Stream1->NDTR = Length_USART3_RX_Buff;
//		DMA_Cmd(DMA1_Stream1, ENABLE);
//		
//		/*********�������Զ��岿��**********/	
//		if(USART3_len){
////			Driver_LobotReadDMA((uint8_t*)DMA1_Stream1->M0AR);
////            receiveHandle(); //���մ���                                
////			digitalIncreasing(&getsupervisorData()->Lobot_ErrorCount);
//			
//			//for(uint16_t i = 0;i < USART3_len;i++)
//				//ANO_DT_Data_Receive_Prepare(Array_USART3_RX[i]);			
//			
//			
//		}
//	}
//}

///*
//***************************************************
//��������UART4_IRQHandler
//���ܣ�����4�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//*/
////DMA1-2
//void UART4_IRQHandler(void){																			
//	//�������ݵĳ���
//	uint16_t UART4_len;	
//	if(USART_GetITStatus(UART4,USART_IT_IDLE) == SET){
//		//��USART_IT_IDLE��־
//		UART4_len = UART4->SR;
//		UART4_len = UART4->DR; 
//		//�ر�DMA
//		DMA_Cmd(DMA1_Stream2,DISABLE); 
//		//��ձ�־λ		
//		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
//		while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
//		//��ȡ�������ݳ���
//		UART4_len = Length_UART4_RX_Buff - DMA1_Stream2->NDTR;				
//		DMA1_Stream2->NDTR = Length_UART4_RX_Buff;
//		DMA_Cmd(DMA1_Stream2, ENABLE);
//		
//		/*********�������Զ��岿��**********/	
//		if(UART4_len){
//		};	//����ֻ��Ϊ�˼���warning��ʾ��ʹ��ʱ����ɾ��
//	}
//}

///*
//***************************************************
//��������UART5_IRQHandler
//���ܣ�����5�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//*/
////DMA1-0
//void UART5_IRQHandler(void){																			
//	//�������ݵĳ���
//	uint16_t UART5_len;	
//	if(USART_GetITStatus(UART5,USART_IT_IDLE) == SET){
//		//��USART_IT_IDLE��־
//		UART5_len = UART5->SR;
//		UART5_len = UART5->DR; 
//		//�ر�DMA
//		DMA_Cmd(DMA1_Stream0,DISABLE); 
//		//��ձ�־λ
//		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);	
//		while(DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);
//		//��ȡ�������ݳ���
//		UART5_len = Length_UART5_RX_Buff - DMA1_Stream0->NDTR;	
//		DMA1_Stream0->NDTR = Length_UART5_RX_Buff;
//		DMA_Cmd(DMA1_Stream0, ENABLE);
//		
//		/*********�������Զ��岿��**********/	
//		if(UART5_len){
//		}
//	}
//}

///*
//***************************************************
//��������USART6_IRQHandler
//���ܣ�����6�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//*/
////DMA2-6
//void USART6_IRQHandler(void){	
//	//�������ݵĳ���
//	uint16_t USART6_len;	
//	if(USART_GetITStatus(USART6,USART_IT_IDLE) == SET){
//		//��USART_IT_IDLE��־
//		USART6_len = USART6->SR;
//		USART6_len = USART6->DR; 
//		//�ر�DMA
//		DMA_Cmd(DMA2_Stream1,DISABLE); 
//		//��ձ�־λ		
//		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	
//		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
//		//��ȡ�������ݳ���
//		USART6_len = Length_USART6_RX_Buff - DMA2_Stream1->NDTR;			
//		DMA2_Stream1->NDTR = Length_USART6_RX_Buff;
//		DMA_Cmd(DMA2_Stream1, ENABLE);
//		
//		/*********�������Զ��岿��**********/	
//		if(USART6_len){
//           // Driver_WT931ReadDMA(Array_USART6_RX);  //��ȡ�����ϵ�����������
//           // digitalIncreasing(&getimuData()->IMU_ErrorCount);         //����������״̬��                       
//            
//            
//		}	
//	}
//}


///*
//***************************************************
//��������UART7_IRQHandler
//���ܣ�����7�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************
//*/
////*
////***************************************************
////��������UART7_IRQHandler
////���ܣ�����7�жϷ�����
////��ע���������ж�Ϊ�����ж�+DMA�ж�
////***************************************************
////DMA1-3
//	void UART7_IRQHandler(void){	
//		//�������ݵĳ���
//		uint16_t UART7_len;	
//		if(USART_GetITStatus(UART7,USART_IT_IDLE) == SET){
//			//��USART_IT_IDLE��־
//			UART7_len = UART7->SR;
//			UART7_len = UART7->DR; 
//			//�ر�DMA
//			DMA_Cmd(DMA1_Stream3,DISABLE);
//			//��ձ�־λ
//			DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);	
//			while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
//			//��ȡ�������ݳ���
//			UART7_len = Length_UART7_RX_Buff - DMA1_Stream3->NDTR;				
//			DMA1_Stream3->NDTR = Length_UART7_RX_Buff;
//			DMA_Cmd(DMA1_Stream3, ENABLE);
//			
//			/*********�������Զ��岿��**********/	
//			if(UART7_len){                
//              
//				//Openmv_Recive(Array_UART7_RX);
//				
//			}
//		}
//	}


