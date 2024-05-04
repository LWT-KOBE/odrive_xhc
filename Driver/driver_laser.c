//#include "driver_laser.h"
//#include "MODBUSProtocol.h"

//usart_buf_t g_L1Mod_node;

//uint16_t Laser_distance=0;
//void Driver_Laser_Init(USART_TypeDef* USARTx, BSP_GPIOSource_TypeDef *USART_RX, BSP_GPIOSource_TypeDef *USART_TX, \
//														uint32_t baudRate, uint8_t PreemptionPriority, uint8_t SubPriority){
//	BSP_USART_TypeDef laser_USART;
//	laser_USART.USARTx = USARTx;
//	laser_USART.USART_RX = USART_RX;
//	laser_USART.USART_TX = USART_TX;
//	laser_USART.USART_InitStructure.USART_BaudRate = baudRate;							
//	laser_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
//	laser_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;			/*һ��ֹͣλ*/
//	laser_USART.USART_InitStructure.USART_Parity = USART_Parity_No;					/*��У��λ*/
//	laser_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
//	laser_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
//	
//	BSP_USART_Init(&laser_USART,PreemptionPriority,SubPriority);
////	BSP_USART_RX_DMA_Init(&laser_USART);	
////	BSP_USART_TX_DMA_Init(&laser_USART);	
//}
//	

///*****************************************************
//* ��������Init_L1_Usart
//* ��  ����
//*    USARTx[in]: ����
//*    pdata[in]:����ָ��
//*    length[in]:�������ݳ���
//* ����ֵ��void
//* ��  �������ڷ�������
//* ��  �ߣ�pamala�� 2020.2.16
//*****************************************************/
//void Usart_Write_Bytes(USART_TypeDef* USARTx, unsigned char *pdata, int length)
//{
//    int i = 0;
//    for(i = 0; i < length; i++)
//    {
//        USART_SendData(USARTx, pdata[i]);
//        while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
//        {}
//    }
//}

///*****************************************************
//* ��������Clear_L1Mod_Usart
//* ��  ����void
//* ����ֵ��void
//* ��  ����������ڻ�����
//* ��  �ߣ�pamala�� 2020.2.16
//*****************************************************/
//void Clear_L1Mod_Usart(void)
//{
//	  g_L1Mod_node.length = 0;
//	  g_L1Mod_node.valid  = false;
//	  g_L1Mod_node.type   = 0;
//	  //memset(g_L1Mod_node.buf, 0, 256);
//	  
//}

///***********************************************************
//* ��������HEX_Conti_Meas_Cmd
//* ��  ����void
//* ����ֵ��void
//* ��  ��������������������
//* ���ߣ�pamala, 2020.2.15
//************************************************************/
//void HEX_Conti_Meas_Cmd(void)             
//{
//    unsigned char cmd[5] = {0XA5, 0X5A, 0X03, 0X00, 0XFC};
//		Usart_Write_Bytes(USART2, cmd, 5);
//		g_L1Mod_node.type = 2;
//}


///***********************************************************
//* ��������HEX_FastConti_Meas_Cmd
//* ��  ����void
//* ����ֵ��void
//* ��  �������Ϳ���������������
//* ���ߣ�pamala, 2020.2.15
//************************************************************/
//void HEX_FastConti_Meas_Cmd(void)          
//{
//    unsigned char cmd[5] = {0XA5, 0X5A, 0X04, 0X00, 0XFB};
//		Usart_Write_Bytes(USART2, cmd, 5);
//		g_L1Mod_node.type = 2;
//}








///*****************************************************
//* ��������test11
//* ��  ����void
//* ����ֵ��void
//* ��  ����MODBUSЭ��Ķ�ȡ��������
//* ��  �ߣ�pamala�� 2020.2.16
//*****************************************************/
//void test11(void)
//{
//	unsigned short check = 0;
//	  int len = 0;
//	  unsigned char lsb, msb;
//	  //unsigned int result = 0;
//	  //unsigned char str[16] = {0};
//	
//    Get_Meas_Dis(1);

//	  //delay_test();

//	  len = g_L1Mod_node.length;
//	  check = CRC16(g_L1Mod_node.buf, len-2);
//	  lsb = check&0XFF;
//	  msb = (check >> 8)& 0xFF;
//	  if((lsb == g_L1Mod_node.buf[len-2])&&(msb == g_L1Mod_node.buf[len-1]))
//		{
//			  Laser_distance = ((g_L1Mod_node.buf[3]<<24)&0xff000000)|((g_L1Mod_node.buf[4]<<16)&0x00ff0000)
//			           |((g_L1Mod_node.buf[5]<<8)&0x0000ff00)|(g_L1Mod_node.buf[6]&0xff);
//			  //sprintf((char*)str, "D=%lfm\n", result/10000.0);
//			  //Usart_Write_Bytes(DEBUG_USART, str, strlen((char*)str));
//		}
//}

