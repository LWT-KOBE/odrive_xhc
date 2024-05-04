#include "board.h"
/*
***************************************************
��������CAN1_RX0_IRQHandler
���ܣ�CAN1�����ж�
��ע��y:700 p:3180
				
***************************************************
*/
u32 rxbuf1,rxbuf2,rxbuf3,rxbuf4,rxbuf5,rxbuf6;
//u8 buf[10],buf1[10];


// 0x021��M0����������ź�CAN��ַ������ָ��Ϊ odrv0.axis0.config.can_node_id =0x001
// 0x041��M1����������ź�CAN��ַ������ָ��Ϊ odrv0.axis0.config.can_node_id =0x002
//        rxbuf2=  can1_rx_msg.StdId>>5;
//		rxbuf4 = 0x021>>5;
//		rxbuf5 = 0x041>>5;
//		rxbuf6 = 0x061>>5;
//rxbuf3=can1_rx_msg.StdId;
	
void CAN1_RX0_IRQHandler(void){
	CanRxMsg can1_rx_msg;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);		
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		rxbuf3=can1_rx_msg.StdId;	
		rxbuf6 = rxbuf3>>5;
		rxbuf5 = rxbuf3 & 0x01F;
		
		
		rxbuf4 = 0x3FF>>5;		
		/*********�������Զ��岿��**********/
		switch(can1_rx_msg.StdId>>5){         
				case AXIS0_ID:
		
					switch(can1_rx_msg.StdId&0x01F){ 

						case MSG_GET_VBUS_VOLTAGE:
								ODReadVbusData(&can1_rx_msg,&OdReceivedData);
											
						break;	
						
						case MSG_GET_TEMP:
								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis0);
											
						break;								
					
						case MSG_GET_MOTOR_ERROR:
								ODReadMotorErrorData(&can1_rx_msg,&OdReceivedData,axis0);			
						break;
						
						case MSG_GET_ENCODER_COUNT:
								ODReadEncoderCountData(&can1_rx_msg,&OdReceivedData,axis0);			
						break;						
						
						case MSG_GET_ENCODER_ESTIMATES:
								ODReadEncodeEstimatesData(&can1_rx_msg,&OdReceivedData,axis0);			
						break;								
		
						case MSG_GET_ENCODER_ERROR:
								ODReadEncodeErrorData(&can1_rx_msg,&OdReceivedData,axis0);			
						break;						
	
						case MSG_ODRIVE_HEARTBEAT:
							ODReadHeartBeatData(&can1_rx_msg,&OdReceivedData,axis0);
						break;	
						
						
					default:	break;																			

					
					}						
						
				break;			
			
				case AXIS1_ID:							
					switch(can1_rx_msg.StdId&0x01F){ 
		
						case MSG_GET_MOTOR_ERROR:
								ODReadMotorErrorData(&can1_rx_msg,&OdReceivedData,axis1);			
						break;	
						case MSG_GET_TEMP:
								ODReadTempData(&can1_rx_msg,&OdReceivedData,axis1);
											
						break;						
						
						case MSG_GET_ENCODER_COUNT:
								ODReadEncoderCountData(&can1_rx_msg,&OdReceivedData,axis1);			
						break;								
	
						case MSG_GET_ENCODER_ESTIMATES:
								ODReadEncodeEstimatesData(&can1_rx_msg,&OdReceivedData,axis1);			
						break;	
	
						case MSG_GET_ENCODER_ERROR:
								ODReadEncodeErrorData(&can1_rx_msg,&OdReceivedData,axis1);			
						break;	
						case MSG_ODRIVE_HEARTBEAT:
							ODReadHeartBeatData(&can1_rx_msg,&OdReceivedData,axis1);
						break;							
						
					default:	break;																			

					
					}						
						
				break;					
				
				
			default:	break;		

        }	 


	}
}


/*
***************************************************
��������CAN1_TX_IRQHandler
���ܣ�CAN1�����ж�
��ע��
***************************************************
*/
void CAN1_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

		/*********�������Զ��岿��**********/
        
        
	}
}

/*
***************************************************
��������CAN2_RX0_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************fricWheelData
*/	
void CAN2_RX0_IRQHandler(void){
//   CanRxMsg can2_rx_msg;
//	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET){
//		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
//		
//		CAN_Receive(CAN2, CAN_FIFO0, &can2_rx_msg);
//				
//		/*********�������Զ��岿��**********/

//		switch(can2_rx_msg.StdId){         

//			case PITCH_RMD:
//				gimbal_readData(&can2_rx_msg,&pitchMotorData);
//				digitalIncreasing(&getGimbalData()->gimbalError[0].errorCount);
//				break;
//			case YAW_RMD:
//				gimbal_readData(&can2_rx_msg,&yawMotorData);
//				digitalIncreasing(&getGimbalData()->gimbalError[1].errorCount);
//				break;			
//				case AXIS0_ID:			
//					ODReadData(&can2_rx_msg,&OdReceivedData);	
//				break;		
////				case TEST_ID:			
////					ODReadData(&can2_rx_msg,&OdReceivedData);	
////				break;	
//				
////				case HeartBeat_Signal:
////			
////					ODReadHeartBeatData(&can2_rx_msg,&OdReceivedData);	
////				break;				
//			
//			default:	break;	

//        
//	}
//}

}
/*
***************************************************
��������CAN2_TX_IRQHandler
���ܣ�CAN2�����ж�
��ע��
***************************************************
*/
void CAN2_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		
		/*********�������Զ��岿��**********/
	}
}
