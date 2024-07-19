#include "board.h"

void EXTI0_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){    
		EXTI_ClearFlag(EXTI_Line0);          
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		/*********�������Զ��岿��**********/
	}
}

void EXTI1_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){    
		EXTI_ClearFlag(EXTI_Line1);          
		EXTI_ClearITPendingBit(EXTI_Line1);
		Angle_Goal.target ++;
		/*********�������Զ��岿��**********/
	}
}

void EXTI2_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line2) != RESET){    
		EXTI_ClearFlag(EXTI_Line2);          
		EXTI_ClearITPendingBit(EXTI_Line2);
		
		/*********�������Զ��岿��**********/
	}
}

void EXTI3_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line3) != RESET){    
		EXTI_ClearFlag(EXTI_Line3);          
		EXTI_ClearITPendingBit(EXTI_Line3);
		
		/*********�������Զ��岿��**********/
	}
}

void EXTI4_IRQHandler(void){   

		
	if(EXTI_GetITStatus(EXTI_Line4) != RESET){    
		EXTI_ClearFlag(EXTI_Line4);          
		EXTI_ClearITPendingBit(EXTI_Line4);
		
		/*********�������Զ��岿��**********/
	}
   
        
	
}

void EXTI9_5_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line6) != RESET){
		//delay_ms(10);
		EXTI_ClearFlag(EXTI_Line6);          
		EXTI_ClearITPendingBit(EXTI_Line6);
		//Angle_Goal.target++;
		
		
		/*********�������Զ��岿��**********/
	}
    
}

void EXTI15_10_IRQHandler(void) {
	
       //��ʱ10ms
     delay_ms(10); 
	
	if(KEY3==0)	 	 //KEY1����
	{    
	
        if(Menu==1)	Set_Pos+=0.25f;   
        else if(Menu==2)	Set_Vel+=0.5f;  
        else if(Menu==3)	Set_Cur+=0.05f;    
//		else if(Menu==0){
//			Set_Pos =0.0f;
//			Set_Vel =0.0f;
//			Set_Cur =0.0f;
//		}			
	
    }


	
	EXTI_ClearITPendingBit(EXTI_Line12); //���LINE12�ϵ��жϱ�־λ      
	//EXTI_ClearITPendingBit(EXTI_Line15); //���LINE12�ϵ��жϱ�־λ      


	
}

