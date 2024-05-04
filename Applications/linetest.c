#include "application.h"
#include "driver.h"
#include "nav_para.h"
#include "math.h"




float R = 10; 					 //�뾶����(cm)
float angle = 40.0;					 //�ڶ��Ƕ�����(��)
uint8_t RoundDir = 0; 				 //����ת����
float priod = 512.0;  //��������(����)
////��λ���� 0, 10   20   30   40   50   60   70   80   90   100  110  120  130  140  150  160  170 180
float Phase[19]= {0,-0.1,-0.05,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.05,0.05,0.05,0.07,0};
float Ax = 0.0;
float Ay = 0.0;
float Pendulum_angle = 0.0;
uint32_t pOffset = 0;
float Normalization = 0.0;
float Omega = 0.0;  
float phase = 0.0;

/**************************************����********************************************/
//��ڲ�����
//R���ڶ��ľ���
//angle���ڶ��ĽǶ�
void Draw_Line(float R,float angle)			//���� �߳�1-60cm����1cm �Ƕ�0-180 ����10��
{

   static float MoveTimeCnt;      
    if(getcontrolData()->loops%2){    
    MoveTimeCnt += 6;							 //ÿ4ms����1��
    }
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ�� ��������ڶ�Ӧʱ���뵥�����ڵĹ�ϵ
	Omega = 2.0f*M_PI*Normalization;			 //��һ������	�����ڶ�Ӧ��ʱ��ӳ�䵽0-2pi����  	 					
	Pendulum_angle = atan(R/15.00f)*57.2958f; 	//���ݰڷ�����Ƕ�,94.8Ϊ�ڸ˾�����泤��(cm)57.2958�ǰѻ���ת�Ƕ�
	
	//����������ͼ�ν��߷ֽ⵽X,Y����
	Ax = Pendulum_angle*cos(angle*0.017453f);	 //�����X����ڷ�������0.017453Ϊ����ת��
	Ay = Pendulum_angle*sin(angle*0.017453f);	 //�����Y����ڷ�����
									
	getGimbalData()->pitchAngleRef = Ax*sin(Omega); //�����X����̬�ڽ�
	getGimbalData()->yawAngleRef  = Ay*sin(Omega); //�����Y����̬�ڽ�
	
//	getGimbalData()->yawAngleOut = constrainFloat(getGimbalData()->yawAngleOut,getConfigData()->yawMinRange,getConfigData()->yawMaxRange); 
//	getGimbalData()->pitchAngleOut = constrainFloat(getGimbalData()->pitchAngleOut,getConfigData()->pitchMinRange,getConfigData()->pitchMaxRange);     
    
    
}

/**************************************��Բ********************************************/
void Round(float R)
{
	
   static float MoveTimeCnt;      
    if(getcontrolData()->loops%2){    
    MoveTimeCnt += 4;							 //ÿ4ms����1��
    }
	Normalization = (float)MoveTimeCnt / priod;	 //�Ե������ڹ�һ�� ��������ڶ�Ӧʱ���뵥�����ڵĹ�ϵ
	Omega = 2.0f*M_PI*Normalization;			 //��һ������	�����ڶ�Ӧ��ʱ��ӳ�䵽0-2pi����  	 					
	Pendulum_angle = atan(R/15.00f)*57.2958f; 	//���ݰڷ�����Ƕ�,94.8Ϊ�ڸ˾�����泤��(cm)57.2958�ǰѻ���ת�Ƕ�
 
	//����������ͼ�ν��߷ֽ⵽X,Y����	
	getGimbalData()->pitchAngleRef = Pendulum_angle*sin(Omega); 	 		  //�����X����ǰ�ڽ�
	getGimbalData()->yawAngleRef  = Pendulum_angle*sin(Omega+3.141592/2)-1; //�����Y����ǰ�ڽ�	��-1λY����������
   
}    
    
void Mode_1(void){
	OLED_ShowCHinese(48,2,6);//��
	OLED_ShowCHinese(66,2,7);//��
	OLED_ShowCHinese(84,2,8);//һ

}

void Mode_2(void){
    //Draw_Line(R*0.80f,angle);
	OLED_ShowCHinese(48,2,6);//��
	OLED_ShowCHinese(66,2,7);//��
	OLED_ShowCHinese(84,2,9);//��      
}

void Mode_3(void){
    
	OLED_ShowCHinese(48,2,10);//��
	OLED_ShowCHinese(66,2,11);//��
	OLED_ShowCHinese(84,2,8);//һ    
    
}

void Mode_4(void){
    
    
    //Round(R);
	OLED_ShowCHinese(48,2,10);//��
	OLED_ShowCHinese(66,2,11);//��
	OLED_ShowCHinese(84,2,9);//��    
   
}



