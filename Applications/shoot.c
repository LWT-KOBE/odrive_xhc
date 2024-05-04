#include "application.h"
#include "bsp.h"

shootStruct_t shootData;

#ifdef USE_SNAIL
void shootUpdate(uint8_t FricONflag){
	static uint16_t shootPwm = SHOOT_OFF;
	static uint16_t maxShootPwm;
	if(shootData.shootMode == MANUAL_SINGLE){
		maxShootPwm = HIGH_SPEED;
	}
	else{
		maxShootPwm = LOW_SPEED;
	}
	if(FricONflag){																									//ֱ�Ӱ�shootPWM����MAXshootpwm ��Ħ���־ʹ򿪼���
		LASER = LASER_ON;
		
#ifdef USE_2019_B
		CHANGE = CHANGE_ON;
#endif
		
		if(shootPwm > maxShootPwm){
			shootPwm -= SHOOT_PWM_INCREMENT;
			if(shootPwm < maxShootPwm)
				shootPwm = maxShootPwm;
		}
		if(shootPwm < maxShootPwm){
			shootPwm += SHOOT_PWM_INCREMENT;
			if( shootPwm > maxShootPwm ) 
				shootPwm = maxShootPwm;																		//��������ģʽ���26m/s����
		}
		FricMotor_L = FricMotor_R = shootPwm;
	}
	else{
#ifdef USE_2019_B
		CHANGE = CHANGE_OFF;
#endif
		
		LASER = LASER_OFF;																						//Ħ���ֲ�������Ҳ����
		FricMotor_L = FricMotor_R = SHOOT_OFF;
	}
}

void shootInit(void){																							//���������ʼ��
	digitalLo(&shootData.shootState);	
	BSP_TIM_PWM_Init(SHOOT_TIM,SHOOT_PERIOD,SHOOT_PRES,SHOOT_L_PIN,SHOOT_R_PIN,NULL,NULL);	//������ˢ���,Ƶ��400Hz																								
	FricMotor_L = FricMotor_R = SHOOT_OFF;
	shootData.FricONflag = DISABLE;
	digitalHi(&shootData.shootState);	
}
#endif

void bulletMonitorUpdata(void){
	if(BULLET_MONITOR_FLAG)
		shootData.bulletMonitorFlag = BULLET_READY;
	else
		shootData.bulletMonitorFlag = NO_BULLET;
}

void bulletMonitorInit(){
#ifdef USE_2019_A
	BSP_GPIO_Init(BSP_GPIOC0,GPIO_Mode_IPU);										  //17mmС�ӵ�ʶ��
#endif
#ifdef USE_2019_B
	BSP_GPIO_Init(BSP_GPIOC5,GPIO_Mode_IPU);										  //17mmС�ӵ�ʶ��
	BSP_GPIO_Init(BSP_GPIOB9,GPIO_Mode_Out_PP);										//����װ�ÿ���
#endif
		BSP_GPIO_Init(BSP_GPIOA9,GPIO_Mode_Out_PP);										//�����ʼ��
}
