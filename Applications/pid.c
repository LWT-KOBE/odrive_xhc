#include "parameter.h"
#include "pid.h"
#include "board.h"
pidStruct_t *pidInit(systemConfigPID_t *PIDConfigData){
	pidStruct_t *pid;
    
	pid = (pidStruct_t *)aqDataCalloc(1, sizeof(pidStruct_t));

	pid->pMax = &PIDConfigData->PID_PM;
	pid->iMax = &PIDConfigData->PID_IM;
	pid->dMax = &PIDConfigData->PID_DM;
	pid->oMax = &PIDConfigData->PID_OM;
	pid->pGain = &PIDConfigData->PID_P;
	pid->iGain = &PIDConfigData->PID_I;
	pid->dGain = &PIDConfigData->PID_D;
	pid->fGain = &PIDConfigData->PID_F;
	pid->pTrim = NULL;
	pid->iTrim = NULL;
	pid->dTrim = NULL;
	pid->fTrim = NULL;

	return pid;
}

/*
@ param differential:	differential data
@ param currentData:	new data
@ return:							update data
*/
f32_t differentialCal(differentialDataStruct_t *differential,f32_t currentData){
	uint8_t loopNum = differential -> differentialLength;
	differential -> inputData[loopNum - 1] = currentData;
	
	/* H(z)������� */
	differential -> outputData[loopNum - 1] = 0.0f;
	for(uint8_t i = 0;i < loopNum;i++){
		differential -> outputData[loopNum - 1] +=	\
		differential -> coefficient -> xCoefficient[i] * differential -> inputData[loopNum - 1 - i];
		
		if(i != loopNum - 1)
			differential -> outputData[loopNum - 1] -=	\
			differential -> coefficient -> yCoefficient[i + 1] * differential -> outputData[loopNum - 1 - i - 1];
	}

	/* x(n) ���б��� */
	for(uint8_t i = 0;i < loopNum - 1;i++){
			differential->inputData[i] = differential -> inputData[i + 1];
	}
	
	/* y(n) ���б��� */
	for(uint8_t i = 0;i < loopNum - 1;i++){
			differential->outputData[i] = differential -> outputData[i + 1];
	}
	return (differential->outputData[loopNum - 1]);
}






float pidUpdate(pidStruct_t *pid, float setpoint, float position,float Dt){
    float error;
    float p = *pid->pGain;
    float i = *pid->iGain;
    float d = (pid->dGain) ? *pid->dGain : 0.0f;
    //�����ַ�������룬��ȡ��ַ�ڵ�ֵ��û����ȡ1
    float f = (pid->fGain) ? *pid->fGain : 1.0f;															

    error = setpoint - position;

    //���������                                       
    pid->pTerm_1 = p * error;
    if (pid->pTerm_1 > *pid->pMax) {
			pid->pTerm_1 = *pid->pMax;
    }
    else if (pid->pTerm_1 < -*pid->pMax) {
			pid->pTerm_1 = -*pid->pMax;
    }
    //���ʵ��ļ��޼������״̬   
    pid->iState += error;
    pid->iTerm_1 = i * pid->iState * Dt;
    if (pid->iTerm_1 > *pid->iMax) {
			pid->iTerm_1 = *pid->iMax;
			pid->iState = pid->iTerm_1 / i;
    }
    else if (pid->iTerm_1 < -*pid->iMax) {
			pid->iTerm_1 = -*pid->iMax;
			pid->iState = pid->iTerm_1 / i;
    }

    //΢��																															
    if (pid->dGain) { //�������΢����																													
			error = -position;
        
            //�ڴ˴�ȥ��ʱ��ϵ��
			pid->dTerm_1 = (d * f) * (error - pid->dState);													
			pid->dState += f * (error - pid->dState);
			if (pid->dTerm_1 > *pid->dMax) {
					pid->dTerm_1 = *pid->dMax;
			}
			else if (pid->dTerm_1 < -*pid->dMax) {
					pid->dTerm_1 = -*pid->dMax;
			}
    }
    else {  //��������΢�����ȡ0                       
			pid->dTerm_1 = 0.0f;																										
    }

    pid->pv_1 = position;
    pid->sp_1 = setpoint;
    pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;

    //��PID����޷�
    if (pid->co_1 > *pid->oMax) {																							
			pid->co_1 = *pid->oMax;
    }
    else if (pid->co_1 < -*pid->oMax) {
			pid->co_1 = -*pid->oMax;
    }

    return pid->co_1;
}

void pidZeroIntegral(pidStruct_t *pid, float pv, float iState){
	if (*pid->iGain != 0.0f)
		pid->iState = iState / *pid->iGain;
	pid->dState = -pv;
	pid->sp_1 = pv;
	pid->co_1 = 0.0f;
	pid->pv_1 = pv;
	pid->pv_2 = pv;
}

void pidZeroState(pidStruct_t *pid){
	pid->setPoint = 0.0f;
	pid->dState = 0.0f;
	pid->iState = 0.0f;
	pid->co_1 = 0.0f;
	pid->pv_1 = 0.0f;
	pid->sp_1 = 0.0f;
	pid->pTerm_1 = 0.0f;
	pid->iTerm_1 = 0.0f;
	pid->dTerm_1 = 0.0f;
}


/**************************************************************************
�������ܣ��޷�����
��ڲ�����insert��ֵ,low��Сֵ,high���ֵ
����  ֵ���޷����ֵ
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}



/**************************************************************************
�������ܣ��Ƕ�ת��PID
��ڲ�����angle_target�����Ƕ�,angle_current��ǰ�Ƕ�,limit������ƣ��ٶȣ�
����  ֵ���޷����ֵ
**************************************************************************/
float PID_angel(float angle_target, float angle_current,float limit){
	
	//PID���ֵ
	static float motor_out;
	//���嵱ǰƫ��,��һ��ƫ��,�۲�ֵ
	static float bias,bias_last,err_num,Kp = 0.075f,Ki = 0.0000f,Kd = 0.5f;
	
	//���ƫ��ֵ,����΢��,�����������,������
	bias = angle_target - angle_current;
	//����޷�������ת��Ƕȵ�������ֵ,
	bias = target_limit_float(bias,-90,90);
	//��û���ת��,��־λΪ0
	//����۲�ֵ,���ڻ���,�ӽ���ʵֵ
	err_num += bias;
	
	//PID���������PWMֵ
	motor_out = Kp*bias + Ki*err_num + Kd*(bias-bias_last);
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	
	//����������
	if(motor_out > limit)
		motor_out = limit;
	if(motor_out < -limit)
		motor_out = -limit;
	
//	//��ǰɲͣ,���ù���ת��
	if(fabs(bias) < 0.25){
		err_num = 0;
		bias_last = 0;
		motor_out = 0;
		Angle_Goal.change++;
		if(Angle_Goal.change>50){
			Angle_Goal.finish = 1;
			Angle_Goal.change = 0;
		}
	}else{
		Angle_Goal.finish = 0;
		Angle_Goal.change = 0;
	}

	//��������Ƶ��ֵ
	return motor_out;
}

///**************************************************************************
//�������ܣ�λ��ʽPID������
//��ڲ�����ʵ��λ�ã�Ŀ��λ��
//����  ֵ�����PWM
//����λ��ʽ��ɢPID��ʽ
//λ��ʽPID����������: Kp:�����Ӧ�ٶ� Ki:��̬ Kd:������
//pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
//e(k)������ƫ�� 
//e(k-1)������һ�ε�ƫ��  
//��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,...,k;
//pwm�������
//**************************************************************************/
//float XC_Position_PID(float reality,float target)
//{ 	
//    static float Bias,Pwm,Last_Bias,Integral_bias=0;
//    
//    Bias=target-reality;                            /* ����ƫ�� */
//    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
//    
//    if(Integral_bias> 360) Integral_bias = 360;   	/* �����޷� */
//    if(Integral_bias<-360) Integral_bias =-360;
//    
//    Pwm = (Angle_Position_KP*Bias)                        /* �������� */
//         +(Angle_Position_KI*Integral_bias)               /* ���ֻ��� */
//         +(Angle_Position_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
//    
//    Last_Bias=Bias;                                 	/* �����ϴ�ƫ�� */
//	
//														/* ����޷� */
//    return Pwm;                                     	/* ������ */
//}

///**************************************************************************
//�������ܣ�����PID������
//��ڲ�����ʵ��ֵ��Ŀ��ֵ
//����  ֵ�����PWM
//��������ʽ��ɢPID��ʽ 
//����ʽPID����������: Kp:������ Ki:�����Ӧ�ٶ� Kd:��̬
//pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
//e(k)������ƫ�� 
//e(k-1)������һ�ε�ƫ��  �Դ����� 
//pwm�����������
//**************************************************************************/
//float XC_Incremental_PID(float reality,float target)
//{ 	
//	 static float Bias,Pwm,Last_bias=0,Prev_bias=0;
//    
//	 Bias=target-reality;                                   /* ����ƫ�� */
//    
//	 Pwm += (Angle_Incremental_KP*(Bias-Last_bias))               /* �������� */
//           +(Angle_Incremental_KI*Bias)                           /* ���ֻ��� */
//           +(Angle_Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* ΢�ֻ��� */ 
//    
//     Prev_bias=Last_bias;                                   /* �������ϴ�ƫ�� */
//	 Last_bias=Bias;	                                    /* ������һ��ƫ�� */
//		
//	 if(fabs(Bias)<0.75){
//		 Prev_bias = 0;
//		 Last_bias = 0;
//		 Pwm = 0;
//	 }
//    
//	if(Pwm >  75) Pwm =  75;
//	if(Pwm < -75) Pwm = -75;
//	return Pwm;                                            /* ������ */
//}


