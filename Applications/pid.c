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

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ
λ��ʽPID����������: Kp:�����Ӧ�ٶ� Ki:��̬ Kd:������
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,...,k;
pwm�������
**************************************************************************/
float Position_PID(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0,Position_KP,Position_KI,Position_KD;
    Position_KP = 0.35f;
	Position_KI = 0.005f;
	Position_KD = 0.0f;
	
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> 4) Integral_bias = 4;   	/* �����޷� */
    if(Integral_bias<-4) Integral_bias =-4;
    
    Pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(Position_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 	/* �����ϴ�ƫ�� */
	
	//�޷�
//	if(Pwm >  0.9f) Pwm =  0.9f;
//	if(Pwm < -0.9f) Pwm = -0.9f;						/* ����޷� */
	
	if(Pwm >  0.1f) Pwm =  0.1f;
	if(Pwm < -0.1f) Pwm = -0.1f;						/* ����޷� */
	
//	if(fabs(Bias)<0.15){
//		Pwm = 0;
//	}
	
    return Pwm;                                     	/* ������ */
}

/**************************************************************************
�������ܣ�����PID������
��ڲ�����ʵ��ֵ��Ŀ��ֵ
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
����ʽPID����������: Kp:������ Ki:�����Ӧ�ٶ� Kd:��̬
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
**************************************************************************/
float Incremental_PID(float reality,float target)
{ 	
	 static float Bias,Pwm,Last_bias=0,Prev_bias=0,Incremental_KP,Incremental_KI,Incremental_KD;
     Incremental_KP = 0.5f;
	 Incremental_KI = 0.5f;
	 Incremental_KD = 0.00f;
	 Bias=target-reality;                                   /* ����ƫ�� */
    
	 Pwm += (Incremental_KP*(Bias-Last_bias))               /* �������� */
           +(Incremental_KI*Bias)                           /* ���ֻ��� */
           +(Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* ΢�ֻ��� */ 
    
     Prev_bias=Last_bias;                                   /* �������ϴ�ƫ�� */
	 Last_bias=Bias;	                                    /* ������һ��ƫ�� */
		
//	 if(fabs(Bias)<0.75){
//		 Prev_bias = 0;
//		 Last_bias = 0;
//		 Pwm = 0;
//	 }
    
	//�޷�
	if(Pwm >  2.2f) Pwm =  2.2f;
	if(Pwm < -2.2f) Pwm = -2.2f;
	return Pwm;                                            /* ������ */
}

//λ��ʽPID����
float Position_based_PID(float target,float current){
	static float err,err_num,Pwm,err_last,Kp,Ki,Kd;
	
	
	
	//�������
	err = target - current;
	if(fabs(err) > 100){
		err = 0;
	}
	//����ۼ�
	err_num += err;
	
	
	
	//
	//���������ת��
	Kp = 0.05f;
	Ki = 0.000f;
	Kd = 0.5f;
	//���������
//	Kp = 2.5f;
//	Ki = 0.0005f;
//	Kd = 5.0f;
	
	//�ֶ�PID
	if(err < 5){ 
		Kp = 0.05f;
		Ki = 0.000f;
		Kd = 0.5f;
	}
	
	//����
	Pwm = Kp*err + Ki*err_num + Kd*(err - err_last);
	//����
	err_last = err;
	
	//�޷�

	if(Pwm>0.1) Pwm  =  0.1;
	if(Pwm<-0.1)Pwm  = -0.1;
	
	return Pwm;
}


/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ
λ��ʽPID����������: Kp:�����Ӧ�ٶ� Ki:��̬ Kd:������
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,...,k;
pwm�������
**************************************************************************/
float Position_PID_G(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0,Position_KP,Position_KI,Position_KD;
	
	//��ʹ�û���
	Position_KP = 0.055f;
	Position_KI = 0.001f;
	Position_KD = 7.0f;
	
//	//ʹ�û���
//	Position_KP = 0.045f;
//	Position_KI = 0.0001f;
//	Position_KD = 4.5f;
	
//	Position_KP = 0.20f;
//	Position_KI = 0.0000f;
//	Position_KD = 00.0f;
	
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> 100) Integral_bias = 100;   	/* �����޷� */
    if(Integral_bias<-100) Integral_bias =-100;
    
    Pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(Position_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 	/* �����ϴ�ƫ�� */
	
	//�޷�
	if(Pwm >  1.5f) Pwm =  1.5f;
	if(Pwm < -1.5f) Pwm = -1.5f;						/* ����޷� */
	

	
	if(fabs(Bias)<0.75){
		Pwm = 0;
	}
	
    return Pwm;                                     	/* ������ */
}


float Position_PID_P(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0,Position_KP,Position_KI,Position_KD;
	
	//��ʹ�û���
	Position_KP = 0.055f;
	Position_KI = 0.001f;
	Position_KD = 7.0f;
	
	//ʹ�û���
//	Position_KP = 0.045f;
//	Position_KI = 0.0000f;
//	Position_KD = 4.5f;
	
//	Position_KP = 0.20f;
//	Position_KI = 0.0000f;
//	Position_KD = 00.0f;
	
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> 100) Integral_bias = 100;   	/* �����޷� */
    if(Integral_bias<-100) Integral_bias =-100;
    
    Pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(Position_KD*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 	/* �����ϴ�ƫ�� */
	
	//�޷�
	if(Pwm >  1.5f) Pwm =  1.5f;
	if(Pwm < -1.5f) Pwm = -1.5f;						/* ����޷� */
	

	
	if(fabs(Bias)<0.75){
		Pwm = 0;
	}
	
    return Pwm;                                     	/* ������ */
}

/**************************************************************************
�������ܣ�����ȫ΢��λ��ʽPID������
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ
λ��ʽPID����������: Kp:�����Ӧ�ٶ� Ki:��̬ Kd:������
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,...,k;
pwm�������
**************************************************************************/
float Position_PID_N(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0,Position_KP,Position_KI,Position_KD,last_dev,this_dev,alpha,last_Input,dInput;
	//ʹ�û���
	Position_KP = 0.1f;
	Position_KI = 0.0000f;
	Position_KD = 9.0f;
	
	//��ͨ�˲�ϵ��
	alpha = 0.1;

    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
	dInput = reality - last_Input;					/* ��������ƫ�� */
    
    if(Integral_bias> 100) Integral_bias = 100;   	/* �����޷� */
    if(Integral_bias<-100) Integral_bias =-100;
    
	//һ�׵�ͨ�˲�΢�ֲ��ִ���
	this_dev = Position_KD * (1 - alpha) * (Bias - Last_Bias) + alpha * last_dev;
	
//	//΢�ֳ��֮���ͨ�˲�����
//	this_dev = Position_KD * (1 - alpha) * dInput + alpha * last_dev;
	
    Pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(this_dev);           					/* ΢�ֻ��� */
		 
    
    Last_Bias=Bias;                                 	/* �����ϴ�ƫ�� */
	
	//�޷�
	if(Pwm >  1.7f) Pwm =  1.7f;
	if(Pwm < -1.7f) Pwm = -1.7f;						/* ����޷� */
	
	
	//��¼��һ����ͨ΢�ִ���֮���ֵ
	last_dev = this_dev;
	
	//����
	if(fabs(Bias)<0.7){
		Pwm = 0;
		Integral_bias = 0;
	}
	
	//������һ�ε�����ֵ
	last_Input = reality;
    return Pwm;                                     	/* ������ */
}



/**************************************************************************
�������ܣ�����ȫ΢��λ��ʽPID������
��ڲ�����ʵ��λ�ã�Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ
λ��ʽPID����������: Kp:�����Ӧ�ٶ� Ki:��̬ Kd:������
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,...,k;
pwm�������
**************************************************************************/
float Position_PID_N1(float reality,float target)
{ 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0,Position_KP,Position_KI,Position_KD,last_dev,this_dev,alpha;
	//ʹ�û���
	Position_KP = 0.1f;
	Position_KI = 0.0000f;
	Position_KD = 9.5f;
	
	//��ͨ�˲�ϵ��
	alpha = 0.1;

    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> 100) Integral_bias = 100;   	/* �����޷� */
    if(Integral_bias<-100) Integral_bias =-100;
    
	//һ�׵�ͨ�˲�΢�ֲ��ִ���
	this_dev = Position_KD * (1 - alpha) * (Bias - Last_Bias) + alpha * last_dev;
	
    Pwm = (Position_KP*Bias)                        /* �������� */
         +(Position_KI*Integral_bias)               /* ���ֻ��� */
         +(this_dev);           					/* ΢�ֻ��� */
		 
    
    Last_Bias=Bias;                                 	/* �����ϴ�ƫ�� */
	
	//�޷�
	if(Pwm >  1.7f) Pwm =  1.7f;
	if(Pwm < -1.7f) Pwm = -1.7f;						/* ����޷� */
	
	
	//��¼��һ����ͨ΢�ִ���֮���ֵ
	last_dev = this_dev;
	
	//����
	if(fabs(Bias)<0.7){
		Pwm = 0;
		Integral_bias = 0;
	}
	
    return Pwm;                                     	/* ������ */
}


//���ڳ�ʼ��pid�����ĺ���
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
	
	//��ͨ�˲�������
	pid->alpha = 0.1f;
	pid->this_dev = 0;
	pid->last_dev = 0;
}
 
//����һ��pid����
//����Ϊ(pid�ṹ��,Ŀ��ֵ,����ֵ)������������pid�ṹ���output��Ա��
void PID_Calc(PID *pid, float reference, float feedback)
{
 	//��������
    pid->lastError = pid->error; //����error������
    pid->error = reference - feedback; //������error
		
    //�����ͨ�˲�֮���΢��
	pid->this_dev = (1 - pid->alpha) * pid->kd * (pid->error - pid->lastError) + pid->alpha * pid->last_dev; 

    //�������
    pid->pout = pid->error * pid->kp;
    //�������
    pid->integral += pid->error * pid->ki;
	
    //�����޷�
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //�������
    pid->output = pid->pout + pid->this_dev + pid->integral;
	
	//������һ�ε�ͨ�˲�֮���΢��ֵ
	pid->last_dev = pid->this_dev;
	
	
	if(fabs(pid->error)<0.75){
		pid->output = 0;
		Motor_SpeedB_Goal.change++;
		if(Motor_SpeedB_Goal.change > 50){
			Motor_SpeedB_Goal.finish = 1;
			Motor_SpeedB_Goal.change = 0;
		}
	}else{
		Motor_SpeedB_Goal.finish = 0;
		Motor_SpeedB_Goal.change = 0;
	}
	
    //����޷�
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}



//����PID�ļ��㺯��
//����(PID�ṹ��,�⻷Ŀ��ֵ,�⻷����ֵ,�ڻ�����ֵ)
//�����λ�ô�������,��outerRefΪ����λ�ã�outerFdbΪ��ǰλ��,innerFdbΪ��ǰ�ٶ�
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid->outer, outerRef, outerFdb); //�����⻷
    PID_Calc(&pid->inner, pid->outer.output, innerFdb); //�����ڻ�
    pid->output = pid->inner.output; //�ڻ�������Ǵ���PID�����
}

