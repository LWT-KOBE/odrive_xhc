#ifndef __PID_H
#define __PID_H

#include "util.h"
#include "config.h"

typedef struct{
	f32_t *yCoefficient;
	f32_t *xCoefficient;
}coefficientStruct_t;

typedef struct{
	uint8_t differentialLength;
	f32_t 	*inputData;
	f32_t 	*outputData;
	uint8_t	initFlag;
	coefficientStruct_t *coefficient;
}differentialDataStruct_t;

typedef struct {
								// Last setpoint 											��������ֵ
	f32_t 						setPoint;		
								// Last position input										����״̬����
	f32_t 						dState;		
								// Integrator state   										����״̬
	f32_t 						iState;		
								// integral gain											��������
	f32_t 						*iGain;		
								// proportional gain										��������
	f32_t 						*pGain;		
								// derivative gain											΢������
	f32_t 						*dGain;		
								// low pass filter factor (1 - pole) for derivative gain	����΢������ĵ�ͨ�˲�������
	f32_t 						*fGain;
	f32_t 						*pMax, *iMax, *dMax, *oMax;
								// pointers to radio trim channels (or NULL)
	int16_t 					*pTrim, *iTrim, *dTrim, *fTrim;	
	f32_t 						pv_1, pv_2;
	f32_t 						co_1;
	f32_t 						pTerm_1;
	f32_t 						iTerm_1;
	f32_t 						dTerm_1;
	f32_t 						sp_1;
	
	differentialDataStruct_t 	*differential;
} pidStruct_t;




//����PID�ṹ�����ڴ��һ��PID������
typedef struct
{
   	float kp, ki, kd; //����ϵ��
    float error, lastError; //���ϴ����
    float integral, maxIntegral; //���֡������޷�
    float output, maxOutput; //���������޷�
	float pout;//�����������
	float alpha;//��ͨ�˲���ϵ��
	float last_dev;//��һ�ε�ͨ�˲����
	float this_dev;//��ǰ��ͨ�˲����
}PID;

//����PID�Ľṹ�壬������������PID
typedef struct
{
    PID inner; //�ڻ�
    PID outer; //�⻷
    float output; //�������������inner.output
}CascadePID;



typedef struct {
	f32_t dataFbd;
    f32_t dataRef;
    f32_t dataOut;
} pidData_t;

pidStruct_t *pidInit(systemConfigPID_t *PIDConfigData);
f32_t differentialCal(differentialDataStruct_t *differential,f32_t currentData);

float pidUpdate(pidStruct_t *pid, float setpoint, float position,float Dt);
void pidZeroIntegral(pidStruct_t *pid, float pv, float iState);
void pidZeroState(pidStruct_t *pid);
float PID_angel(float angle_target, float angle_current,float limit);
float Position_based_PID(float target,float current);
float Incremental_PID(float reality,float target);
float Position_PID(float reality,float target);
float Position_PID_G(float reality,float target);
float Position_PID_P(float reality,float target);
float Position_PID_N(float reality,float target);
float Position_PID_N1(float reality,float target);

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);

#endif


