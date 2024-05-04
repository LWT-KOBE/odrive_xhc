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


#endif


