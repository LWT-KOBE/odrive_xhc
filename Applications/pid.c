#include "parameter.h"
#include "pid.h"

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
	
	/* H(z)计算更新 */
	differential -> outputData[loopNum - 1] = 0.0f;
	for(uint8_t i = 0;i < loopNum;i++){
		differential -> outputData[loopNum - 1] +=	\
		differential -> coefficient -> xCoefficient[i] * differential -> inputData[loopNum - 1 - i];
		
		if(i != loopNum - 1)
			differential -> outputData[loopNum - 1] -=	\
			differential -> coefficient -> yCoefficient[i + 1] * differential -> outputData[loopNum - 1 - i - 1];
	}

	/* x(n) 序列保存 */
	for(uint8_t i = 0;i < loopNum - 1;i++){
			differential->inputData[i] = differential -> inputData[i + 1];
	}
	
	/* y(n) 序列保存 */
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
    //如果地址存在输入，则取地址内的值，没有则取1
    float f = (pid->fGain) ? *pid->fGain : 1.0f;															

    error = setpoint - position;

    //计算比例项                                       
    pid->pTerm_1 = p * error;
    if (pid->pTerm_1 > *pid->pMax) {
			pid->pTerm_1 = *pid->pMax;
    }
    else if (pid->pTerm_1 < -*pid->pMax) {
			pid->pTerm_1 = -*pid->pMax;
    }
    //用适当的极限计算积分状态   
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

    //微分																															
    if (pid->dGain) { //如果存在微分项																													
			error = -position;
        
            //在此处去除时间系数
			pid->dTerm_1 = (d * f) * (error - pid->dState);													
			pid->dState += f * (error - pid->dState);
			if (pid->dTerm_1 > *pid->dMax) {
					pid->dTerm_1 = *pid->dMax;
			}
			else if (pid->dTerm_1 < -*pid->dMax) {
					pid->dTerm_1 = -*pid->dMax;
			}
    }
    else {  //不存在则微分输出取0                       
			pid->dTerm_1 = 0.0f;																										
    }

    pid->pv_1 = position;
    pid->sp_1 = setpoint;
    pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;

    //给PID输出限幅
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




