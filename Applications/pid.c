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


/**************************************************************************
函数功能：限幅函数
入口参数：insert幅值,low最小值,high最大值
返回  值：限幅后的值
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
函数功能：角度转向PID
入口参数：angle_target期望角度,angle_current当前角度,limit输出限制（速度）
返回  值：限幅后的值
**************************************************************************/
float PID_angel(float angle_target, float angle_current,float limit){
	
	//PID输出值
	static float motor_out;
	//定义当前偏差,上一个偏差,累差值
	static float bias,bias_last,err_num,Kp = 0.075f,Ki = 0.0000f,Kd = 0.5f;
	
	//获得偏差值,用于微分,起到阻尼的作用,防超调
	bias = angle_target - angle_current;
	//误差限幅，限制转向角度的最大误差值,
	bias = target_limit_float(bias,-90,90);
	//还没完成转向,标志位为0
	//获得累差值,用于积分,接近真实值
	err_num += bias;
	
	//PID计算电机输出PWM值
	motor_out = Kp*bias + Ki*err_num + Kd*(bias-bias_last);
	
	//记录上次偏差
	bias_last = bias;
	
	
	//限制最大输出
	if(motor_out > limit)
		motor_out = limit;
	if(motor_out < -limit)
		motor_out = -limit;
	
//	//提前刹停,利用惯性转完
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

	//返回脉冲频率值
	return motor_out;
}

///**************************************************************************
//函数功能：位置式PID控制器
//入口参数：实际位置，目标位置
//返回  值：电机PWM
//根据位置式离散PID公式
//位置式PID的三个参数: Kp:提高响应速度 Ki:稳态 Kd:抑制震荡
//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
//e(k)代表本次偏差 
//e(k-1)代表上一次的偏差  
//∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,...,k;
//pwm代表输出
//**************************************************************************/
//float XC_Position_PID(float reality,float target)
//{ 	
//    static float Bias,Pwm,Last_Bias,Integral_bias=0;
//    
//    Bias=target-reality;                            /* 计算偏差 */
//    Integral_bias+=Bias;	                        /* 偏差累积 */
//    
//    if(Integral_bias> 360) Integral_bias = 360;   	/* 积分限幅 */
//    if(Integral_bias<-360) Integral_bias =-360;
//    
//    Pwm = (Angle_Position_KP*Bias)                        /* 比例环节 */
//         +(Angle_Position_KI*Integral_bias)               /* 积分环节 */
//         +(Angle_Position_KD*(Bias-Last_Bias));           /* 微分环节 */
//    
//    Last_Bias=Bias;                                 	/* 保存上次偏差 */
//	
//														/* 输出限幅 */
//    return Pwm;                                     	/* 输出结果 */
//}

///**************************************************************************
//函数功能：增量PID控制器
//入口参数：实际值，目标值
//返回  值：电机PWM
//根据增量式离散PID公式 
//增量式PID的三个参数: Kp:抑制震荡 Ki:提高响应速度 Kd:稳态
//pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
//e(k)代表本次偏差 
//e(k-1)代表上一次的偏差  以此类推 
//pwm代表增量输出
//**************************************************************************/
//float XC_Incremental_PID(float reality,float target)
//{ 	
//	 static float Bias,Pwm,Last_bias=0,Prev_bias=0;
//    
//	 Bias=target-reality;                                   /* 计算偏差 */
//    
//	 Pwm += (Angle_Incremental_KP*(Bias-Last_bias))               /* 比例环节 */
//           +(Angle_Incremental_KI*Bias)                           /* 积分环节 */
//           +(Angle_Incremental_KD*(Bias-2*Last_bias+Prev_bias));  /* 微分环节 */ 
//    
//     Prev_bias=Last_bias;                                   /* 保存上上次偏差 */
//	 Last_bias=Bias;	                                    /* 保存上一次偏差 */
//		
//	 if(fabs(Bias)<0.75){
//		 Prev_bias = 0;
//		 Last_bias = 0;
//		 Pwm = 0;
//	 }
//    
//	if(Pwm >  75) Pwm =  75;
//	if(Pwm < -75) Pwm = -75;
//	return Pwm;                                            /* 输出结果 */
//}


