#include "application.h"
#include "driver.h"
#include "ramp.h"
#include "math.h"

/*
***************************************************
�� �� ����	gimbalUpdate
��		�ܣ���̨�������
��ڲ�����	gimbalData.visionYawCmd				��������ϵ����
					gimbalData.visionPitchCmd			��������ϵ����
					gimbalData.autoMode		�л��Զ������־
�� �� ֵ��	��
Ӧ�÷�Χ��	�ⲿ����
��		ע��
***************************************************
*********************************************************/

gimbalStruct_t gimbalData;
gimbalStruct_t* getGimbalData(){
    return &gimbalData;
}

ramp_t yawVisionRamp = RAMP_GEN_DAFAULT;											//��̨б�³�ʼ��
ramp_t pitchVisionRamp = RAMP_GEN_DAFAULT;

static ramp_t yawRamp = RAMP_GEN_DAFAULT;											//��̨б�³�ʼ��
static ramp_t pitchRamp = RAMP_GEN_DAFAULT;

int8_t getInstallDirect(uint8_t installPara,bool type){
	int8_t res;
	if(type)
		res = (installPara & 0x02)?-1:1;
	else
		res = (installPara & 0x01)?-1:1;
	return res;
}


u8 acc;
void gimbalUpdate(void){
	getGimbalCtrlDate();															
	
	switch (robotMode){
		case MODE_INIT:           
			gimbalInitHandle();   
			break;//��ʼ��ģʽ
		case MODE_RC: {
			if(gimbalData.ctrlMode == GIMBAL_INIT){
				gimbalInitHandle();
			}
			else if(gimbalData.ctrlMode == GIMBAL_NORMAL){
				gimbalFollowHandle();
			}
			else if(gimbalData.ctrlMode == GIMBAL_STOP){
				gimbalStopHandle();
			}
			break;
		}
		case MODE_STOP:{	 
			gimbalStopHandle();	 
			break;//����ģʽ
		}
		case MODE_RELAX:{        
			gimbalRelaxHandle();  
			break;//�������Ȩ
		}
		default:                                        
			break;
	}

	gimbalData.time[0] = getClockCount();
	gimbalData.intervalTime = (float)(gimbalData.time[0] - gimbalData.time[1]);
	gimbalData.time[1] = gimbalData.time[0];
	
	
	if(isnan(gimbalData.yawAngleOut)||isnan(gimbalData.pitchAngleOut)){
		pidZeroState(gimbalData.pitchAnglePID);
		pidZeroState(gimbalData.pitchSpeedPID);
		pidZeroState(gimbalData.yawAnglePID);
		pidZeroState(gimbalData.yawSpeedPID);

		digitalClan(&gimbalData.yawAngleOut);
		digitalClan(&gimbalData.pitchAngleOut);
		digitalClan(&gimbalData.yawSpeedOut);
		digitalClan(&gimbalData.pitchSpeedOut);		
		
		gimbalData.pitchAngleRef = gimbalData.pitchAngleFbd;				
		gimbalData.yawAngleRef = gimbalData.yawAngleFbd;	
	}
	

	gimbalData.yawAngleOut   = pidUpdate(gimbalData.yawAnglePID, gimbalData.yawAngleRef, gimbalData.yawAngleFbd,gimbalData.intervalTime); 
	gimbalData.yawSpeedRef   = -gimbalData.yawAngleOut;
	

	gimbalData.yawAngleOut   = pidUpdate(gimbalData.yawAnglePID, gimbalData.yawAngleRef, gimbalData.yawAngleFbd,gimbalData.intervalTime); 
	gimbalData.yawSpeedRef   = -gimbalData.yawAngleOut;
	
	gimbalData.pitchAngleOut = pidUpdate(gimbalData.pitchAnglePID, gimbalData.pitchAngleRef, gimbalData.pitchAngleFbd,gimbalData.intervalTime);
	gimbalData.pitchSpeedRef = -gimbalData.pitchAngleOut;
	
	
	gimbalData.pitchSpeedFbd = IMU_RATEY;                    // �ٶȻ�������imu���ٶ��ٶ�
	gimbalData.yawSpeedFbd   = IMU_RATEZ;
	
	gimbalData.pitchSpeedOut = -getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_TURN) * \
								pidUpdate(gimbalData.pitchSpeedPID,gimbalData.pitchSpeedRef,gimbalData.pitchSpeedFbd,gimbalData.intervalTime);
	gimbalData.yawSpeedOut   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_TURN) * \
							   pidUpdate(gimbalData.yawSpeedPID,gimbalData.yawSpeedRef,gimbalData.yawSpeedFbd,gimbalData.intervalTime);
	
	if(robotMode == MODE_RELAX){
		digitalClan(&gimbalData.yawSpeedOut);
		digitalClan(&gimbalData.pitchSpeedOut);
		digitalClan(&gimbalData.rollSpeedOut);
	}
}


float shiftAngle = 0;
static void gimbalFollowHandle(void){
	digitalClan(&gimbalData.initFinishFlag);
	digitalClan(&gimbalData.motorFlag);

	gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
	gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
	gimbalData.yawAngleRef += remoteControlData.yawGyroTarget ;
	gimbalData.pitchAngleRef += remoteControlData.pitchGyroTarget;


	//pitch��Ƕ��޷�
	//gimbalData.pitchAngleRef = constrainFloat(gimbalData.pitchAngleRef,getConfigData()->pitchMinRange,getConfigData()->pitchMaxRange); 
}

static void gimbalInitHandle(void){
	static f32_t pitchRampAngle;
	static f32_t yawRampAngle;
	
	static TickType_t xLastWakeTime = 0;
    //ÿһ�ν����ʼ��ģʽ����ȡ��ǰ�ǶȲ���ʼ��б�º���
	if(lastRobotMode != robotMode||gimbalData.ctrlMode != gimbalData.lastCtrlMode){																				
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
        //��ȡpitch��imu��ǰ�Ƕ�
		pitchRampAngle = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  				
        //��ȡyaw���̵�ǰ�Ƕ�
		yawRampAngle   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;   					

	}
    //����pitch�ᷴ��//pitch�����
	gimbalData.pitchAngleFbd = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;	
	gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));																			
    //����yaw�ᷴ��//yaw�����
	gimbalData.yawAngleFbd = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;   		
	gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));       																		

	//�ȴ�pitch��yaw�����
	if(((gimbalData.yawMotorAngle < 0.5f && gimbalData.yawMotorAngle > -0.5f)\
	&&(gimbalData.pitchMotorAngle < 0.5f && gimbalData.pitchMotorAngle > -0.5f))\
	|| xTaskGetTickCount() - xLastWakeTime > getConfigData()->backCenterTime * 2){
        //������̨ģʽΪ����ģʽ
		gimbalData.ctrlMode  = GIMBAL_NORMAL;

		
		//����ӳ�ʼ��ģʽ������ģʽ�������ǽǶ�	
		gimbalData.pitchAngleSave = gimbalData.pitchGyroAngle;		
		gimbalData.yawAngleSave = gimbalData.yawGyroAngle;
		//����ֵ����
		digitalClan(&gimbalData.yawAngleRef);                    	
		digitalClan(&gimbalData.pitchAngleRef);
		
		//��̨������ɱ�־��һ
		digitalHi(&gimbalData.initFinishFlag);                   	
	}
}


static void gimbalStopHandle(void){
	static f32_t pitchRampAngle;
	static f32_t yawRampAngle;
	static TickType_t xLastWakeTime = 0;
	//ÿһ�ν����ʼ��ģʽ����ȡ��ǰ�ǶȲ���ʼ��б�º���
	if(lastRobotMode != robotMode || gimbalData.ctrlMode != gimbalData.lastCtrlMode){																						
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
		//��ȡpitch���̵�ǰ�Ƕ�
		pitchRampAngle = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  			
		//��ȡyaw���̵�ǰ�Ƕ�			
		yawRampAngle   = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;    						

	}
	//����pitch�ᷴ��	
	gimbalData.pitchAngleFbd = getInstallDirect(PITCH_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.pitchMotorAngle;				
	//����yaw�ᷴ��
	gimbalData.yawAngleFbd = getInstallDirect(YAW_INSTALL_CONFIG,INSTALL_ENCODER) * gimbalData.yawMotorAngle;  						

	if(gimbalData.motorFlag){
		//pitch�����
		gimbalData.pitchAngleRef = pitchRampAngle + (gimbalData.pitchAngleStop - pitchRampAngle) * LinearRampCalc(&pitchRamp,1);			
		//yaw�����	  
		gimbalData.yawAngleRef = yawRampAngle + (gimbalData.yawAngleStop - yawRampAngle) * LinearRampCalc(&yawRamp,1);					

	}
	else{
		digitalClan(&gimbalData.pitchAngleStop);
		digitalClan(&gimbalData.yawAngleStop);
        //pitch�����
		gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));													
		//yaw�����
		gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));

	}

  	//�ȴ�pitch��yaw�����
	if(((gimbalData.yawMotorAngle < gimbalData.yawAngleStop + 2.0f && \
        gimbalData.yawMotorAngle > gimbalData.yawAngleStop - 2.0f)\
		&& (gimbalData.pitchMotorAngle < gimbalData.pitchAngleStop + 2.0f && \
        gimbalData.pitchMotorAngle > gimbalData.pitchAngleStop - 2.0f))\
		|| xTaskGetTickCount() - xLastWakeTime > getConfigData()->backCenterTime * 2 ){
        //����ӳ�ʼ��ģʽ������ģʽ�������ǽǶ�
		gimbalData.pitchAngleSave = gimbalData.pitchGyroAngle;								
		gimbalData.yawAngleSave = gimbalData.yawGyroAngle;

		if(!gimbalData.motorFlag){
			//����ֵ����
			digitalClan(&gimbalData.yawAngleRef);                   						
			digitalClan(&gimbalData.pitchAngleRef);
		}		
	}
}

static void gimbalRelaxHandle(void){
	 digitalLo(&gimbalData.initFinishFlag);
	 digitalLo(&gimbalData.motorFlag);
	 pidZeroState(gimbalData.pitchAnglePID);
	 pidZeroState(gimbalData.pitchSpeedPID);
	 pidZeroState(gimbalData.yawAnglePID);
	 pidZeroState(gimbalData.yawSpeedPID);

	 gimbalData.ctrlMode = GIMBAL_INIT;
}


//б�º�����ʼ��
void gimbalRampInit(void){ 													  				
    RampInit(&pitchRamp, getConfigData()->backCenterTime/2);
    RampInit(&yawRamp, getConfigData()->backCenterTime/2);
    RampInit(&pitchVisionRamp, 100);
    RampInit(&yawVisionRamp, 100);
}


void gimbalInit(void){
    
    gimbalData.ctrlMode  = GIMBAL_RELAX;
	gimbalRampInit();
    
	gimbalData.pitchAnglePID = pidInit(&getConfigData()->pitchAnglePID);    
	gimbalData.yawAnglePID = pidInit(&getConfigData()->yawAnglePID);    
	gimbalData.pitchSpeedPID = pidInit( &getConfigData()->pitchRatePID);        
    gimbalData.yawSpeedPID = pidInit(&getConfigData()->yawRatePID);    
	digitalLo(&getGimbalData()->initFinishFlag);    
    
}








