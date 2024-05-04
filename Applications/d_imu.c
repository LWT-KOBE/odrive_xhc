#include "application.h"
#include "driver.h"
#include "bsp_iic.h"

imusensorStruct_t imuSensorData;
uint8_t gyroFsr = BMI088_FSR_2000DPS;
uint8_t accFsr = BMI088_FSR_3G;


void imuSensorDataUpdate(void){
	imuSensorData.rawGyo.x = meanRecursiveFilter(BMI088Data.gyroscope[0].s16_temp,0);
	imuSensorData.rawGyo.y = meanRecursiveFilter(BMI088Data.gyroscope[1].s16_temp,1);
	imuSensorData.rawGyo.z = meanRecursiveFilter(BMI088Data.gyroscope[2].s16_temp,2); 
	
	
	imuSensorData.rawAcc.x = (float)LowPass_Filter(BMI088Data.acceleration[0].s16_temp,&sensorRotate.lowPassFilterData[ACCEL_X_1000HZ_LOWPASS]);	
	imuSensorData.rawAcc.y = (float)LowPass_Filter(BMI088Data.acceleration[1].s16_temp,&sensorRotate.lowPassFilterData[ACCEL_Y_1000HZ_LOWPASS]);
	imuSensorData.rawAcc.z = (float)LowPass_Filter(BMI088Data.acceleration[2].s16_temp,&sensorRotate.lowPassFilterData[ACCEL_Z_1000HZ_LOWPASS]);	
}

void imuSensorFaultCheck(void){
	if(imuSensorData.deviceCNTR < IMU_CNTR){
		digitalIncreasing(&imuSensorData.dataFault);
	}
	else{
		digitalClan(&imuSensorData.dataFault);
	}
	//�������50ms���������ݶ�����������˵������GG��
	if(imuSensorData.dataFault > ERROR_TIME){
		imuSensorData.deviceInitState = false;
		digitalClan(&imuSensorData.dataFault);
	}
}

static void dimuUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){
		
	
		if(imuSensorData.deviceInitState){
			vTaskDelayUntil(&xLastWakeTime,DIMU_NORMAL_PERIOD);
			
		
			//��ȡ����������
			imuSensorData.deviceCNTR = BMI088_read();	

			//�Դ��������ݽ��г�������
			imuSensorDataUpdate(); 
			sensorProcessUpdate(&imuSensorData,&imuSensorData.rawAcc,&imuSensorData.rawGyo,NULL,&BMI088Data.temperature);
			if(!((imuSensorData.loops + 1) % 4)){
				if(imuSensorData.MagFlag==IST8310_NO_ERROR)
					IST8310_Updata(&imuSensorData);
			
				sensorProcessUpdate(&imuSensorData,&imuSensorData.rawAcc,&imuSensorData.rawGyo,&imuSensorData.rawMag,NULL);
				
			}			 			
					
			if(getsupervisorData()->state & STATE_IMUCALI){
				//imuУ׼����
				dIMUTare();                                   		
			}
			imuSensorFaultCheck();
		}
		else{
			//���û�ж�ȡ��ICM42605����ÿ��500ms��������һ��
			imuSensorData.deviceInitState = BMI088_init(&gyroFsr,&accFsr);

		}
		digitalIncreasing(&imuSensorData.loops);
	}
}

void dimuInit(void){
	

	imuSensorData.ImuFlag = BMI088_init(&gyroFsr,&accFsr);
	imuSensorData.MagFlag = IST8310_Init();	
	
	initAccLowPassFilter();	
	getsupervisorData()->taskEvent[IMU_TASK] = xTaskCreate(dimuUpdateTask,"DIMU",DIMU_STACK_SIZE,NULL,DIMU_PRIORITY,&imuSensorData.xHandleTask);
}

void dIMUTare(void){
	static float acc[3], gyo[3];
	
	int i;
	imuSensorData.state = IMU_CAIL_BEING;
	// reset all parameters								//�������в���
	if(!imuSensorData.imuTareLoop){
		for (i = 0; i < 3; i++) {
			if(imuSensorData.accTare){
				acc[i] = 0.0f;
			}
			gyo[i] = 0.0f;									//��������
		}
	}

	
	if ((IMU_ACCX + IMU_ACCY + IMU_ACCZ) < (IMU_STATIC_STD * 10)){			//�����׼�����С��0.02
			if(getsupervisorData()->state & STATE_SENSOR_ERROR)
				supervisorStateSwitch(STATE_SENSOR_ERROR,DISABLE);
			if(imuSensorData.accTare){
				acc[0] += IMU_RAW_ACCX;
				acc[1] += IMU_RAW_ACCY;
				acc[2] += IMU_RAW_ACCZ;
			}
			gyo[0] += IMU_RAW_RATEX;
			gyo[1] += IMU_RAW_RATEY;
			gyo[2] += IMU_RAW_RATEZ;
			digitalIncreasing(&imuSensorData.imuTareLoop);

	}
	else{
		for (i = 0; i < 3; i++) {
			if(imuSensorData.accTare){
				acc[i] = 0.0f;
			}
			gyo[i] = 0.0f;									//��������
		}
		digitalClan(&imuSensorData.imuTareLoop);										//�����ͷ��ʼУ׼
		if(!(getsupervisorData()->state & STATE_SENSOR_ERROR))
			supervisorStateSwitch(STATE_SENSOR_ERROR,ENABLE);
	}	
	if(imuSensorData.imuTareLoop >= DIMU_TARE_LOOP){
		for(i = 0;i < 3;i++){
			if(imuSensorData.accTare){
				digitalClan(&imuSensorData.accBIAS[i]);
			}
			digitalClan(&imuSensorData.gyoBIAS[i]);
		}
		if(imuSensorData.accTare){
			imuSensorData.accBIAS[0] = -(acc[0] / imuSensorData.imuTareLoop);
			imuSensorData.accBIAS[1] = -(acc[1] / imuSensorData.imuTareLoop);
			imuSensorData.accBIAS[2] = (GRAVITY - (acc[2] / imuSensorData.imuTareLoop));
		}
		imuSensorData.gyoBIAS[0] = -(gyo[0] / imuSensorData.imuTareLoop);
		imuSensorData.gyoBIAS[1] = -(gyo[1] / imuSensorData.imuTareLoop);
		imuSensorData.gyoBIAS[2] = -(gyo[2] / imuSensorData.imuTareLoop);

			
		getConfigData()->imu_acc_bias_x = imuSensorData.accBIAS[0];
		getConfigData()->imu_acc_bias_y = imuSensorData.accBIAS[1];
		getConfigData()->imu_acc_bias_z = imuSensorData.accBIAS[2];

		getConfigData()->imu_gyo_bias_x = imuSensorData.gyoBIAS[0];
		getConfigData()->imu_gyo_bias_y = imuSensorData.gyoBIAS[1];
		getConfigData()->imu_gyo_bias_z = imuSensorData.gyoBIAS[2];		
						
					
		digitalClan(&imuSensorData.imuTareLoop);
		supervisorStateSwitch(STATE_IMUCALI,DISABLE);
		digitalHi(&getsupervisorData()->flashSave);
		imuSensorData.state = IMU_CAIL_FINISH;
		if(imuSensorData.accTare){
			digitalLo(&imuSensorData.accTare);
		}
	}
}

