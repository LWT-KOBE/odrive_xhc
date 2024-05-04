#include "application.h"

runStruct_t runData;
void runUpdateTask(void *Parameters){
  volatile uint32_t axis = 0;
	while(true){
		xEventGroupWaitBits(taskInitData.eventGroups,SENSOR_FLAG,pdTRUE,pdTRUE,portMAX_DELAY);	//�����¼���־��
		runData.time[1] = getClockCount();
		/*---------------------- ����Ϊһ�������˲� ----------------------*/
		//��¼��ʷ�ڼ��ٶȡ�������ѹ���Ķ����Ա�����ƽ����Ŀ��			 											
		// acc
		runData.sumAcc[0] -= runData.accHist[0][runData.sensorHistIndex];												//��ȥ��һ����ʷ����
		runData.sumAcc[1] -= runData.accHist[1][runData.sensorHistIndex];
		runData.sumAcc[2] -= runData.accHist[2][runData.sensorHistIndex];

		runData.accHist[0][runData.sensorHistIndex] = IMU_ACCX;																	//sensorHistIndexΪ��������ʷָ��
		runData.accHist[1][runData.sensorHistIndex] = IMU_ACCY;																	//��¼��ε���ʷ����
		runData.accHist[2][runData.sensorHistIndex] = IMU_ACCZ;

		runData.sumAcc[0] += runData.accHist[0][runData.sensorHistIndex];												//������ε���ʷ����
		runData.sumAcc[1] += runData.accHist[1][runData.sensorHistIndex];
		runData.sumAcc[2] += runData.accHist[2][runData.sensorHistIndex];
#ifdef USE_MAG
		// mag																								
		runData.sumMag[0] -= runData.magHist[0][runData.sensorHistIndex];
		runData.sumMag[1] -= runData.magHist[1][runData.sensorHistIndex];
		runData.sumMag[2] -= runData.magHist[2][runData.sensorHistIndex];

		runData.magHist[0][runData.sensorHistIndex] = IMU_MAGX;
		runData.magHist[1][runData.sensorHistIndex] = IMU_MAGY;
		runData.magHist[2][runData.sensorHistIndex] = IMU_MAGZ;

		runData.sumMag[0] += runData.magHist[0][runData.sensorHistIndex];
		runData.sumMag[1] += runData.magHist[1][runData.sensorHistIndex];
		runData.sumMag[2] += runData.magHist[2][runData.sensorHistIndex];
#endif	
		// gyo																								
		runData.sumGyo[0] -= runData.gyoHist[0][runData.sensorHistIndex];
		runData.sumGyo[1] -= runData.gyoHist[1][runData.sensorHistIndex];
		runData.sumGyo[2] -= runData.gyoHist[2][runData.sensorHistIndex];
		
		runData.gyoHist[0][runData.sensorHistIndex] = IMU_RATEX;
		runData.gyoHist[1][runData.sensorHistIndex] = IMU_RATEY;
		runData.gyoHist[2][runData.sensorHistIndex] = IMU_RATEZ;
		
		runData.sumGyo[0] += runData.gyoHist[0][runData.sensorHistIndex];
		runData.sumGyo[1] += runData.gyoHist[1][runData.sensorHistIndex];
		runData.sumGyo[2] += runData.gyoHist[2][runData.sensorHistIndex];
		
		runData.sensorHistIndex = (runData.sensorHistIndex + 1) % RUN_SENSOR_HIST;	
		//�ߵ�UKF�������
		navUkfInertialUpdate();		
		if (!((runData.loops+1) % 20)) {		
			simDoAccUpdate(runData.sumAcc[0]*(1.0f / (float)(RUN_SENSOR_HIST)), runData.sumAcc[1]*(1.0f / (float)(RUN_SENSOR_HIST)), runData.sumAcc[2]*(1.0f / (float)(RUN_SENSOR_HIST)));
		}
#ifdef USE_MAG
		else if (!((runData.loops+3) % 20)) {
			simDoMagUpdate(runData.sumMag[0]*(1.0f / (float)(RUN_SENSOR_HIST)), runData.sumMag[1]*(1.0f / (float)(RUN_SENSOR_HIST)), runData.sumMag[2]*(1.0f / (float)(RUN_SENSOR_HIST)));
		}
#endif
		arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &runData.stdAccX);
		arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &runData.stdAccY);
		arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &runData.stdAccZ);
//		if(wiredControlData.cmd.robotMode == MODE_RELAX){
			if ((runData.stdAccX + runData.stdAccY + runData.stdAccZ) < (IMU_STATIC_STD / 10)){				//�������׼��С��0.02ʱ						
				if (!((axis + 0) % 3))
					navUkfZeroRate(IMU_RATEX, 0);
				else if (!((axis + 1) % 3))
					navUkfZeroRate(IMU_RATEY, 1);
				else if (!((axis + 2) % 3))
					navUkfZeroRate(IMU_RATEZ, 2);
				axis++;
			}
//		}
		navUkfFinish();																																						//����navUKF�Ĺ��̣�������Ԫ�������ŷ����
		digitalIncreasing(&runData.loops);
		digitalIncreasing(&runData.CNTR.s16_temp);
			
		//�Ƕ�����ʹ��	
		//wiredTransTypeSwitch(TRANS_ADD_ANGLE, ENABLE);
		runData.time[0] = getClockCount();
		runData.executionTime = (float)(runData.time[0] - runData.time[1]);	
	}
}

void runInit(void){
	float acc[3], mag[3];
	int i;

	memset((void *)&runData, 0, sizeof(runData));
	/*-----------�ߵ���������-----*/
	getsupervisorData()->taskEvent[RUN_TASK] = xTaskCreate(runUpdateTask,"RUN",RUN_STACK_SIZE,NULL,RUN_PRIORITY,&runData.xHandleTask);		

	acc[0] = IMU_ACCX;
	acc[1] = IMU_ACCY;
	acc[2] = IMU_ACCZ;

	mag[0] = IMU_MAGX;
	mag[1] = IMU_MAGY;
	mag[2] = IMU_MAGZ;

	// initialize sensor history
	for (i = 0; i < RUN_SENSOR_HIST; i++){
			runData.accHist[0][i] = acc[0];
			runData.accHist[1][i] = acc[1];
			runData.accHist[2][i] = acc[2];
			runData.magHist[0][i] = mag[0];
			runData.magHist[1][i] = mag[1];
			runData.magHist[2][i] = mag[2];

			runData.sumAcc[0] += acc[0];
			runData.sumAcc[1] += acc[1];
			runData.sumAcc[2] += acc[2];
			runData.sumMag[0] += mag[0];
			runData.sumMag[1] += mag[1];
			runData.sumMag[2] += mag[2];
	}
	runData.sensorHistIndex = 0;
	runData.bestHacc = 1000.0f;
	runData.accMask = 1000.0f;
	runData.initFlag = true;
}


