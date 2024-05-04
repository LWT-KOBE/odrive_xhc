#include "nav_ukf.h"
#include "imu.h"
#include "run.h"
#include "processing.h"
#include "clockcount.h"
#include "supervisor.h"

//���̼�������Э�����״̬ת�ƾ�����ʵ�ʹ���֮������
#define UKF_ACC_BIAS_Q          +1.3317e-03	     					
#define UKF_GYO_BIAS_Q          +4.5256e-02    
#define UKF_QUAT_Q              +5.4005e-04   
#define UKF_ACC_BIAS_V          +7.8673e-07    
#define UKF_GYO_BIAS_V          +4.0297e-09    
#define UKF_RATE_V              +1.7538e-05    
#define UKF_ACC_N               +6.3287e-05   
#define UKF_DIST_N              +9.7373e-03  
#define UKF_MAG_N               +5.2355e-01    

#ifndef __CC_ARM
#include <intrinsics.h>
#endif

navUkfStruct_t navUkfData;

void navUkfNormalizeVec3(float *vr, float *v) {
    float norm;

    norm = __sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    vr[0] = v[0] / norm;
    vr[1] = v[1] / norm;
    vr[2] = v[2] / norm;
}

void navUkfNormalizeQuat(float *qr, float *q) {
    float norm;

    norm = 1.0f / __sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    qr[0] *= norm;
    qr[1] *= norm;
    qr[2] *= norm;
    qr[3] *= norm;
}

void crossVector3(float *vr, float *va, float *vb) {
    vr[0] = va[1] * vb[2] - vb[1] * va[2];
    vr[1] = va[2] * vb[0] - vb[2] * va[0];
    vr[2] = va[0] * vb[1] - vb[0] * va[1];
}

float dotVector3(float *va, float *vb) {
    return va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2];
}

void navUkfRotateVectorByQuat(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    navUkfRotateVectorByQuat(vr, v, qc);			//����Ԫ����ת��ȡ������в��
}
//�����ٶ�ת����������ϵ
void navUkfRotateVecByMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[0*3 + 1]*v[1] + m[0*3 + 2]*v[2];
    vr[1] = m[1*3 + 0]*v[0] + m[1*3 + 1]*v[1] + m[1*3 + 2]*v[2];
    vr[2] = m[2*3 + 0]*v[0] + m[2*3 + 1]*v[1] + m[2*3 + 2]*v[2];
}

static void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}
/*--------------- ����Ԫ��������������� ---------------*/
static void navUkfQuatToMatrix(float *m, float *q, int normalize) {
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
    float tmp1, tmp2;
    float invs;

    // get the invert square length					��ȡ����ƽ������
    if (normalize)												//����Ѿ���һ����Ҳ����Ԫ�����й�һ��
			invs = 1.0f / (sqx + sqy + sqz + sqw);
    else
			invs = 1.0f;

    // rotation matrix is scaled by inverse square length		����ƽ������������ת���󣿣���
    m[0*3 + 0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1*3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2*3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1*3 + 0] = 2.0f * (tmp1 + tmp2) * invs;
    m[0*3 + 1] = 2.0f * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2*3 + 0] = 2.0f * (tmp1 - tmp2) * invs;
    m[0*3 + 2] = 2.0f * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2*3 + 1] = 2.0f * (tmp1 + tmp2) * invs;
    m[1*3 + 2] = 2.0f * (tmp1 - tmp2) * invs;
}

void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll) {
    float q0, q1, q2, q3;

    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

	  *yaw = -atan2f((2.0f * (q1 * q2 + q0 * q3)), (1.0f - 2.0f * q2 * q2 - 2.0f * q3 *q3));
    *pitch = (0.5f * M_PI) - acosf(2.0f *(q1 * q3 - q0 *q2));
    *roll = atan2f((2.0f * (q0 * q1 + q3 * q2)), (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2));
}

// result and source can be the same				//������������һ��
//�������Ҿ���
static void navUkfRotateQuat(float *qOut, float *qIn, float *rate) {
    float q[4];
    float r[3];

    r[0] = rate[0] * -0.5f;
    r[1] = rate[1] * -0.5f;
    r[2] = rate[2] * -0.5f;

    q[0] = qIn[0];
    q[1] = qIn[1];
    q[2] = qIn[2];
    q[3] = qIn[3];

    // rotate
    qOut[0] =       q[0] + r[0]*q[1] + r[1]*q[2] + r[2]*q[3];
    qOut[1] = -r[0]*q[0] +      q[1] - r[2]*q[2] + r[1]*q[3];
    qOut[2] = -r[1]*q[0] + r[2]*q[1] +      q[2] - r[0]*q[3];
    qOut[3] = -r[2]*q[0] - r[1]*q[1] + r[0]*q[2] +      q[3];
}
/*------------------- ��������൱��״̬�Ƶ���ʽ ------------------*/
void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {				
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];
    float q[4];
    int i;

    // assume out == in									//�ٶ������������
    out = in;

		for (i = 0; i < n; i++){

			// create rot matrix from current quat			//��Ԫ���������Ҿ���
			q[0] = in[UKF_STATE_Q1*n + i];
			q[1] = in[UKF_STATE_Q2*n + i];
			q[2] = in[UKF_STATE_Q3*n + i];
			q[3] = in[UKF_STATE_Q4*n + i];
			navUkfQuatToMatrix(mat3x3, q, 1);						//�����������

			// acc											//�м������ٶ� = ������ٶ�(ԭʼ��) +����ƫ����ٶ�
			tmp[0] = u[0] + in[UKF_STATE_ACC_BIAS_X*n + i];
			tmp[1] = u[1] + in[UKF_STATE_ACC_BIAS_Y*n + i];
			tmp[2] = u[2] + in[UKF_STATE_ACC_BIAS_Z*n + i];

			// rotate acc to world frame					//�����ٶ�ת����������ϵ
			navUkfRotateVecByMatrix(acc, tmp, mat3x3);
			acc[2] += GRAVITY;								//Z���������

			// acc bias										//������ٶ�ƫ�� = ������ٶ�ƫ��+ ����*ʱ�� 
			out[UKF_STATE_ACC_BIAS_X*n + i] = in[UKF_STATE_ACC_BIAS_X*n + i] + noise[UKF_V_NOISE_ACC_BIAS_X*n + i] * dt;
			out[UKF_STATE_ACC_BIAS_Y*n + i] = in[UKF_STATE_ACC_BIAS_Y*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Y*n + i] * dt;
			out[UKF_STATE_ACC_BIAS_Z*n + i] = in[UKF_STATE_ACC_BIAS_Z*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Z*n + i] * dt;

			// rate = rate + bias + noise					//������ٶ� = (������ٶ�(ԭʼ��) + ����ƫ����ٶ� + ����)*ʱ�� 
			rate[0] = (u[3] + in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_RATE_X*n + i]) * dt;
			rate[1] = (u[4] + in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_RATE_Y*n + i]) * dt;
			rate[2] = (u[5] + in[UKF_STATE_GYO_BIAS_Z*n + i] + noise[UKF_V_NOISE_RATE_Z*n + i]) * dt;

			// rotate quat												//��ת��Ԫ��
			navUkfRotateQuat(q, q, rate);					//��һ����ɽ��ٶȶ���Ԫ���ĸ���
			out[UKF_STATE_Q1*n + i] = q[0];
			out[UKF_STATE_Q2*n + i] = q[1];
			out[UKF_STATE_Q3*n + i] = q[2];
			out[UKF_STATE_Q4*n + i] = q[3];

			// gbias															//������ٶ�ƫ�� = ������ٶ�ƫ�� + ���ٶ�����*ʱ�� 
			out[UKF_STATE_GYO_BIAS_X*n + i] = in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_GYO_BIAS_X*n + i] * dt;
			out[UKF_STATE_GYO_BIAS_Y*n + i] = in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Y*n + i] * dt;
			out[UKF_STATE_GYO_BIAS_Z*n + i] = in[UKF_STATE_GYO_BIAS_Z*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Z*n + i] * dt;
    }
}

void navUkfRateUpdate(float *u, float *x, float *noise, float *y) {
	y[0] = -x[UKF_STATE_GYO_BIAS_X+(int)u[0]] + noise[0];			//�����u�����Զ�ѡ��״̬�������±�
}

void navUkfAccUpdate(float *u, float *x, float *noise, float *y) {
	navUkfRotateVectorByRevQuat(y, navUkfData.v0a, &x[UKF_STATE_Q1]);			//����Ԫ�����в��
	y[0] += noise[0];
	y[1] += noise[1];
	y[2] += noise[2];
}

void navUkfMagUpdate(float *u, float *x, float *noise, float *y) {
	navUkfRotateVectorByRevQuat(y, navUkfData.v0m, &x[UKF_STATE_Q1]);			//����Ԫ�����в��
	y[0] += noise[0];
	y[1] += noise[1];
	y[2] += noise[2];
}
/*-------------------- ����navUKF���㣬������Ԫ����ֵ����ȡŷ���� ----------------------*/
void navUkfFinish(void) {
	navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);														//��׼����Ԫ��
	navUkfQuatExtractEuler(&UKF_Q1, &navUkfData.yaw.float_temp, &navUkfData.pitch.float_temp, &navUkfData.roll.float_temp);		//��Ԫ����ȡŷ����
	navUkfData.yaw.float_temp = compassNormalize(navUkfData.yaw.float_temp * RAD_TO_DEG);
	navUkfData.pitch.float_temp *= RAD_TO_DEG;
	navUkfData.roll.float_temp *= RAD_TO_DEG;
	//    x' = x cos f - y sin f
	//    y' = y cos f + x sin f
	navUkfData.yawCos = cosf(navUkfData.yaw.float_temp * DEG_TO_RAD);
	navUkfData.yawSin = sinf(navUkfData.yaw.float_temp * DEG_TO_RAD);
	
}
/*----------------- �˺���Ϊ����UKF���Ը��� -----------------*/
void navUkfInertialUpdate(void) {
	float u[6];

	u[0] = IMU_ACCX;																			//��ȡ��������
	u[1] = IMU_ACCY;
	u[2] = IMU_ACCZ;

	u[3] = IMU_RATEX;
	u[4] = IMU_RATEY;
	u[5] = IMU_RATEZ;
	
	navUkfData.time[0] = getClockCount();
	navUkfData.intervalTime = (float)(navUkfData.time[0] - navUkfData.time[1]);
	navUkfData.time[1] = navUkfData.time[0];

	srcdkfTimeUpdate(navUkfData.kf, u, navUkfData.intervalTime);//UKFʱ����£��ڶ�������Ϊ״̬����������������Ϊ��������
}

void navUkfZeroRate(float rate, int axis) {
    float noise[1];        // measurement variance		��������
    float y[1];            // measurment(s)				������
    float u[1];		   // user data						�û�����

    noise[0] = 0.00001f;
    y[0] = rate;
    u[0] = (float)axis;

    srcdkfMeasurementUpdate(navUkfData.kf, u, y, 1, 1, noise, navUkfRateUpdate);
}

void simDoAccUpdate(float accX, float accY, float accZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    // normalize vector						//��һ������
    norm =  __sqrtf(accX*accX + accY*accY + accZ*accZ);
		navUkfData.accNorm = __sqrtf(accX*accX + accY*accY + accZ*accZ);
    y[0] = accX / norm;
    y[1] = accY / norm;
    y[2] = accZ / norm;

    noise[0] = UKF_ACC_N + fabsf(GRAVITY - norm) * UKF_DIST_N;
    noise[1] = noise[0];
    noise[2] = noise[0];

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfAccUpdate);
}

void simDoMagUpdate(float magX, float magY, float magZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    noise[0] = UKF_MAG_N;
    noise[1] = noise[0];
    noise[2] = noise[0];

    // normalize vector
    norm = 1.0f / __sqrtf(magX*magX + magY*magY + magZ*magZ);
    y[0] = magX * norm;
    y[1] = magY * norm;
    y[2] = magZ * norm;

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfMagUpdate);
}

void navUkfResetBias(void) {
    // acc bias
    UKF_ACC_BIAS_X = 0.0f;
    UKF_ACC_BIAS_Y = 0.0f;
    UKF_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_GYO_BIAS_X = 0.0f;
    UKF_GYO_BIAS_Y = 0.0f;
    UKF_GYO_BIAS_Z = 0.0f;
}

void imuQuasiStatic(int n) {
	float stdX, stdY, stdZ;
	float vX[n];
	float vY[n];
	float vZ[n];
	int i, j;
	i = 0;
	j = 0;
	do{
		xEventGroupWaitBits(taskInitData.eventGroups,UKF_INIT_FLAG,pdTRUE,pdTRUE,portMAX_DELAY);
		vX[j] = IMU_ACCX;
		vY[j] = IMU_ACCY;
		vZ[j] = IMU_ACCZ;
		j = (j + 1) % n;
		//���ټ���n�����ݵı�׼��
		if (i >= n){
			arm_std_f32(vX, n, &stdX);
			arm_std_f32(vY, n, &stdY);
			arm_std_f32(vZ, n, &stdZ);
		}
		i++;
	}while (i < (int)(1.0f / DIMU_OUTER_DT) * IMU_STATIC_TIMEOUT && (i <= n || (stdX + stdY + stdZ) > IMU_STATIC_STD));		
	//ѭ��ֱ��������С��1000���׼���ܺ�С��IMU_STATIC_STD,����ѭ��n��
}

/*--------------- ��ʼ��״̬���� ---------------*/
static void navUkfInitState(void *Parameters) {
	float acc[3];
	float estAcc[3];
#ifdef USE_MAG
	float mag[3];
	float estMag[3];
#endif
	float m[3*3];
	//��һ�θ�����ȴ���ֵ
	if(!navUkfData.initLoops){
		// acc bias
		UKF_ACC_BIAS_X = 0.0f;
		UKF_ACC_BIAS_Y = 0.0f;
		UKF_ACC_BIAS_Z = 0.0f;

		// gyo bias
		UKF_GYO_BIAS_X = 0.0f;
		UKF_GYO_BIAS_Y = 0.0f;
		UKF_GYO_BIAS_Z = 0.0f;

		// quat
		UKF_Q1 =  1.0f;
		UKF_Q2 =  0.0f;
		UKF_Q3 =  0.0f;
		UKF_Q4 =  0.0f;

		// wait for lack of movement		�ȴ����ٶȼƳ�����
		imuQuasiStatic(UKF_GYO_AVG_NUM);
		// estimate initial orientation		���Ƴ�ʼ����
	}
	while(navUkfData.initLoops < 250){
		//�ȴ�������׼�����
		xEventGroupWaitBits(taskInitData.eventGroups,UKF_INIT_FLAG,pdTRUE,pdTRUE,portMAX_DELAY);
		float rotError[3];
		
		vTaskDelay(4*(configTICK_RATE_HZ / 1000));
#ifdef USE_MAG
		mag[0] = IMU_MAGX;
		mag[1] = IMU_MAGY;
		mag[2] = IMU_MAGZ;
#endif
		acc[0] = IMU_ACCX;
		acc[1] = IMU_ACCY;
		acc[2] = IMU_ACCZ;

		navUkfNormalizeVec3(acc, acc);								//���ٶȹ�һ��
#ifdef USE_MAG
		navUkfNormalizeVec3(mag, mag);								//�����ƹ�һ��
#endif		
		navUkfQuatToMatrix(m, &UKF_Q1, 1);
		// rotate gravity to body frame of reference	//��ת�������������ϵ
		navUkfRotateVecByRevMatrix(estAcc, navUkfData.v0a, m);
#ifdef USE_MAG
		// rotate mags to body frame of reference			//��ת�������������ϵ
		navUkfRotateVecByRevMatrix(estMag, navUkfData.v0m, m);	
#endif
		// measured error, starting with ACC vector		//�������Ӽ��ٶ�ʸ����ʼ
		rotError[0] = -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 1.0f;
		rotError[1] = -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 1.0f;
		rotError[2] = -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 1.0f;
#ifdef USE_MAG
		// add in MAG vector													//��������Ӵ���ʸ��
		rotError[0] += -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 1.0f;
		rotError[1] += -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 1.0f;
		rotError[2] += -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 1.0f;
#endif
		navUkfRotateQuat(&UKF_Q1, &UKF_Q1, rotError);	//����UKF��Ԫ��
		navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);				//UKF����Ԫ����һ��
		digitalIncreasing(&navUkfData.initLoops);
	}
	//��ʼ������
	navUkfData.initState = true;
	digitalClan(&navUkfData.initLoops);
	//ɾ����ǰ����
	vTaskDelete(navUkfData.xHandleTask);
}

void navUkfTaskInit(void){
	navUkfData.initState = false;
	digitalClan(&navUkfData.initLoops);
	getsupervisorData()->taskEvent[UKF_INIT_TASK] = xTaskCreate(navUkfInitState,"UKF_INIT",UKF_INIT_STACK_SIZE,NULL,UKF_INIT_PRIORITY,&navUkfData.xHandleTask);
}

/*------------- UKF��ʼ�� ------------*/
void navUkfInit(void) {
	float Q[SIM_S];		// state variance		״̬����
	float V[SIM_V];		// process variance		������������
#ifdef USE_MAG
	float mag[3];
#endif
  memset((void *)&navUkfData, 0, sizeof(navUkfData));

  navUkfData.v0a[0] = 0.0f;
  navUkfData.v0a[1] = 0.0f;
  navUkfData.v0a[2] = -1.0f;
#ifdef USE_MAG
	// calculate mag vector based on inclination			������ǵ�MAGʸ������
	mag[0] = cosf(-65.0 * DEG_TO_RAD);
	mag[1] = 0.0f;
	mag[2] = -sinf(-65.0 * DEG_TO_RAD);
	// rotate local mag vector to align with true north		��ת����MAGʸ�����汱����
	navUkfData.v0m[0] = mag[0] * cosf(-65.0 * DEG_TO_RAD) - mag[1] * sinf(-65.0  * DEG_TO_RAD);
	navUkfData.v0m[1] = mag[1] * cosf(-65.0 * DEG_TO_RAD) + mag[0] * sinf(-65.0  * DEG_TO_RAD);
	navUkfData.v0m[2] = mag[2];
#endif
	//navUkfData.kf����17��״̬��������������Ϊ3������������Ϊ12�����۲�������Ϊ3
	navUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navUkfTimeUpdate);					//Srcdkf�е�������ΪnavUkfData.kf

	navUkfData.x = srcdkfGetState(navUkfData.kf);
	//������״̬���������ֵ
	Q[0] = UKF_ACC_BIAS_Q;
	Q[1] = UKF_ACC_BIAS_Q;
	Q[2] = UKF_ACC_BIAS_Q;
	Q[3] = UKF_GYO_BIAS_Q;
	Q[4] = UKF_GYO_BIAS_Q;
	Q[5] = UKF_GYO_BIAS_Q;
	Q[6] = UKF_QUAT_Q;
	Q[7] = UKF_QUAT_Q;
	Q[8] = UKF_QUAT_Q;
	Q[9] = UKF_QUAT_Q;
	//������������������ֵ
	V[UKF_V_NOISE_ACC_BIAS_X] = UKF_ACC_BIAS_V;
	V[UKF_V_NOISE_ACC_BIAS_Y] = UKF_ACC_BIAS_V;
	V[UKF_V_NOISE_ACC_BIAS_Z] = UKF_ACC_BIAS_V;
	V[UKF_V_NOISE_GYO_BIAS_X] = UKF_GYO_BIAS_V;
	V[UKF_V_NOISE_GYO_BIAS_Y] = UKF_GYO_BIAS_V;
	V[UKF_V_NOISE_GYO_BIAS_Z] = UKF_GYO_BIAS_V;
	V[UKF_V_NOISE_RATE_X] = UKF_RATE_V;
	V[UKF_V_NOISE_RATE_Y] = UKF_RATE_V;
	V[UKF_V_NOISE_RATE_Z] = UKF_RATE_V;
	//����û�и��۲���������ֵ
	srcdkfSetVariance(navUkfData.kf, Q, V, 0, 0);												//���ù��������͹۲�������Э����
}
