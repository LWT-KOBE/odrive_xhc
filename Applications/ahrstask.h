#ifndef __AHRS_H
#define __AHRS_H
#include "bsp.h"
#include "FreeRTOS_board.h"
#include "board.h"

#define AHRS_PRIORITY 9
#define AHRS_STACK_SIZE 512
#define AHRS_NORMAL_PERIOD 2

#define EXP_TEMPERATURE 60
#define TEMPERATURE_KP 25.0f
#define TEMPERATURE_KI 5.0f
#define TEMPERATURE_KD 3.5f
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )



#define DCM_KP_ACC			0.600f		//���ٶȲ���������PI����
#define DCM_KI_ACC			0.005f

#define DCM_KP_MAG			1.000f		//�����Ʋ���������PI����
#define DCM_KI_MAG			0.000f

#define SPIN_RATE_LIMIT     20			//��ת����
#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

#ifndef sq
#define sq(x) ((x)*(x))
#endif

#define RAD    (M_PIf / 180.0f)


#define DEGREES_TO_RADIANS(angle) ((angle) * RAD)
#define RADIANS_TO_DEGREES(angle) ((angle) / RAD)
#define M_PIf       3.14159265358979323846f

typedef struct {
	TaskHandle_t xHandleTask;
	//robotModeStruct_t robotMode;
	uint8_t dataInitFlag;
	uint32_t loops;
	
} ahrsStruct_t;	


extern ahrsStruct_t AhrsData;

void ahrsInit(void);




#endif




