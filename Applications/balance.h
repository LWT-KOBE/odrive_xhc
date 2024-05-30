#ifndef __BALANCE_H
#define __BALANCE_H

#include "stm32f4xx.h"
#include "FreeRTOS_board.h"
#include "stdbool.h"
#include "bsp.h"
#include "util.h"

#define USE_MAG

#define BALANCE_PRIORITY 9
#define BALANCE_STACK_SIZE 512
#define BALANCE_NORMAL_PERIOD 5



typedef struct {
	TaskHandle_t xHandleTask;
	uint8_t dataInitFlag;
	uint32_t loops;
} balanceStruct_t;

typedef struct{
	float target;
	uint8_t finish;
	uint8_t change;
}balance_target_t;

balanceStruct_t* getbalanceData(void);


extern balance_target_t Angle_Goal;
extern float pbuf[10];
extern balance_target_t Motor_SpeedA_Goal;
extern balance_target_t Motor_SpeedB_Goal;
void balanceInit(void);

#endif
