#ifndef __CAN2_TASK_H
#define __CAN2_TASK_H

#include "stm32f4xx.h"
#include "FreeRTOS_board.h"
#include "stdbool.h"
#include "bsp.h"
#include "util.h"

#define CAN2_Task_PRIORITY 11
#define CAN2_Task_STACK_SIZE 512
#define CAN2_Task_NORMAL_PERIOD 5




typedef struct {
	TaskHandle_t xHandleTask;
	uint8_t dataInitFlag;
	uint32_t loops;
} CAN2_TaskStruct_t;	

CAN2_TaskStruct_t* getCAN2Data(void);



void CAN2DataInit(void);

#endif
