#ifndef __DRIVER_H
#define __DRIVER_H

#include "driver_clockCount.h"
#include "driver_usbvcp.h"
#include "driver_flash.h"
#include "driver_rgb.h"
#include "driver_clockCount.h"
#include "driver_crc.h"
#include "driver_key.h"



#include "driver_lcd.h"




/*��ʼ����ṹ��*/
typedef struct{
	void (*Init) 	(void);
    
}deviceInitClass;
/*ʱ����*/
extern deviceInitClass clockClass;
/*ADC��*/
extern deviceInitClass adcClass;
/*������*/
extern deviceInitClass sightClass;
/*������*/
extern deviceInitClass DTUClass;
/*��������*/
extern deviceInitClass IMUIintClass;
/*ң������*/
extern deviceInitClass DT7IintClass;
/*�����*/
extern deviceInitClass motorSeverInitClass;
/*�����*/
extern deviceInitClass LobotServoInitClass;



#endif
