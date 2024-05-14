//*******************************
//STM32F407 ����LED����
//File: led.h
//Author: xxpcb(wxgzh:��ũ��ѧϰ)
//Vesion: V1.0
//Date: 2020/05/30
//*******************************

#ifndef __CIGAN_H
#define __CIGAN_H
#include "bsp_gpio.h"

#define digitalHigh(p,i)     {p->BSRRH=i;}  //��������ߵ�ƽ       
#define digitalLow(p,i)      {p->BSRRL=i;}   //��������͵�ƽ
#define digitalToggle(p,i)   {p->ODR ^=i;} //��ת

#define cigan_on			PAin(4) = 1;
#define cigan_off			PAin(4) = 0;
void cigan_Init(void);//��ʼ��		 
void EXTI_Configuration(void);
#endif
