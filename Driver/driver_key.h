#ifndef __KEY_H
#define __KEY_H	 
#include "bsp.h"
#include "FreeRTOS_board.h"
/**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define KEY1 PAin(8)
#define KEY2 PCin(9)
#define KEY3 PDin(12)








void KEY_Init(void);          //������ʼ��
u8 click_N_Double (u8 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);           //����ɨ��  
#endif  
