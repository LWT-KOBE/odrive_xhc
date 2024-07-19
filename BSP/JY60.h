#ifndef __JY60_H
#define __JY60_H

//�ṹ������
typedef struct 
{
	short a[3];
	short T;
}SAcc;						//���ٶȼ����ݽṹ��
typedef struct 
{
	short w[3];
	short T;
}SGyro;						//���������ݽṹ��
typedef struct 
{
	short Angle[3];
	short T;
}SAngle;					//��̬ת��֮������ݽṹ��
 
extern char YAWCMD[3];			//Z��Ƕȹ���
extern char ACCCMD[3];			//���ٶȼ�У׼
extern char SLEEPCMD[3];		//�����������
extern char UARTMODECMD[3];		//����ģʽ
extern char IICMODECMD[3];		//IICģʽ

//��������
void sendcmd(char cmd[]);						//ģ��ָ��ͺ���
void CopeSerial2Data(unsigned char ucData);		//���ڶ�ȡ����
void CopeSerial1Data(unsigned char ucData);		//ת������1�յ������ݸ�����2��JYģ�飩
void JY60_send(void);							//����JY60������
void JY60_Calibration(void);					//JY60У׼����
void JY60_Get(float * pbuf);					//��ȡJY60����
float JY_GET_FIRST(float* bbb);
#endif

