#include "JY60.h"
#include <string.h>
#include <stdio.h>
//#include "delay.h"
#include "usartx.h"

char YAWCMD[3] = {0XFF,0XAA,0X52};				//Z��Ƕȹ����ָ��
char ACCCMD[3] = {0XFF,0XAA,0X67};              //���ٶȼ�У׼��ָ��
char SLEEPCMD[3] = {0XFF,0XAA,0X60};            //����������ߵ�ָ��
char UARTMODECMD[3] = {0XFF,0XAA,0X61};         //����ģʽ��ָ��
char IICMODECMD[3] = {0XFF,0XAA,0X62};          //IICģʽ��ָ��

SAcc     stcAcc;	//������ٶȼƽṹ�����
SGyro 	stcGyro;	//���������ǽṹ�����
SAngle	stcAngle;	//����ǶȽṹ�����

int32_t CNT = 0;
float angle_now = 0,angle_last = 0,angle_err = 0;

//�ô���2��JYģ�鷢��ָ��
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<3;i++)
		UART2_Put_Char(cmd[i]);
}

//CopeSerialDataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
					//���յ����ݣ�LED����˸һ��
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;//��ջ�����
	}
}

void CopeSerial1Data(unsigned char ucData)
{	
	UART2_Put_Char(ucData);//ת������1�յ������ݸ�����2��JYģ�飩
}

//����JY60������
void JY60_send(void){
		printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
		//delay_ms(10);
		//������ٶ�
		printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
		//delay_ms(10);
		//����Ƕ�
		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
		//delay_ms(10);//�ȴ��������
}

//У׼JY60����������
void JY60_Calibration(void){
	//У׼���ٶȼ�����
	sendcmd(ACCCMD);//�ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
	//У׼Z��Ƕ�Ϊ0
	sendcmd(YAWCMD);//�ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
	//printf("У׼�ɹ�!\r\n");
}

//��ȡJY60����������
void JY60_Get(float * pbuf){
	
	//��ȡ���ٶȼ�����
	pbuf[0] = (float)stcAcc.a[0]/32768*16;			//Acc_x
	pbuf[1] = (float)stcAcc.a[1]/32768*16;			//Acc_y
	pbuf[2] = (float)stcAcc.a[2]/32768*16;			//Acc_z
	
	//��ȡ����������	
	pbuf[3] = (float)stcGyro.w[0]/32768*2000;		//Gyro_x
	pbuf[4] = (float)stcGyro.w[1]/32768*2000;		//Gyro_y
	pbuf[5] = (float)stcGyro.w[2]/32768*2000;		//Gyro_z
	
	//��̬����֮�������
	pbuf[6] = (float)stcAngle.Angle[0]/32768*180;	//x��
	pbuf[7] = (float)stcAngle.Angle[1]/32768*180;	//y��
	
	if((float)stcAngle.Angle[2]>=0.0f && stcAngle.Angle[2] < 32767){
		pbuf[8] = (float)stcAngle.Angle[2]/32768*180.0f;	//z��
	}
	
	if((float)stcAngle.Angle[2]< 0.0f && stcAngle.Angle[2] > -32768){
		pbuf[8] = (float)stcAngle.Angle[2]/32768*180.0f + 360.0f;	//z��
	}
	
	//��ȡ��ǰ�Ƕ�
	angle_now = pbuf[8];
	//����Ƕ����
	angle_err = angle_now - angle_last;
	
	//�������㷨
	//���������180.0��
	if(angle_err >  180.0f){
		CNT--;
	}
	if(angle_err < -180.0){
		CNT++;
	}
	
	//�����ۻ��Ƕ�
	pbuf[9] = CNT * 360 + pbuf[8];
	//��ȡ��һ�εĽǶ�
	angle_last = angle_now;
	
}


