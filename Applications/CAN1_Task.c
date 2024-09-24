#include "application.h"
#include "usartx.h"
#include "board.h"
CAN1_TaskStruct_t CAN1Data;
Servo_MotorStruct_t SM;
Servo_MotorDataRecv_t SM_Recv;
CANSendStruct_t can1data;

/*
***************************************************
函数名：CAN1SendData
功能：电机命令CAN发送_数据帧
入口参数：	CANx：CAN1 orCAN2
			ID_CAN：CANID 电机0或电机1的ID地址 规定： AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN：odrive的各种命令  
			len：数据帧长度 
			CanSendData：CAN发送的数据结构
返回值：无
应用范围：内部调用
备注： 该函数发送的是数据帧，切记
***************************************************
*/

void CANSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData) {
	CanTxMsg *txMessage;
	uint8_t mbox;
	uint8_t count;
	uint16_t i = 0;
	txMessage = (CanTxMsg*)aqCalloc(8,sizeof(CanTxMsg));
	
	//CAN ID 	
	txMessage->StdId = ID_CAN;	
	txMessage->IDE = CAN_Id_Standard;
	txMessage->RTR = CAN_RTR_Data;
	txMessage->DLC = len;
	for (count = 0; count < len; count++) {
		txMessage->Data[count] = (uint8_t)CanSendData->data[count];
	}
	mbox = CAN_Transmit(CANx, txMessage);
	while (CAN_TransmitStatus(CANx,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
	aqFree(txMessage,8,sizeof(CanTxMsg));
}

/*
***************************************************
函数名：CANSendInputVelData
功能：电机速度闭环命令_数据帧
入口参数：	CANx：CAN1 orCAN2
			ID_CAN：CANID 电机0或电机1的ID地址 规定： AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN：odrive的速度闭环命令（MSG_SET_INPUT_VEL ）
			len：数据帧长度 
			CanSendData：CAN发送的数据结构
			Spetsnaz：发送的速度控制结构体
			axis: 选择电机0或电机1命令发送 规定 ： axis0=0  axis1=1 
返回值：无
应用范围：内部调用
备注：
	 
***************************************************
*/
void CANSendInputVelData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,Servo_MotorStruct_t* Spetsnaz,uint8_t axis,CANSendStruct_t* CanSendData) {

	CanSendData->data[0] = 0x23;
	CanSendData->data[1] = 0x81;
	CanSendData->data[2] = 0x60;
	CanSendData->data[3] = 0x00;
	
	CanSendData->data[4] = Spetsnaz->SetVel[axis].u8_temp[0];
	CanSendData->data[5] = Spetsnaz->SetVel[axis].u8_temp[1];
	CanSendData->data[6] = Spetsnaz->SetVel[axis].u8_temp[2];
	CanSendData->data[7] = Spetsnaz->SetVel[axis].u8_temp[3];
	
	CANSendData(CANx,ID_CAN,len,CanSendData); 
	vTaskDelay(5);
}

/*
***************************************************
函数名：CANSendAcceData
功能：电机加速度修改命令
入口参数：	CANx：CAN1 orCAN2
			ID_CAN：CANID 电机0或电机1的ID地址 规定： AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN：odrive的速度闭环命令（MSG_SET_INPUT_VEL ）
			len：数据帧长度 
			CanSendData：CAN发送的数据结构
			Spetsnaz：发送的速度控制结构体
			axis: 选择电机0或电机1命令发送 规定 ： axis0=0  axis1=1 
返回值：无
应用范围：内部调用
备注：
	 
***************************************************
*/
void CANSendAcceData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,Servo_MotorStruct_t* Spetsnaz,uint8_t axis,CANSendStruct_t* CanSendData) {

	CanSendData->data[0] = 0x2b;
	CanSendData->data[1] = 0x83;
	CanSendData->data[2] = 0x60;
	CanSendData->data[3] = 0x00;
	
	CanSendData->data[4] = Spetsnaz->Acce[axis].u8_temp[0];
	CanSendData->data[5] = Spetsnaz->Acce[axis].u8_temp[1];
	CanSendData->data[6] = 0x00;
	CanSendData->data[7] = 0x00;
	
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
	
}


/*
***************************************************
函数名：CANSend_Torque
功能：电机加速度修改命令
入口参数：	CANx：CAN1 orCAN2
			ID_CAN：CANID 电机0或电机1的ID地址 规定： AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN：odrive的速度闭环命令（MSG_SET_INPUT_VEL ）
			len：数据帧长度 
			CanSendData：CAN发送的数据结构
			Spetsnaz：发送的速度控制结构体
			axis: 选择电机0或电机1命令发送 规定 ： axis0=0  axis1=1 
返回值：无
应用范围：内部调用
备注：
	 
***************************************************
*/
void CANSend_Torque(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,Servo_MotorStruct_t* Spetsnaz,uint8_t axis,CANSendStruct_t* CanSendData) {

	CanSendData->data[0] = 0x2b;
	CanSendData->data[1] = 0x71;
	CanSendData->data[2] = 0x60;
	CanSendData->data[3] = 0x00;
	
	CanSendData->data[4] = Spetsnaz->Acce[axis].u8_temp[0];
	CanSendData->data[5] = Spetsnaz->Acce[axis].u8_temp[1];
	CanSendData->data[6] = 0x00;
	CanSendData->data[7] = 0x00;
	
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
	
}



/*
***************************************************
函数名：CANSend_Slope
功能：电机加速度修改命令
入口参数：	CANx：CAN1 orCAN2
			ID_CAN：CANID 电机0或电机1的ID地址 规定： AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN：odrive的速度闭环命令（MSG_SET_INPUT_VEL ）
			len：数据帧长度 
			CanSendData：CAN发送的数据结构
			Spetsnaz：发送的速度控制结构体
			axis: 选择电机0或电机1命令发送 规定 ： axis0=0  axis1=1 
返回值：无
应用范围：内部调用
备注：
	 
***************************************************
*/
void CANSend_Slope(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,Servo_MotorStruct_t* Spetsnaz,uint8_t axis,CANSendStruct_t* CanSendData) {

	CanSendData->data[0] = 0x2b;
	CanSendData->data[1] = 0x87;
	CanSendData->data[2] = 0x60;
	CanSendData->data[3] = 0x00;
	
	CanSendData->data[4] = Spetsnaz->Acce[axis].u8_temp[0];
	CanSendData->data[5] = Spetsnaz->Acce[axis].u8_temp[1];
	CanSendData->data[6] = 0x00;
	CanSendData->data[7] = 0x00;
	
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
	
}



/*
***************************************************
函数名：CANSendAccData
功能：电机加速度修改命令
入口参数：	CANx：CAN1 orCAN2
			ID_CAN：CANID 电机0或电机1的ID地址 规定： AXIS0_ID 0x001   AXIS1_ID 0x002
			CMD_CAN：odrive的速度闭环命令（MSG_SET_INPUT_VEL ）
			len：数据帧长度 
			CanSendData：CAN发送的数据结构
			Spetsnaz：发送的速度控制结构体
			axis: 选择电机0或电机1命令发送 规定 ： axis0=0  axis1=1 
返回值：无
应用范围：内部调用
备注：
	 
***************************************************
*/
void CANSendDeceData(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,Servo_MotorStruct_t* Spetsnaz,uint8_t axis,CANSendStruct_t* CanSendData) {
	
	CanSendData->data[0] = 0x2b;
	CanSendData->data[1] = 0x84;
	CanSendData->data[2] = 0x60;
	CanSendData->data[3] = 0x00;
	
	CanSendData->data[4] = Spetsnaz->Dece[axis].u8_temp[0];
	CanSendData->data[5] = Spetsnaz->Dece[axis].u8_temp[1];
	CanSendData->data[6] = 0x00;
	CanSendData->data[7] = 0x00;
	
	CANSendData(CANx,ID_CAN,len,CanSendData); 
	
}


void Set_Pos_mode(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
	u8 p_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
	
	//设置位置模式
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = p_mode_set[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
}

void Set_Vel_mode(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
	u8 v_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
	
	//设置速度模式
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = v_mode_set[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
}



void Set_Cur_mode(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
	u8 c_mode_set[8]= {0x2f, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};
	
	//设置力矩模式
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = c_mode_set[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
}


//使能电机
void Enable(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
	u8 enable1[8]= {0x2b, 0x40, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
	u8 enable2[8]= {0x2b, 0x40, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
	u8 enable3[8]= {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
	
	//初始化设备
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = enable1[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
	
	//打开刹车
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = enable2[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
	
	//使能电机
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = enable3[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
	
}

//停止
void Stop(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
	u8 Stop[8]= {0x2b, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = Stop[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
}

//修改转矩斜率
void Cur_Target_Torque(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
	u8 Stop[8]= {0x2b, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = Stop[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(5);
}

//CANOpen使能
void CIA301_Enable(CAN_TypeDef *CANx,CANSendStruct_t* CanSendData){
	
	u8 cia301_enable[2] = {0x01,0x00};
	//**** CIA301 状态机 预操作到操作状态****
	for(int i = 0;i < 2;i++){
		CanSendData->data[i] = cia301_enable[i];
	}
	CANSendData(CANx,000,2,CanSendData);
	vTaskDelay(50);
}

//设置PDO_通讯参数0
void Set_PDO_Vel_Pos(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
//	u8 CIA301_Enable[2] = {0x01,0x00};
	
	u8 PDO1[8]= {0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0x00};	//配置TPD0通讯参数的COB_ID：0x181
	u8 PDO2[8]= {0x2f, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};   //清零从索引个数
	u8 PDO3[8]= {0x23, 0x00, 0x1a, 0x01, 0x20, 0x00, 0x6c, 0x60};   //映射对象1:实际速度
	u8 PDO4[8]= {0x23, 0x00, 0x1a, 0x02, 0x20, 0x00, 0x64, 0x60};   //映射对象2:实际位置
	u8 PDO5[8]= {0x2f, 0x00, 0x1a, 0x00, 0x02, 0x00, 0x00, 0x00};   //从索引个数2
	u8 PDO6[8]= {0x2f, 0x00, 0x18, 0x02, 0xfe, 0x00, 0x00, 0x00};   //传输类型:0XFE(254)异步传输,根据事件设定时间返回PDO数据
	u8 PDO7[8]= {0x2b, 0x00, 0x18, 0x05, 0x0a, 0x00, 0x00, 0x00};   //事件时间10ms
	
	//根据节点ID设置PDO回传ID
	switch(ID_CAN){
		case 1537:PDO1[4] = 0x81;break;
		case 1538:PDO1[4] = 0x82;break;
		case 1539:PDO1[4] = 0x83;break;
		
		default:	break;
	}
	
	//**** CIA301 状态机 预操作到操作状态****
//	for(int i = 0;i < 2;i++){
//		CanSendData->data[i] = CIA301_Enable[i];
//	}
//	CANSendData(CANx,000,2,CanSendData);
//	vTaskDelay(50);
	
	// 配置PDO通讯参数的COB_ID
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO1[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 清零从索引个数
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO2[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 映射对象1：实际速度：0x606C
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO3[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 映射对象2：实际位置：0x6064
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO4[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 从索引个数2
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO5[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 传输类型：0xFE（254）异步传输，根据事件设定时间返回PDO数据
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO6[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 设置10ms回传
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO7[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	
	vTaskDelay(50);
	
}


//设置PDO_通讯参数1
void Set_PDO_Cur(CAN_TypeDef *CANx, uint32_t ID_CAN, uint8_t len,CANSendStruct_t* CanSendData){
	
//	u8 CIA301_Enable[2] = {0x01,0x00};
	
	u8 PDO1[8]= {0x23, 0x01, 0x18, 0x02, 0x81, 0x01, 0x00, 0x00};   //配置TPD1通讯参数的COB_ID：0x281
	u8 PDO2[8]= {0x2f, 0x01, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};	//清零从索引个数
	u8 PDO3[8]= {0x23, 0x01, 0x1a, 0x01, 0x10, 0x00, 0x77, 0x60};	//映射对象1:实际力矩
	u8 PDO4[8]= {0x23, 0x01, 0x1a, 0x02, 0x10, 0x00, 0x41, 0x60};	//映射对象2:状态字
	u8 PDO5[8]= {0x2f, 0x01, 0x1a, 0x00, 0x02, 0x00, 0x00, 0x00};	//从索引个数2
	u8 PDO6[8]= {0x2f, 0x01, 0x18, 0x02, 0xfe, 0x00, 0x00, 0x00};	//传输类型:0XFE(254)异步传输,根据事件设定时间返回PDO数据
	u8 PDO7[8]= {0x2b, 0x01, 0x18, 0x05, 0x0a, 0x00, 0x00, 0x00};	//事件时间10ms
	
	//根据节点ID设置PDO回传ID
	switch(ID_CAN){
		case 1537:PDO1[4] = 0x81;break;
		case 1538:PDO1[4] = 0x82;break;
		case 1539:PDO1[4] = 0x83;break;
		
		default:	break;
	}
	
//	//**** CIA301 状态机 预操作到操作状态****
//	for(int i = 0;i < 2;i++){
//		CanSendData->data[i] = CIA301_Enable[i];
//	}
//	CANSendData(CANx,000,2,CanSendData);
//	vTaskDelay(50);
	
	// 配置PDO通讯参数的COB_ID
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO1[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 清零从索引个数
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO2[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 映射对象1：实际速度：0x606C
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO3[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 映射对象2：实际位置：0x6064
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO4[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 从索引个数2
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO5[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 传输类型：0xFE（254）异步传输，根据事件设定时间返回PDO数据
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO6[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	vTaskDelay(50);
	
	// 设置10ms回传
	for(int i = 0;i < 8;i++){
		CanSendData->data[i] = PDO7[i];
	}
	CANSendData(CANx,ID_CAN,len,CanSendData);
	
	vTaskDelay(50);
	
}


/*
***************************************************
函数名：ReadVel_Pos
功能：读取实时速度、位置
入口参数：	
			CanSendData：CAN发送的数据结构
			Spetsnaz：接收CANOPen伺服电机的速度、位置数据结构体
			axis: 选择电机0或电机1命令发送 规定 ： axis0=0  axis1=1 
返回值：无
应用范围：内部调用
***************************************************
*/
void ReadVel_Pos(CanRxMsg* CanRevData,Servo_MotorDataRecv_t* Spetsnaz,uint8_t axis) {
		
	
	Spetsnaz->vel_estimate[axis].u8_temp[0] = CanRevData->Data[0];	
	Spetsnaz->vel_estimate[axis].u8_temp[1] = CanRevData->Data[1];	
	Spetsnaz->vel_estimate[axis].u8_temp[2] = CanRevData->Data[2];	
	Spetsnaz->vel_estimate[axis].u8_temp[3] = CanRevData->Data[3];
	
	Spetsnaz->pos_estimate[axis].u8_temp[0] = CanRevData->Data[4];
	Spetsnaz->pos_estimate[axis].u8_temp[1] = CanRevData->Data[5];
	Spetsnaz->pos_estimate[axis].u8_temp[2] = CanRevData->Data[6];
	Spetsnaz->pos_estimate[axis].u8_temp[3] = CanRevData->Data[7];
}


CAN1_TaskStruct_t* getCAN1_Task(){
    return &CAN1Data;
}

/* 放置初始化 */
void CAN2_TaskGlobalInit(void){
	//初始化CAN2 波特率125K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,24,CAN_Mode_Normal);
	//初始化CAN2 波特率250K
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,12,CAN_Mode_Normal);
	//初始化CAN2 波特率1M
	//CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
}

void CAN1_TaskUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getCAN1_Task()->dataInitFlag);
	int i = 0;
	while(true){
		vTaskDelayUntil(&xLastWakeTime,CAN1_Task_NORMAL_PERIOD);
		SM.Acce[0].s16_temp = 1000;
		SM.Dece[0].s16_temp = 1000;
        //防止重复初始化
		if(!CAN1Data.dataInitFlag){
			i++;
            //所有控制全部初始化            
			CAN2_TaskGlobalInit();
			if(i < 3){
				CIA301_Enable(CAN1,&can1data);
				CIA301_Enable(CAN1,&can1data);
				CIA301_Enable(CAN1,&can1data);
				//初始化POD配置，默认回传实时速度与位置
				Set_PDO_Vel_Pos(CAN1,Servo_Motor_ID0,8,&can1data);
				Set_PDO_Vel_Pos(CAN1,Servo_Motor_ID1,8,&can1data);
				Set_PDO_Vel_Pos(CAN1,Servo_Motor_ID2,8,&can1data);
			}
			if(i > 3 && i < 6){
				//使能电机
				Enable(CAN1,Servo_Motor_ID0,8,&can1data);
				Enable(CAN1,Servo_Motor_ID1,8,&can1data);
				Enable(CAN1,Servo_Motor_ID2,8,&can1data);
			}
			if(i > 6){
				//设置速度模式
				Set_Vel_mode(CAN1,Servo_Motor_ID0,8,&can1data);
				Set_Vel_mode(CAN1,Servo_Motor_ID1,8,&can1data);
				Set_Vel_mode(CAN1,Servo_Motor_ID2,8,&can1data);
				//设置加速度
				CANSendDeceData(CAN1,Servo_Motor_ID0,8,&SM,0,&can1data);
				CANSendDeceData(CAN1,Servo_Motor_ID1,8,&SM,0,&can1data);
				CANSendDeceData(CAN1,Servo_Motor_ID2,8,&SM,0,&can1data);
				//设置减速度
				CANSendAcceData(CAN1,Servo_Motor_ID0,8,&SM,0,&can1data);
				CANSendAcceData(CAN1,Servo_Motor_ID1,8,&SM,0,&can1data);
				CANSendAcceData(CAN1,Servo_Motor_ID2,8,&SM,0,&can1data);
				
			}
			if(i == 9)
			digitalHi(&getCAN1_Task()->dataInitFlag);
			
		}

		CANSendInputVelData(CAN1,Servo_Motor_ID0,8,&SM,0,&can1data);
		CANSendInputVelData(CAN1,Servo_Motor_ID1,8,&SM,1,&can1data);
		CANSendInputVelData(CAN1,Servo_Motor_ID2,8,&SM,2,&can1data);
		
		digitalIncreasing(&getCAN1_Task()->loops);        

	}
}



//     Frame
// nodeID
CanRxMsg can1_rx_msg;
u32 rxbuf3;
void CAN1_RX0_IRQHandler(void){
	//CanRxMsg can1_rx_msg;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		// 清除中断标志和标志位
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
		
		// 从接收 FIFO 中读取消息		
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		
		// 存储接收到的标准 ID
		rxbuf3=can1_rx_msg.StdId;
		// 递增错误计数
		
		digitalIncreasing(&OdriveData.OdError.errorCount);

		/*********以下是自定义部分**********/
		
		switch(can1_rx_msg.StdId){
			
			case 385:
				ReadVel_Pos(&can1_rx_msg,&SM_Recv,0);
			break;
			case 386:
				ReadVel_Pos(&can1_rx_msg,&SM_Recv,1);
			break;
			case 387:
				ReadVel_Pos(&can1_rx_msg,&SM_Recv,2);
			break;							
				
	
				
			default:	break;
		}

	}
}


/*
***************************************************
函数名：CAN1_TX_IRQHandler
功能：CAN1发送中断
备注：
***************************************************
*/
void CAN1_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

		/*********以下是自定义部分**********/
        OdriveSend_RemoteCmd(CAN1,AXIS0_ID,MSG_GET_MOTOR_ERROR);
        
	}
}




void CAN2DataInit(void){
	getsupervisorData()->taskEvent[CAN1_Task] = xTaskCreate(CAN1_TaskUpdateTask,"CAN1_Task",CAN1_Task_STACK_SIZE,NULL,CAN1_Task_PRIORITY,&CAN1Data.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
