#ifndef __ODRIVE_H
#define __ODRIVE_H

#include "stm32f4xx.h"
#include "Util.h"

#define AXIS0_ID 0x001 //M0的CANID
#define AXIS1_ID 0x002 //M1的CANID

#define ODRIVE_PRIORITY 3
#define ODRIVE_STACK_SIZE 512
#define ODRIVE_NORMAL_PERIOD 10



//电机的ID 
enum{	
	axis0=0,	
	axis1=1,		
	
};

//电机错误值
enum MOTOR_Error {
            ERROR_NONE  = 0,
			ERR_OVER_VOLTAGE =  1,
			ERR_UNDER_VOLTAGE = 2,
			ERR_OVER_SPEED = 4,
			ERR_OVER_TEMP = 10,
};

//编码器错误值
 enum ENCODER_Error {
	CE_NULL = 0,
	CE_PHASE_RESISTANCE_OUT_OF_RANGE,
	CE_MOTOR_POLE_PAIRS_OUT_OF_RANGE,
};		




	
	//CAN的电机控制命令
typedef  enum {
        MSG_CO_NMT_CTRL = 0x000,             		 // CANOpen NMT Message REC
		MSG_ODRIVE_HEARTBEAT = 0x001,                // ODrive心跳消息
		MSG_ODRIVE_ESTOP = 0x002,                    // ODrive急停消息
		MSG_GET_MOTOR_ERROR = 0x003,                 // 获取电机错误信息
		MSG_GET_ENCODER_ERROR = 0x004,               // 获取编码器错误信息
		MSG_GET_SENSORLESS_ERROR = 0x005,            // 获取无传感器错误信息
		MSG_SET_AXIS_NODE_ID = 0x006,                // 设置轴节点ID
		MSG_SET_AXIS_REQUESTED_STATE = 0x007,        // 设置轴请求状态
		MSG_SET_AXIS_STARTUP_CONFIG = 0x008,         // 设置轴启动配置
		MSG_GET_ENCODER_ESTIMATES = 0x009,           // 获取编码器估计值
		MSG_GET_ENCODER_COUNT = 0x00A,               // 获取编码器计数
		MSG_SET_CONTROLLER_MODES = 0x00B,            // 设置控制器模式
		MSG_SET_INPUT_POS = 0x00C,                   // 设置位置控制输入
		MSG_SET_INPUT_VEL = 0x00D,                   // 设置速度控制输入
		MSG_SET_INPUT_TORQUE = 0x00E,                // 设置扭矩控制输入
		MSG_SET_LIMITS = 0x00F,                      // 设置限制
		MSG_START_ANTICOGGING = 0x010,               // 启动反扭矩
		MSG_SET_TRAJ_VEL_LIMIT = 0x011,              // 设置轨迹速度限制
		MSG_SET_TRAJ_ACCEL_LIMITS = 0x012,          // 设置轨迹加速度限制
		MSG_SET_TRAJ_INERTIA = 0x013,                // 设置轨迹惯性
		MSG_GET_IQ = 0x014,                          // 获取电流
		MSG_GET_SENSORLESS_ESTIMATES = 0x015,        // 获取无传感器估计值
		MSG_RESET_ODRIVE = 0x016,                    // 重置ODrive
		MSG_GET_VBUS_VOLTAGE = 0x017,                // 获取总线电压
		MSG_CLEAR_ERRORS = 0x018,                    // 清除错误

		//新增
		MSG_GET_TEMP = 0x019,                        // 获取温度
		MSG_SAVE_CONFIG = 0x01A,                     // 保存配置
		MSG_SET_MOTOR_ENABLE = 0x01B,                // 启用电机
		MSG_SET_MOTOR_DISABLE = 0x01C,               // 禁用电机
		MSG_SET_CONTROL_MODE = 0x01D,                // 设置控制模式
		MSG_SET_POS_GAIN = 0x01E,					 //	设置位置环增益
        MSG_SET_VEL_GAINS = 0x01F,					 // 设置速度环增益

		MSG_CO_HEARTBEAT_CMD = 0x700,        // CANOpen NMT Heartbeat SEND
    }ODCmdStruct_t;
	
	
	
//电机控制模式
typedef enum {
	CONTROL_MODE_CURRENT		= 0,//电流控制模式——直接控制
	CONTROL_MODE_CURRENT_RAMP	= 1,//电流控制模式——梯形
	CONTROL_MODE_VELOCITY		= 2,//速度控制模式——直接控制
	CONTROL_MODE_VELOCITY_RAMP	= 3,//速度控制模式——梯形
	CONTROL_MODE_POSITION		= 4,//位置控制模式——直接控制
	CONTROL_MODE_POSITION_TRAP	= 5,//位置控制模式——梯形

} ODControlMode;


//电机控制状态
typedef enum {

	// CMD
	CMD_MENU	=	0x1B,	// Esc 释放电机
	CMD_MOTOR   		=	'm',//电机进入闭环模式
	CMD_CALIBRATION		=	'c',//电机校准
		
	//以下控制模式请在串口上位机上配置	
	CMD_ANTICOGGING		=	'a',
	CMD_UPDATE_CONFIGS	=	'u',
	CMD_RESET_ERROR		=	'z',
	CMD_DEBUG_Q			=	'q',
	CMD_DEBUG_W			=	'w',
	CMD_UART_SETUP 		=	's',
	
}ODAxisStateStruct_t;


/*Odrive 的CAN发送结构体*/
typedef struct {
	uint8_t cmd;
	uint8_t data[8];
}ODCANSendStruct_t;


/*Odrive 的CAN接收结构体*/
typedef struct {   
		
	formatTrans32Struct_t vbus_voltage ;//Odrive总线电压
	formatTrans32Struct_t ibus ;//Odrive总线电流	
	formatTrans32Struct_t MotorError[2];//电机错误
	formatTrans32Struct_t EncoderError[2];//编码器错误		
	formatTrans32Struct_t temperature[2];//温度	
	
	//心跳信号反馈的速度/位置/电流数据，float型，精度0.1f
	float heartbeat_Pos[2];
	float heartbeat_Cur[2];
	float heartbeat_Vel[2];	
	
	//心跳信号反馈的轴错误数据
	uint8_t heartbeat_AxisError[2];
	
	//心跳信号反馈的电机控制状态数据	
	uint8_t heartbeat_Current_State[2];
	
	//电机控制模式
	ODControlMode control_mode[2];				
	
	float motor_phase_resistance[2];//电机电阻值
	float motor_phase_inductance[2];//电机电感值
	uint8_t motor_pole_pairs[2];//电机的极对数

	formatTrans32Struct_t shadow_count[2];//shadow	
	formatTrans32Struct_t count_in_cpr[2];//CPR
	formatTrans32Struct_t pos_estimate[2];//位置	
	formatTrans32Struct_t vel_estimate[2];//速度
	formatTrans32Struct_t Iq_measured[2];//电流	

	formatTrans32Struct_t vel_limit[2]; //速度限制——接收
	formatTrans32Struct_t current_limit[2];//电流限制——接收


}ODCanDataRecv_t;


typedef struct {

	//电机的控制状态
	ODAxisStateStruct_t	AxisState[2];
	
	//电机的控制模式	
	ODControlMode ControlMode[2];
	
	//设定电机的位置——float型
	formatTrans32Struct_t SetPos[2];
	
	//设定电机的速度——float型	
	formatTrans32Struct_t SetVel[2];
	
	//设定电机的电流——float型	
	formatTrans32Struct_t SetCur[2];	
		
	//配置电机状态FLAG	
	uint8_t RequestedStateFlag;

	//保存ODRIVE的配置参数FLAG
	uint8_t flashSaveFlag;
	
	//配置电机控制模式FLAG	
	uint8_t ControlModeFlag;	
	
	//清楚电机错误FLAG
	uint8_t clearerrorFlag;
	
	//查询电机的电感/电阻/极对数和电机控制状态
	uint8_t readpairsFlag;

	//设置电流电压限制FLAG	
	uint8_t SetLimitFlag;


	//速度限制——发送
	formatTrans32Struct_t vel_limit[2]; 
	
	//电流限制——发送
	formatTrans32Struct_t current_limit[2];

	//位置环增益
	formatTrans32Struct_t pos_gain[2];
	
	//速度环增益
	formatTrans32Struct_t vel_gain[2];

	//freertos相关
	TaskHandle_t xHandleTask;
	uint8_t dataInitFlag;
	uint32_t loops;
	
	errorScanStruct_t OdError;
	
	
	
	//CAN的电机控制CMD结构体
	ODCmdStruct_t OdriveCmd;
} OdriveStruct_t;	

void OdriveSendData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,ODCANSendStruct_t* CanSendData);
void ODSendInputPosData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);
void set_axis_requested_state(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);


void ODReadHeartBeatData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODReadVbusData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz);
void OdriveSend_RemoteCmd(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN);
void ODReadTempData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODReadMotorErrorData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODReadEncoderCountData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODReadEncodeEstimatesData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODReadEncodeErrorData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODSendInputVelData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);
void ODSendInputCurData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);
void ODReadMotorPolePairs(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODReadLimitData(CanRxMsg* CanRevData,ODCanDataRecv_t* Spetsnaz,uint8_t axis);
void ODSendLimitData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);
void ODSendPos_gainData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);

void OdriveInit(void);
void ODRequestedState(void);	
void ODFlashSave(void);	
void ODClearError(void);		
void ODFReadMotorPairs(void);	
void ODSetLimit(void);	

extern OdriveStruct_t OdriveData; 
extern ODCanDataRecv_t OdReceivedData;


#endif
