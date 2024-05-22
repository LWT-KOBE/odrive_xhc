#ifndef __ODRIVE_H
#define __ODRIVE_H

#include "stm32f4xx.h"
#include "Util.h"

#define AXIS0_ID 0x000 //M0��CANID
#define AXIS1_ID 0x003 //M1��CANID

#define ODRIVE_PRIORITY 3
#define ODRIVE_STACK_SIZE 512
#define ODRIVE_NORMAL_PERIOD 10



//�����ID 
enum{	
	axis0=0,	
	axis1=1,		
	
};

//�������ֵ
enum MOTOR_Error {
            ERROR_NONE  = 0,
			ERR_OVER_VOLTAGE =  1,
			ERR_UNDER_VOLTAGE = 2,
			ERR_OVER_SPEED = 4,
			ERR_OVER_TEMP = 10,
};

//����������ֵ
 enum ENCODER_Error {
	CE_NULL = 0,
	CE_PHASE_RESISTANCE_OUT_OF_RANGE,
	CE_MOTOR_POLE_PAIRS_OUT_OF_RANGE,
};		




	
	//CAN�ĵ����������
typedef  enum {
        MSG_CO_NMT_CTRL = 0x000,             		 // CANOpen NMT Message REC
		MSG_ODRIVE_HEARTBEAT = 0x001,                // ODrive������Ϣ
		MSG_ODRIVE_ESTOP = 0x002,                    // ODrive��ͣ��Ϣ
		MSG_GET_MOTOR_ERROR = 0x003,                 // ��ȡ���������Ϣ
		MSG_GET_ENCODER_ERROR = 0x004,               // ��ȡ������������Ϣ
		MSG_GET_SENSORLESS_ERROR = 0x005,            // ��ȡ�޴�����������Ϣ
		MSG_SET_AXIS_NODE_ID = 0x006,                // ������ڵ�ID
		MSG_SET_AXIS_REQUESTED_STATE = 0x007,        // ����������״̬
		MSG_SET_AXIS_STARTUP_CONFIG = 0x008,         // ��������������
		MSG_GET_ENCODER_ESTIMATES = 0x009,           // ��ȡ����������ֵ
		MSG_GET_ENCODER_COUNT = 0x00A,               // ��ȡ����������
		MSG_SET_CONTROLLER_MODES = 0x00B,            // ���ÿ�����ģʽ
		MSG_SET_INPUT_POS = 0x00C,                   // ����λ�ÿ�������
		MSG_SET_INPUT_VEL = 0x00D,                   // �����ٶȿ�������
		MSG_SET_INPUT_TORQUE = 0x00E,                // ����Ť�ؿ�������
		MSG_SET_LIMITS = 0x00F,                      // ��������
		MSG_START_ANTICOGGING = 0x010,               // ������Ť��
		MSG_SET_TRAJ_VEL_LIMIT = 0x011,              // ���ù켣�ٶ�����
		MSG_SET_TRAJ_ACCEL_LIMITS = 0x012,           // ���ù켣���ٶ�����
		MSG_SET_TRAJ_INERTIA = 0x013,                // ���ù켣����
		MSG_GET_IQ = 0x014,                          // ��ȡ����
		MSG_GET_SENSORLESS_ESTIMATES = 0x015,        // ��ȡ�޴���������ֵ
		MSG_RESET_ODRIVE = 0x016,                    // ����ODrive
		MSG_GET_VBUS_VOLTAGE = 0x017,                // ��ȡ���ߵ�ѹ
		MSG_CLEAR_ERRORS = 0x018,                    // �������

		//����
		MSG_GET_TEMP = 0x019,                        // ��ȡ�¶�
		MSG_SAVE_CONFIG = 0x01A,                     // ��������
		MSG_SET_MOTOR_ENABLE = 0x01B,                // ���õ��
		MSG_SET_MOTOR_DISABLE = 0x01C,               // ���õ��
		MSG_SET_CONTROL_MODE = 0x01D,                // ���ÿ���ģʽ
		MSG_SET_POS_GAIN = 0x01E,					 //	����λ�û�����
        MSG_SET_VEL_GAINS = 0x01F,					 // �����ٶȻ�����	
		MSG_SET_VEL_INTEGRATOR_GAIN = 0x020,		 // �����ٶȻ���������
		
		MSG_GET_POS_GAIN = 0x23,
		MSG_GET_VEL_GAINS = 0x24,

		MSG_CO_HEARTBEAT_CMD = 0x700,        // CANOpen NMT Heartbeat SEND
    }ODCmdStruct_t;
	
	
	
//�������ģʽ
typedef enum {
	CONTROL_MODE_CURRENT		= 0,//��������ģʽ����ֱ�ӿ���
	CONTROL_MODE_CURRENT_RAMP	= 1,//��������ģʽ��������
	CONTROL_MODE_VELOCITY		= 2,//�ٶȿ���ģʽ����ֱ�ӿ���
	CONTROL_MODE_VELOCITY_RAMP	= 3,//�ٶȿ���ģʽ��������
	CONTROL_MODE_POSITION		= 4,//λ�ÿ���ģʽ����ֱ�ӿ���
	CONTROL_MODE_POSITION_TRAP	= 5,//λ�ÿ���ģʽ��������

} ODControlMode;


//�������״̬
typedef enum {

	// CMD
	CMD_MENU	=	0x1B,	// Esc �ͷŵ��
	CMD_MOTOR   		=	'm',//�������ջ�ģʽ
	CMD_CALIBRATION		=	'c',//���У׼
		
	//���¿���ģʽ���ڴ�����λ��������	
	CMD_ANTICOGGING		=	'a',
	CMD_UPDATE_CONFIGS	=	'u',
	CMD_RESET_ERROR		=	'z',
	CMD_DEBUG_Q			=	'q',
	CMD_DEBUG_W			=	'w',
	CMD_UART_SETUP 		=	's',
	
}ODAxisStateStruct_t;


/*Odrive ��CAN���ͽṹ��*/
typedef struct {
	uint8_t cmd;
	uint8_t data[8];
}ODCANSendStruct_t;


/*Odrive ��CAN���սṹ��*/
typedef struct {   
		
	formatTrans32Struct_t vbus_voltage ;//Odrive���ߵ�ѹ
	formatTrans32Struct_t ibus ;//Odrive���ߵ���	
	formatTrans32Struct_t MotorError[2];//�������
	formatTrans32Struct_t EncoderError[2];//����������		
	formatTrans32Struct_t temperature[2];//�¶�
	
	formatTrans32Struct_t Pos_gain[2];//λ�û�����		
	formatTrans32Struct_t Vel_gain[2];//�ٶȻ�����
	formatTrans32Struct_t Vel_integrator_gain[2];//�ٶȻ���������
	
	//�����źŷ������ٶ�/λ��/�������ݣ�float�ͣ�����0.1f
	float heartbeat_Pos[2];
	float heartbeat_Cur[2];
	float heartbeat_Vel[2];	
	
	//�����źŷ��������������
	uint8_t heartbeat_AxisError[2];
	
	//�����źŷ����ĵ������״̬����	
	uint8_t heartbeat_Current_State[2];
	
	//�������ģʽ
	ODControlMode control_mode[2];				
	
	float motor_phase_resistance[2];//�������ֵ
	float motor_phase_inductance[2];//������ֵ
	uint8_t motor_pole_pairs[2];//����ļ�����

	formatTrans32Struct_t shadow_count[2];//shadow	
	formatTrans32Struct_t count_in_cpr[2];//CPR
	formatTrans32Struct_t pos_estimate[2];//λ��	
	formatTrans32Struct_t vel_estimate[2];//�ٶ�
	formatTrans32Struct_t Iq_measured[2];//����	

	formatTrans32Struct_t vel_limit[2]; //�ٶ����ơ�������
	formatTrans32Struct_t current_limit[2];//�������ơ�������


}ODCanDataRecv_t;


typedef struct {
	
	//λ�û�����
	formatTrans32Struct_t Pos_gain[2];
	
	//�ٶȻ�����
	formatTrans32Struct_t Vel_gain[2];
	
	//�ٶȻ���������
	formatTrans32Struct_t Vel_integrator_gain[2];
	
	//����Ŀ���״̬
	ODAxisStateStruct_t	AxisState[2];
	
	//����Ŀ���ģʽ	
	ODControlMode ControlMode[2];
	
	//�趨�����λ�á���float��
	formatTrans32Struct_t SetPos[2];
	
	//�趨������ٶȡ���float��	
	formatTrans32Struct_t SetVel[2];
	
	//�趨����ĵ�������float��	
	formatTrans32Struct_t SetCur[2];	
		
	//���õ��״̬FLAG	
	uint8_t RequestedStateFlag;

	//����ODRIVE�����ò���FLAG
	uint8_t flashSaveFlag;
	
	//���õ������ģʽFLAG	
	uint8_t ControlModeFlag;	
	
	//����������FLAG
	uint8_t clearerrorFlag;
	
	//��ѯ����ĵ��/����/�������͵������״̬
	uint8_t readpairsFlag;

	//���õ�����ѹ����FLAG	
	uint8_t SetLimitFlag;


	//�ٶ����ơ�������
	formatTrans32Struct_t vel_limit[2]; 
	
	//�������ơ�������
	formatTrans32Struct_t current_limit[2];

	

	//freertos���
	TaskHandle_t xHandleTask;
	uint8_t dataInitFlag;
	uint32_t loops;
	
	errorScanStruct_t OdError;
	
	
	
	//CAN�ĵ������CMD�ṹ��
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
void ODSendVel_gainData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);
void ODSendVel_integrator_gainData(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);
void OdriveInit(void);
void ODRequestedState(void);	
void ODFlashSave(void);	
void ODClearError(void);		
void ODFReadMotorPairs(void);	
void ODSetLimit(void);	



extern OdriveStruct_t OdriveData; 
extern ODCanDataRecv_t OdReceivedData;
extern ODCANSendStruct_t ODSendData;


#endif
