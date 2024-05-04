#ifndef __DRIVER_ODRIVE_H
#define __DRIVER_ODRIVE_H

#include "stm32f4xx.h"
#include "Util.h"

#define AXIS0_ID 0x001 //M0��CANID
#define AXIS1_ID 0x002 //M1��CANID

enum{	
	axis0=0,	
	axis1,		
	
};

        enum MOTOR_Error {
            MOTOR_ERROR_NONE                       = 0x00000000,
            ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000001,
            ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000002,
            ERROR_ADC_FAILED                 = 0x00000004,
            ERROR_DRV_FAULT                  = 0x00000008,
            ERROR_CONTROL_DEADLINE_MISSED    = 0x00000010,
            ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 0x00000020,
            ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 0x00000040,
            ERROR_MODULATION_MAGNITUDE       = 0x00000080,
            ERROR_BRAKE_DEADTIME_VIOLATION   = 0x00000100,
            ERROR_UNEXPECTED_TIMER_CALLBACK  = 0x00000200,
            ERROR_CURRENT_SENSE_SATURATION   = 0x00000400,
            ERROR_CURRENT_LIMIT_VIOLATION    = 0x00001000,
            ERROR_BRAKE_DUTY_CYCLE_NAN       = 0x00002000,
            ERROR_DC_BUS_OVER_REGEN_CURRENT  = 0x00004000,
            ERROR_DC_BUS_OVER_CURRENT        = 0x00008000,
        };

        enum ENCODER_Error {
            ENCODER_ERROR_NONE                       = 0x00000000,
            ERROR_UNSTABLE_GAIN              = 0x00000001,
            ERROR_CPR_POLEPAIRS_MISMATCH     = 0x00000002,
            ERROR_NO_RESPONSE                = 0x00000004,
            ERROR_UNSUPPORTED_ENCODER_MODE   = 0x00000008,
            ERROR_ILLEGAL_HALL_STATE         = 0x00000010,
            ERROR_INDEX_NOT_FOUND_YET        = 0x00000020,
            ERROR_ABS_SPI_TIMEOUT            = 0x00000040,
            ERROR_ABS_SPI_COM_FAIL           = 0x00000080,
            ERROR_ABS_SPI_NOT_READY          = 0x00000100,
        };		
		
		
		

typedef  enum {
        MSG_CO_NMT_CTRL = 0x000,       // CANOpen NMT Message REC
        MSG_ODRIVE_HEARTBEAT, //0x001
        MSG_ODRIVE_ESTOP, //0x002
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,//0x004
        MSG_GET_SENSORLESS_ERROR,//0x005
        MSG_SET_AXIS_NODE_ID,//0x006
        MSG_SET_AXIS_REQUESTED_STATE, //0x007
        MSG_SET_AXIS_STARTUP_CONFIG,//0x008
        MSG_GET_ENCODER_ESTIMATES,//0x009
        MSG_GET_ENCODER_COUNT,//0x00A
        MSG_SET_CONTROLLER_MODES,//0x00B
        MSG_SET_INPUT_POS,//0x00C
        MSG_SET_INPUT_VEL,//0x00D
        MSG_SET_INPUT_TORQUE,//0x00E
        MSG_SET_VEL_LIMIT,//0x00F
        MSG_START_ANTICOGGING,//0x010
        MSG_SET_TRAJ_VEL_LIMIT,//0x011
        MSG_SET_TRAJ_ACCEL_LIMITS,//0x012
        MSG_SET_TRAJ_INERTIA,//0x013
        MSG_GET_IQ,//0x014
        MSG_GET_SENSORLESS_ESTIMATES,//0x015
        MSG_RESET_ODRIVE, //0x016
        MSG_GET_VBUS_VOLTAGE, //0x017
        MSG_CLEAR_ERRORS,
        MSG_GET_TEMP,//19	
        MSG_SAVE_CONFIG,//20		
        MSG_CO_HEARTBEAT_CMD = 0x01F,  // CANOpen NMT Heartbeat  SEND
    }ODCmdStruct_t;

enum{
	CONTROL_MODE_VOLTAGE_CONTROL     = 0,//��ѹ����ģʽ
	CONTROL_MODE_TORQUE_CONTROL      = 1,//���ؿ���ģʽ
	CONTROL_MODE_VELOCITY_CONTROL    = 2,//�ٶȿ���ģʽ
	CONTROL_MODE_POSITION_CONTROL    = 3,//λ�ÿ���ģʽ
} ;

enum{
	INPUT_MODE_INACTIVE              = 0,// ����Ծ������ģʽ
	INPUT_MODE_PASSTHROUGH           = 1,//ͨ������ģʽ
	INPUT_MODE_VEL_RAMP              = 2,//�ٶ�б��ģʽ
	INPUT_MODE_POS_FILTER            = 3,//λ�ù���ģʽ��
	INPUT_MODE_MIX_CHANNELS          = 4,//�������ģʽ��
	INPUT_MODE_TRAP_TRAJ             = 5,//���ι켣ģʽ
	INPUT_MODE_TORQUE_RAMP           = 6,//����б��ģʽ
	INPUT_MODE_MIRROR                = 7,//����ģʽ
} ;

typedef enum {
	AXIS_STATE_UNDEFINED             = 0,         // δ����״̬
	AXIS_STATE_IDLE                  = 1,		 //����״̬
	AXIS_STATE_STARTUP_SEQUENCE      = 2,		//����˳��
	AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,	//ȫ��У׼״̬
	AXIS_STATE_MOTOR_CALIBRATION     = 4,		//���У׼״̬
	AXIS_STATE_SENSORLESS_CONTROL    = 5,		//�޴���������
	AXIS_STATE_ENCODER_INDEX_SEARCH  = 6,		//Ѱ�Ҵű����������ź�״̬
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7,	//���ƫ��У׼״̬
	AXIS_STATE_CLOSED_LOOP_CONTROL   = 8,		//����ջ�ģʽ
	AXIS_STATE_LOCKIN_SPIN           = 9,		//�����ת��������ģʽ
	AXIS_STATE_ENCODER_DIR_FIND      = 10,		// Ѱ�ұ������Ļ����źţ�
	AXIS_STATE_HOMING                = 11,		//��״̬�麽��
}ODAxisStateStruct_t;

#define Vbus_Voltage	0x017	//��ѯ���ߵ�ѹ   DATA[8]:AxisState 10 00 00 00 00 00 00  ����ǰ��λ���磺33 8D 56 41 00 00 00 00

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
	
	formatTrans32Struct_t shadow_count[2];//shadow
	
	formatTrans32Struct_t count_in_cpr[2];//CPR
	
	
	formatTrans32Struct_t pos_estimate[2];//λ��	
	formatTrans32Struct_t vel_estimate[2];//�ٶ�
	formatTrans32Struct_t Iq_measured[2];//����

}ODCanDataRecv_t;

typedef struct {   
		
	formatTrans32Struct_t InputPos[2];	
	formatTrans16Struct_t InputCur[2];	
	formatTrans16Struct_t InputVel[2];			
	
}ODPosControl_t;

typedef struct {   
		
	formatTrans32Struct_t InputVel[2];	
	formatTrans32Struct_t InputCur[2];	
	
}ODVelControl_t;



typedef struct {

	ODAxisStateStruct_t	AxisState;
	
	
//	ODControlModeStruct_t ControlMode;
//	ODInputModeStruct_t InputMode;	
	
	formatTrans32Struct_t ControlMode[2];
	formatTrans32Struct_t InputMode[2];	
	
	
	ODPosControl_t SetPos;
	ODVelControl_t SetVel;
	formatTrans32Struct_t SetCur[2];	
	
	
	
	uint8_t RequestedState;

	//����ODRIVE�����ò���
	uint8_t flashSave;
	
	uint32_t loops;
	//uint8_t Check_Dcbus;
	ODCmdStruct_t OdriveCmd;
} OdriveStruct_t;	

extern OdriveStruct_t OdriveData; 
extern ODCanDataRecv_t OdReceivedData;


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
void ODSendControllerModes(CAN_TypeDef *CANx, uint32_t ID_CAN,uint32_t CMD_CAN, uint8_t len,OdriveStruct_t* Spetsnaz,uint8_t axis,ODCANSendStruct_t* CanSendData);






#endif
