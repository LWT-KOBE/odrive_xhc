#include "driver_flash.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "board.h"
systemConfig_t  systemConfigData __attribute__((at(0x10006000)));
systemConfig_t *getConfigData(void){
    return &systemConfigData;
}



/*----------------------------
== �˴����������ɳ���16���ַ� ==
----------------------------*/
const char *configParameterStrings[] = {
	"CONFIG_VERSION",									//�汾��
	"YAW_ANGLE_P",
	"YAW_ANGLE_I",
	"YAW_ANGLE_D",
	"YAW_ANGLE_F",
	"YAW_ANGLE_PM",
	"YAW_ANGLE_IM",
	"YAW_ANGLE_DM",
	"YAW_ANGLE_OM",    
  
    
	"PITCH_ANGLE_P",
	"PITCH_ANGLE_I",
	"PITCH_ANGLE_D",
	"PITCH_ANGLE_F",
	"PITCH_ANGLE_PM",
	"PITCH_ANGLE_IM",
	"PITCH_ANGLE_DM",
	"PITCH_ANGLE_OM", 
	"PITCH_ANGLE_MIX",
	"PITCH_ANGLE_MIN",         
    
	"PITCH_RATE_P",
	"PITCH_RATE_I",
	"PITCH_RATE_D",
	"PITCH_RATE_F",
	"PITCH_RATE_PM",
	"PITCH_RATE_IM",
	"PITCH_RATE_DM",
	"PITCH_RATE_OM",
	"YAW_RATE_P",
	"YAW_RATE_I",
	"YAW_RATE_D",
	"YAW_RATE_F",
	"YAW_RATE_PM",
	"YAW_RATE_IM",
	"YAW_RATE_DM",
	"YAW_RATE_OM",   


    "PITCH_CENTER",
	"YAW_CENTER",	   
    "BACK_CENTER_TIME",    
	"YAW_CONFIG",    
	
	"YAW_FIX",	
	"YAW_TURN",	
	"YAW_TYPE",	
	"YAW_INSTALL",	

	"PITCH_FIX",	
	"PITCH_TURN",	
	"PITCH_TYPE",	
	"PITCH_INSTALL",

	"IMU_ACC_BIAS_X",
	"IMU_ACC_BIAS_Y",
	"IMU_ACC_BIAS_Z",
	"IMU_GYO_BIAS_X",
	"IMU_GYO_BIAS_Y",
	"IMU_GYO_BIAS_Z",
	"IMU_MAG_BIAS_X",
	"IMU_MAG_BIAS_Y",
	"IMU_MAG_BIAS_Z",	
	
	
};
void configLoadDefault(void){	
	systemConfigData.configVersion = DEFAULT_CONFIG_VERSION;
    
	systemConfigData.yawAnglePID.PID_P = DEFAULT_YAW_ANGLE_P;
	systemConfigData.yawAnglePID.PID_I = DEFAULT_YAW_ANGLE_I;
	systemConfigData.yawAnglePID.PID_D = DEFAULT_YAW_ANGLE_D;
	systemConfigData.yawAnglePID.PID_F = DEFAULT_YAW_ANGLE_F;
	systemConfigData.yawAnglePID.PID_PM = DEFAULT_YAW_ANGLE_PM;
	systemConfigData.yawAnglePID.PID_IM = DEFAULT_YAW_ANGLE_IM;
	systemConfigData.yawAnglePID.PID_DM = DEFAULT_YAW_ANGLE_DM;
	systemConfigData.yawAnglePID.PID_OM = DEFAULT_YAW_ANGLE_OM;

  
	systemConfigData.pitchAnglePID.PID_P = DEFAULT_PITCH_ANGLE_P;
	systemConfigData.pitchAnglePID.PID_I = DEFAULT_PITCH_ANGLE_I;
	systemConfigData.pitchAnglePID.PID_D = DEFAULT_PITCH_ANGLE_D;
	systemConfigData.pitchAnglePID.PID_F = DEFAULT_PITCH_ANGLE_F;
	systemConfigData.pitchAnglePID.PID_PM = DEFAULT_PITCH_ANGLE_PM;
	systemConfigData.pitchAnglePID.PID_IM = DEFAULT_PITCH_ANGLE_IM;
	systemConfigData.pitchAnglePID.PID_DM = DEFAULT_PITCH_ANGLE_DM;
	systemConfigData.pitchAnglePID.PID_OM = DEFAULT_PITCH_ANGLE_OM;


	systemConfigData.pitchRatePID.PID_P = DEFAULT_PITCH_RATE_P;
	systemConfigData.pitchRatePID.PID_I = DEFAULT_PITCH_RATE_I;
	systemConfigData.pitchRatePID.PID_D = DEFAULT_PITCH_RATE_D;
	systemConfigData.pitchRatePID.PID_F = DEFAULT_PITCH_RATE_F;
	systemConfigData.pitchRatePID.PID_PM = DEFAULT_PITCH_RATE_PM;
	systemConfigData.pitchRatePID.PID_IM = DEFAULT_PITCH_RATE_IM;
	systemConfigData.pitchRatePID.PID_DM = DEFAULT_PITCH_RATE_DM;
	systemConfigData.pitchRatePID.PID_OM = DEFAULT_PITCH_RATE_OM;
    
   
	systemConfigData.yawRatePID.PID_P = DEFAULT_YAW_RATE_P;
	systemConfigData.yawRatePID.PID_I = DEFAULT_YAW_RATE_I;
	systemConfigData.yawRatePID.PID_D = DEFAULT_YAW_RATE_D;
	systemConfigData.yawRatePID.PID_F = DEFAULT_YAW_RATE_F;
	systemConfigData.yawRatePID.PID_PM = DEFAULT_YAW_RATE_PM;
	systemConfigData.yawRatePID.PID_IM = DEFAULT_YAW_RATE_IM;
	systemConfigData.yawRatePID.PID_DM = DEFAULT_YAW_RATE_DM;
	systemConfigData.yawRatePID.PID_OM = DEFAULT_YAW_RATE_OM;
 
       

    
	systemConfigData.yawCenter= DEFAULT_YAW_CENTER;								
	systemConfigData.pitchCenter = DEFAULT_PITCH_CENTER;	
	systemConfigData.backCenterTime = DEFAULT_BACK_CENTER_TIME;		


    systemConfigData.yawSetting.u8_temp[0] = DEFAULT_YAW_FIX;
	systemConfigData.yawSetting.u8_temp[1] = DEFAULT_YAW_TURN;
	systemConfigData.yawSetting.u8_temp[2] = DEFAULT_YAW_TYPE;
    systemConfigData.yawSetting.u8_temp[3] = DEFAULT_YAW_INSTALL;		

    systemConfigData.pitchSetting.u8_temp[0] = DEFAULT_PITCH_FIX;
	systemConfigData.pitchSetting.u8_temp[1] = DEFAULT_PITCH_TURN;
	systemConfigData.pitchSetting.u8_temp[2] = DEFAULT_PITCH_TYPE;
    systemConfigData.pitchSetting.u8_temp[3] = DEFAULT_PITCH_INSTALL;



	systemConfigData.imu_acc_bias_x = DEFAULT_IMU_ACC_BIAS_X;
	systemConfigData.imu_acc_bias_y = DEFAULT_IMU_ACC_BIAS_Y;
	systemConfigData.imu_acc_bias_z = DEFAULT_IMU_ACC_BIAS_Z;

	systemConfigData.imu_gyo_bias_x = DEFAULT_IMU_GYO_BIAS_X;
	systemConfigData.imu_gyo_bias_y = DEFAULT_IMU_GYO_BIAS_Y;
	systemConfigData.imu_gyo_bias_z = DEFAULT_IMU_GYO_BIAS_Z;

	systemConfigData.imu_mag_bias_x = DEFAULT_IMU_MAG_BIAS_X;
	systemConfigData.imu_mag_bias_y = DEFAULT_IMU_MAG_BIAS_Y;
	systemConfigData.imu_mag_bias_z = DEFAULT_IMU_MAG_BIAS_Z;

}


configToken_t *configTokenFindEmpty(void) {
	configToken_t *p = (configToken_t *)(FLASH_END_ADDR + 1);
	do {
		p--;
	} while (p->key != 0xffffffff);
	return p;
}

void configTokenStore(configToken_t *token) {
  flashAddress((uint32_t)configTokenFindEmpty(), (uint32_t *)token, sizeof(configToken_t)/sizeof(uint32_t));
}

configToken_t *configTokenGet(uint32_t key) {
	configToken_t *p, *t;
	p = (configToken_t *)(FLASH_END_ADDR + 1);
	t = 0;
	do {
		p--;
		if (p->key == key)
			t = p;
	} while (p->key != 0xffffffff);
	return t;
}
/*------------------- ����FLASH��ȡ ---------------------*/
void configFlashRead(void) {
    float *recs;
    int i;
    //�������׵�ַ��ʼ��ȡ
    recs = (void *)flashStartAddr();
    for (i = 0; i < sizeof(systemConfig_t); i++) {
        *((&systemConfigData.configVersion) + i) = recs[i];
    }    
    
    
}

configToken_t *configTokenIterate(configToken_t *t) {
	if (t == 0)
		t = (configToken_t *)(FLASH_END_ADDR + 1);
	t--;
	if (t->key != 0xffffffff)
		return t;
	else
		return 0;
}
/*------------------- ����FLASHд�� ---------------------*/
uint8_t configFlashWrite(void) {

    float *recs;
	uint8_t ret = 0;
	int i;
    
    //����һ�ζ�̬�ڴ����ڴ�����ʱ��Ϣ
	recs = (void *)aqCalloc(sizeof(systemConfig_t), sizeof(float));
    //����ɹ����뵽�ڴ��ַ��ִ��д�����
	if (recs) {
        //��������ʼ��ַ��ʼ����flash
		ret = flashErase(flashStartAddr(), sizeof(systemConfig_t) * sizeof(float) / sizeof(uint32_t));
        //ʹ�������ݻ�����Ч	        
		FLASH_DataCacheCmd(DISABLE);
        //�������ݻ���
		FLASH_DataCacheReset();
        //�������ݻ��湦��
		FLASH_DataCacheCmd(ENABLE);
        //��������ɹ���ִ��д�����
		if (ret) {
            //���ڴ��д��������б�
			for (i = 0; i < sizeof(systemConfig_t); i++) {
                recs[i] = *((&systemConfigData.configVersion) + i);
			}
            //��flash���ݣ���������ʼ��ַ��ǰ��
			ret = flashAddress(flashStartAddr(), (uint32_t *)recs, \
                                sizeof(systemConfig_t) * sizeof(float) / sizeof(uint32_t));
		}
        else {
            usbVCP_Printf("Flash erase failed! \r\n");
        }
        //�ͷ��ڴ�
		aqFree(recs, sizeof(systemConfig_t), sizeof(float));
	}
	else {
        usbVCP_Printf("Flash failed to apply for memory! \r\n");
    }
	return ret;




}

void configInit(void) {

	float ver;	
    //��Flash�п�ʼ
	configFlashRead();																							
    //��ȡ��ǰflash�汾
	ver = *(float *)(flashStartAddr());																
	if (isnan(ver))
		ver = 0.0f;
    //��������Ĭ��ֵ����flash�汾�ͼ��ذ汾																											
	if (DEFAULT_CONFIG_VERSION > ver || DEFAULT_CONFIG_VERSION > systemConfigData.configVersion){
        //����Ĭ��ֵ
		configLoadDefault();																					
		digitalHi(&getsupervisorData()->flashSave);	
	}
    //���flash�汾���ڵ�ǰ����ڵ�ǰ�汾
	else if (ver >= systemConfigData.configVersion){                                                              
        //��ȡflash   
        configFlashRead();																								
	}																			
    //������صİ汾����flash�汾
	else if (systemConfigData.configVersion > ver){                                                                          
        //д��flash,������ֻ��������SD��ʱ����SD���еİ汾����flash�еİ汾�Żᷢ��        
        configFlashWrite();	
	}																											
    usbVCP_Printf("ConfigInit Successfully \r\n");    
    
    
    
    
}


