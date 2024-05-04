#include "application.h"
#include "driver.h"

imuStruct_t imuData;
imuStruct_t* getimuData(){
    return &imuData;
}


void imuInit(void){
	
	//��&imuData�е�ǰλ�ú���� sizeof(imuData)���ֽ���0���棬����(void *)&imuData
	memset((void *)&imuData, 0, sizeof(imuData));     		
    
#ifdef  IMU_WT931   
	Driver_WT931_Init(MPU_WT931_USARTX,MPU_WT931_USARTX_RX_PIN,MPU_WT931_USARTX_TX_PIN,MPU_WT931_USART_PreemptionPriority,\
    MPU_WT931_USART_SubPriority); 
#endif
  
#ifdef IMU_BZ	
    
	sensorProcessInit();
	imuSensorData.deviceInitState = true;
	imuSensorData.Temp_Control =1;//ʹ�ܺ��¿��� 60��  
	dimuInit();
#endif
        
    usbVCP_Printf("imuInit Successfully \r\n");
    
}










