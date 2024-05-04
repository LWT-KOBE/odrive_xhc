#include "warning.h"
#include "config.h"
#include "supervisor.h"
#include "shoot.h"

warningStruct_t warningData;

void lightBarsReportErrors(void){																			//���ڱ���
	uint8_t index = 0;
	uint8_t nowFaultBit = 0; 
	uint16_t remainFaultSate; 
	if((WARNING_STATE & 0x3f) != 0){
		remainFaultSate = (WARNING_STATE & 0x3f);
		while(index < SK6812_LED_STRIP_LENGTH && !warningData.highestFaultFlag){		//���ڼ�����һ���Ĵ����
			nowFaultBit = remainFaultSate % 2; 
			if(nowFaultBit){
				warningData.highestFault = index;
				digitalHi(&warningData.highestFaultFlag);
			}
			remainFaultSate = remainFaultSate >> 1;
			index ++;
		}
		digitalLo(&warningData.highestFaultFlag);
	}
	index = 0;
	if(warningData.highestFault != 0){																	//�ڵ�һλ����ߴ���λ����һ������Ч�������ڰ�������
		while(index <= warningData.highestFault){
			if(index >= warningData.highestFaultLoop)
				setOneLedHsv(index,&SK6812_DARK);
			else
				setOneLedHsv(index,&SK6812_GREEN);
			index ++;
		}
	}
	index = warningData.highestFault;
	remainFaultSate = (WARNING_STATE & 0x3f) >> warningData.highestFault;
	while(index < SK6812_LED_STRIP_LENGTH){
		nowFaultBit = remainFaultSate % 2;
		if(nowFaultBit){
			setOneLedHsv(index,&SK6812_RED);
		}
		else{
			setOneLedHsv(index,&SK6812_GREEN);
		}
		remainFaultSate = remainFaultSate >> 1;
		index ++;
	}
	if(!(warningData.loops % 5)){					//500ms
		digitalIncreasing(&warningData.highestFaultLoop);
		if(warningData.highestFaultLoop > warningData.highestFault)
			digitalClan(&warningData.highestFaultLoop);
	}
	else{
		warningData.highestFault = 0;
	}
}

void lightBarsOfContrl(uint8_t contrlMode,uint8_t safeMode){					//���ڿ���״̬
	uint8_t index = 0;
	uint8_t invertFreq = 0;
	static uint8_t lightBarsSwitch = ENABLE;
	switch(contrlMode){																									//����ģʽ
		case MANUAL_SINGLE:
			warningData.displayNumber = 1;
			break;
		case MANUAL_CONTINUOUS:
			warningData.displayNumber = 2;
			break;
		case AUTO_CONTINUOUS:
			warningData.displayNumber = 3;
			break;
	}
	switch(safeMode){																										//ʶ�𰲱��ȼ�
		case SAFE:
			warningData.blinkFrequency = 1;
			warningData.displayColor = SK6812_GREEN;
			break;
		/**********�޾���ģʽ���δ˴�*****************
		case WARNING:
			warningData.blinkFrequency = 2;
			warningData.displayColor = SK6812_YELLOW;
			break;
		********************************************/
		case DANGEROUS:
			warningData.blinkFrequency = 5;
			warningData.displayColor = SK6812_RED;
			break;
	}
	invertFreq = (1000 / WARNING_STACK_PERIOD) / warningData.blinkFrequency;
	if(!(warningData.loops % invertFreq)){
		lightBarsSwitch = !lightBarsSwitch;
		if(invertFreq == 10)//��ȫģʽ����
			lightBarsSwitch = ENABLE;
	}
	if(!lightBarsSwitch){
		warningData.displayColor = SK6812_DARK;
	}
	
	//��������ʾ��Ƶ������������������
	while(index < SK6812_LED_STRIP_LENGTH){
		if(index < warningData.displayNumber){
			setOneLedHsv(index,&warningData.displayColor);
		}
		else{
			setOneLedHsv(index,&SK6812_DARK);
		}
		index ++;
	}
	//�Ҳ����������ʾŤ��
	if(warningData.aviodFlag)
		warningData.displayColor = SK6812_RED;
	else if(warningData.rotateFlag)
		warningData.displayColor = SK6812_GREEN;
	else
		warningData.displayColor = SK6812_DARK;
	setOneLedHsv(SK6812_LED_STRIP_LENGTH - 3,&warningData.displayColor);
	//�Ҳ�ڶ�������������ʾ����״̬
	switch(warningData.currentTask){																	//���������Ҳ�ĵ���������ʾ����״̬
		case R_TASK:
			warningData.displayColor = SK6812_RED;													
			break;	
		case V_TASK:
			warningData.displayColor = SK6812_GREEN;
			break;
		case Z_TASK:
			warningData.displayColor = SK6812_YELLOW;
			break;
		default:
			warningData.displayColor = SK6812_DARK;
			break;
	}
	setOneLedHsv(SK6812_LED_STRIP_LENGTH - 2,&warningData.displayColor);
	//���ұߵĵ���ʾ����״̬
	//���ݳ��
	bool CapLight = false;
	if(warningData.chargeFlag){
		if(warningData.capSoc == 0x01){
			//���糤��
			warningData.displayColor = SK6812_GREEN;
		}
		else if(warningData.capSoc == 0x00){
			static uint8_t index = 0;
			if(!(warningData.loops % 2)){
				if(index){
					warningData.displayColor = SK6812_GREEN;
					index = 0;
				}
				else{
					warningData.displayColor = SK6812_DARK;
					index = 1;
				}	
			}
		}
		CapLight = true;
	}
	//���ݷŵ�
	else{
		if(warningData.capSoc == 0x02){
			//���ݼ���û�磬����
			warningData.displayColor = SK6812_RED;
		}
		else if(warningData.capSoc == 0x03){
			//��ȫ����
			warningData.displayColor = SK6812_GREEN;
		}
		CapLight = true;
	}
	//������ʱ�Ʋ���
	if(!warningData.linkCapFlag){
		warningData.displayColor = SK6812_DARK;
		CapLight = true;
	}
	/*if(warningData.judgeDataMask == 0x07){  //�ǵøĻ���
		warningData.displayColor = SK6812_GREEN;
		CapLight = true;
	}else{
		CapLight = false;
	}*/
	if(CapLight)
		setOneLedHsv(SK6812_LED_STRIP_LENGTH - 1,&warningData.displayColor);
	if(wiredControlData.cmd.robotMode == MODE_KM && !shootData.FricONflag){
		setAllLedColors(&SK6812_RED);
	}
}

void lightBarsStateUpdata(void){
	if(WARNING_STATE & 0x0080)
		shootData.shootStatusMode = SAFE;
	else if(WARNING_STATE & 0x0100)
		shootData.shootStatusMode = DANGEROUS;
	if(WARNING_STATE & 0x0200)
		warningData.aviodFlag = true;
	else
		warningData.aviodFlag = false;
	
	if(WARNING_STATE & 0x0400)
		warningData.rotateFlag = true;
	else
		warningData.rotateFlag = false;
	
	if(WARNING_STATE & 0x0800)
		warningData.chargeFlag = true;
	else
		warningData.chargeFlag = false;
	
	if(WARNING_STATE & 0x1000)
		warningData.linkCapFlag = true;
	else
		warningData.linkCapFlag = false;
	
	if(WARNING_STATE & 0x2000)
		warningData.currentTask = R_TASK;
	else if(WARNING_STATE & 0x4000)
		warningData.currentTask = V_TASK;
	else if(WARNING_STATE & 0x8000)
		warningData.currentTask = Z_TASK;
	else
		warningData.currentTask = NO_TASK;
}

void lightBarsUpdate(void){																					//�ƴ�״̬����
	lightBarsStateUpdata();
	if(!warningData.reportError){										
		lightBarsReportErrors();																				//û�н�������ctrl��ʾ����״̬
	}
	else{																															//�����ӵ�п���Ȩ�������
		switch(warningData.typeOfRobot){
			case INFANTRY_ID:
				lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode); 
				break;
			case TANK_ID:
				lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode);
				break;
			case AUXILIARY_ID:																						//���̳����˻����滹�����ǵ�����		
				break;
			case SENTRY_ID:																								//�ڱ����õƴ�
				break;
			case UAV_ID:
				lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode);
				break;
		}
	}
	SK6812UpdateStrip();
}

void warningUpdate(void){
  lightBarsUpdate();																								//sk6812 ����
	digitalIncreasing(&warningData.loops);
}
