#include "driver_spl06.h"


BSP_SPI_TypeDef *SPL06_SPI = &BSP_SPI2;

static struct  {	//�ڲ�����У׼����
    int16 c0;
    int16 c1;
    int32 c00;
    int32 c10;
    int16 c01;
    int16 c11;
    int16 c20;
    int16 c21;
    int16 c30;       
}spl06_calib_param;


struct  {	
    uint8 chip_id; /**<chip id*/	
    int32 i32rawPressure;//ԭʼ��ѹ����
    int32 i32rawTemperature;//ԭʼ�¶�����
    int32 i32kP;    //��ѹ��������
    int32 i32kT;//�¶Ȳ�������
}spl06;



//��д����
void spl0601_write(uint8 REG,uint8 DATA)
{
	SPL06_CS_L;//CS������ΪSPIģʽ
	BSP_SPI_ReadWriteByte(SPL06_SPI,REG&0x7f);//���ͼĴ�����ַ+д����
	BSP_SPI_ReadWriteByte(SPL06_SPI,DATA);
	SPL06_CS_H;
}
uint8 spl0601_read(uint8 REG)
{
	uint8 data;
	SPL06_CS_L;//CS������ΪSPIģʽ
	BSP_SPI_ReadWriteByte(SPL06_SPI,REG|0x80);//���ͼĴ�����ַ+������
	data = BSP_SPI_ReadWriteByte(SPL06_SPI,0xff);
	SPL06_CS_H;
	return data;
}

//ʲô�ǹ����������ǲ�������ȡһ�����ݣ������¶Ȳ���Ƶ��10HZ�����ù�����Ϊ8������ÿ���¶ȶ�ȡ����8��ȡƽ�����ڲ�ʵ������80HZ�����ݲɼ�����̨ģʽ�޹�����
/***********************************************************************
 * ��ѹ����������
 * @param[in] ��̨ģʽbackground_rate����Ƶ��N��/ÿ�루������̨ģʽʹ��,����ģʽ����Ƶ�����û��Լ���������С���ȡ���ڹ���������ʱ��,��ͬ�Ĺ�����ʱ������в�ͬ�Ĺ��ģ���oversamply����������
 * @param[out] 
 * @return     
 **********************************************************************/


void spl06_pressure_rate_config(u8 background_rate,u8 oversamply)
{
		u8 data;
	
		data = (background_rate<<4)|oversamply;
		if(oversamply>PRESSURE_RATE_8_TIMES)//��������������EMPERATURE_RATE_8_TIMES��Ӧ���������ݱ��µ����ݸ��ǣ��ڲ�ӵ����ѹ���¶ȹ�32����FIFO���ڴ���8�Σ�Ҳ���Ǵ��ڻ����16�ι���������ʱ����Ҫ���µ����ݸ��ǣ��������ݾͻᶪʧ
		{
				u8 data;
				data = spl0601_read(CFG_REG);//��ȡԭ�Ĵ���ֵ
				data |= 0X04;//P-SHIFTλ��1
				spl0601_write(CFG_REG, data);//����д�ؼĴ���					
		}		
		switch(oversamply)
    {
        case PRESSURE_RATE_2_TIMES:
          spl06.i32kP = 1572864;
            break;
        case PRESSURE_RATE_4_TIMES:
            spl06.i32kP  = 3670016;
            break;
        case PRESSURE_RATE_8_TIMES:
            spl06.i32kP  = 7864320;
            break;
        case PRESSURE_RATE_16_TIMES:
            spl06.i32kP = 253952;
            break;
        case PRESSURE_RATE_32_TIMES:
            spl06.i32kP = 516096;       
            break;
        case PRESSURE_RATE_64_TIMES:
            spl06.i32kP = 1040384;           
            break;
        case PRESSURE_RATE_128_TIMES:
            spl06.i32kP = 2088960;
            break;
        case PRESSURE_RATE_1_TIMES:
        default:
            spl06.i32kP = 524288;
            break;
    }		
		spl0601_write( PRS_CFG_REG, data);//д������
}

/***********************************************************************
 * �¶Ȳ���������
 * @param[in] background_rate����Ƶ��N��/ÿ�루������̨ģʽʹ��,����ģʽ����Ƶ�����û��Լ���������С���ȡ���ڹ���������ʱ�䣩��oversamply���������� ext�¶ȼ�ѡ��
 * @param[out] 
 * @return     
 **********************************************************************/

void spl06_temperature_rate_config(u8 background_rate,u8 oversamply,u8 ext)
{
		u8 data;
	
			data = (ext<<7)|(background_rate<<4)|oversamply;
			if(oversamply>TEMPERATURE_RATE_8_TIMES)//��������������EMPERATURE_RATE_8_TIMES��Ӧ���������ݱ��µ����ݸ���
			{
					u8 data;
					data = spl0601_read( CFG_REG);//��ȡԭ�Ĵ���ֵ
					data |= 0X08;//T-SHIFTλ��1
					spl0601_write( CFG_REG, data);	//����д�ؼĴ���					
			}			
		switch(oversamply)
    {
        case TEMPERATURE_RATE_2_TIMES:
          spl06.i32kT = 1572864;
            break;
        case TEMPERATURE_RATE_4_TIMES:
            spl06.i32kT  = 3670016;
            break;
        case TEMPERATURE_RATE_8_TIMES:
            spl06.i32kT  = 7864320;
            break;
        case TEMPERATURE_RATE_16_TIMES:
            spl06.i32kT = 253952;
            break;
        case TEMPERATURE_RATE_32_TIMES:
            spl06.i32kT = 516096;       
            break;
        case TEMPERATURE_RATE_64_TIMES:
            spl06.i32kT = 1040384;           
            break;
        case TEMPERATURE_RATE_128_TIMES:
            spl06.i32kT = 2088960;
            break;
        case TEMPERATURE_RATE_1_TIMES:
        default:
            spl06.i32kT = 524288;
            break;
    }		
		spl0601_write( TMP_CFG_REG, data);//д������
}
/***********************************************************************
 * ����������ģʽ������״̬��ȡ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/

//��ȡ���������ݾ�λ״̬//����������״̬
u8 spl06_get_measure_status(void)
{
	return spl0601_read( MEAS_CFG_REG);
}
//���ö�ȡģʽ+��ȡ��ʽ
void spl06_set_measure_mode(u8 mode)  //����Ϊģʽֵ
{
	 spl0601_write( MEAS_CFG_REG,mode);
}
//��������ģʽ��ȡ�¶�ֵ
void spl06_start_temperature(void)
{
    spl0601_write( MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_COMMAND_TMP);
}
//��������ģʽ��ȡ��ѹֵ
void spl06_start_pressure(void)
{
    spl0601_write(MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_COMMAND_PRS);
}
//�������ģʽ�������ֹͣ�ɼ�����ֱ���ٴ��л�ģʽ
void spl06_enter_standby(void)
{
    spl0601_write( MEAS_CFG_REG, MEAS_CFG_MEAS_CTR_STANDBY);
}

/***********************************************************************
 * �ж���FIFO���á�SPIģʽ����
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/


void spl06_set_interrupt(u8 interrupt,u8 type)//�����ж�ʹ��
{
	u8 data;
	data = spl0601_read( CFG_REG);
	if(type!=ENABLE )
		data &= ~interrupt;
	else
		data |= interrupt;	
	spl0601_write( CFG_REG,data);
}

void spl06_set_spi_wire(u8 wire)//����SPI���� //3��/4��SPI��ȡ����
{
	u8 data;
	data = spl0601_read(CFG_REG);
	data &= 0xf7;//SPI����������λ��0
	data |= wire;
	spl0601_write( CFG_REG,data);
}

void spl06_set_intrupt_level(u8 level)//�����ж���Ч��ƽ//INT�ߵ�ƽ��Ч���ߵ͵�ƽ��Ч//levelΪ0��͵�ƽ��Ч,Ϊ1��ߵ�ƽ��Ч
{
	u8 data;
	data = spl0601_read( CFG_REG);
	data &= 0x7f;//�жϵ�ƽ��Чλ��0
	data |= level<<7;
	spl0601_write( CFG_REG,data);
}

/***********************************************************************
 * �ж�״̬��ȡ����Ӳ����1����������������жϣ���ȡ�üĴ����Զ���0
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/


u8 spl06_get_int_status(void)//��ȡ������״̬
{
	return spl0601_read( INT_STS_REG);
}

/***********************************************************************
 * FIFO״̬��ȡ��
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/

u8 spl06_get_fifo_status(void)
{
	return spl0601_read( FIFO_STS_REG);
}
/***********************************************************************
 *��λ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/


void spl06_soft_reset(void)//�����λ
{
	 spl0601_write(RESET_REG,RESET_SOFT);
}
void spl06_reset_fifo(void)//�����FIFO
{
  spl0601_write(RESET_REG,RESET_FIFO_FLUSH);
}
/***********************************************************************
 *ID
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
u8 spl06_get_chip_id(void)//��ȡ��ƷID//��ȡ��Ʒ�汾//���ڰ汾�ڲ�ͬ�Ĵ������в�ͬ��������ֻ�ж�ID��ʶ��SPL06
{
	return spl0601_read( ID_REG);
}

/***********************************************************************
 * ��ȡ��ѹ���ڲ���У׼����
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void spl0601_get_calib_param(void)//�ڲ�У׼ֵ//��ѹ�ƽ����Լ��²�ʹ��//���ڲ������趨
{
    unsigned long h;
    unsigned long m;
    unsigned long l;
    h =  spl0601_read( 0x10);
    l  =  spl0601_read(  0x11);
    spl06_calib_param.c0 = (int16)h<<4 | l>>4;
    spl06_calib_param.c0 = (spl06_calib_param.c0&0x0800)?(0xF000|spl06_calib_param.c0):spl06_calib_param.c0;
    h =  spl0601_read(  0x11);
    l  =  spl0601_read(  0x12);
    spl06_calib_param.c1 = (int16)(h&0x0F)<<8 | l;
    spl06_calib_param.c1 = (spl06_calib_param.c1&0x0800)?(0xF000|spl06_calib_param.c1):spl06_calib_param.c1;
    h =  spl0601_read(  0x13);
    m =  spl0601_read(  0x14);
    l =  spl0601_read(  0x15);
    spl06_calib_param.c00 = (int32)h<<12 | (int32)m<<4 | (int32)l>>4;
    spl06_calib_param.c00 = (spl06_calib_param.c00&0x080000)?(0xFFF00000|spl06_calib_param.c00):spl06_calib_param.c00;
    h =  spl0601_read(  0x15);
    m =  spl0601_read(  0x16);
    l =  spl0601_read(  0x17);
    spl06_calib_param.c10 = (int32)h<<16 | (int32)m<<8 | l;
    spl06_calib_param.c10 = (spl06_calib_param.c10&0x080000)?(0xFFF00000|spl06_calib_param.c10):spl06_calib_param.c10;
    h =  spl0601_read(  0x18);
    l  =  spl0601_read(  0x19);
    spl06_calib_param.c01 = (int16)h<<8 | l;
    h =  spl0601_read(  0x1A);
    l  =  spl0601_read(  0x1B);
    spl06_calib_param.c11 = (int16)h<<8 | l;
    h =  spl0601_read(  0x1C);
    l  =  spl0601_read(  0x1D);
    spl06_calib_param.c20 = (int16)h<<8 | l;
    h =  spl0601_read(  0x1E);
    l  =  spl0601_read(  0x1F);
    spl06_calib_param.c21 = (int16)h<<8 | l;
    h =  spl0601_read(  0x20);
    l  =  spl0601_read(  0x21);
    spl06_calib_param.c30 = (int16)h<<8 | l;
}
/***********************************************************************
 * ��ʼ��
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
u8 spl0601_init(void)
{
		u8 spl06_start_status;
		//�ȴ��ڲ�У׼���ݿ���
		BSP_SPI_Init(SPL06_SPI);
	
	
		do
			spl06_start_status = spl06_get_measure_status();//��ȡ��ѹ������״̬
		while((spl06_start_status&MEAS_CFG_COEF_RDY)!=MEAS_CFG_COEF_RDY);
		//��ȡ�ڲ�У׼ֵ	
    spl0601_get_calib_param();	
		//�ȴ��������ڲ���ʼ�����
		do
			spl06_start_status = spl06_get_measure_status();//��ȡ��ѹ������״̬
		while((spl06_start_status&MEAS_CFG_SENSOR_RDY)!=MEAS_CFG_SENSOR_RDY);
		//��ȡCHIP ID
	  spl06.chip_id = spl06_get_chip_id();
		//�ж϶�ȡ��ID�Ƿ���ȷ������ֻ�жϸ�4λ��ID�����жϵ�4λ�İ汾��
		if((spl06.chip_id&0xf0)!=PRODUCT_ID)
			return 0;//���ID��ȡʧ�ܣ��򷵻�ʧ��
		//��̨���ݲ�������128HZ ��������32��
	  spl06_pressure_rate_config(PRESSURE_RATE_128_TIMES,PRESSURE_RATE_32_TIMES);
		//��̨���ݲ�������32HZ ��������8��//���ô������ϵ��¶ȼ���Ϊ�¶Ȳɼ�
	  spl06_temperature_rate_config(TEMPERATURE_RATE_32_TIMES,TEMPERATURE_RATE_8_TIMES,TEMPERATURE_RATE_TMP_EXT_EXTERNAL);
		//������̨��ȡ����
		spl06_set_measure_mode(MEAS_CFG_MEAS_CTR_BACKGROUND_PSR_TMP);
		return 1;//��ʼ���ɹ�
}


/***********************************************************************
 * ��ȡԭʼ�¶�ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void spl0601_get_raw_temp(void)
{
    uint8 h[3] = {0};
    
		h[0] = spl0601_read(  0x03);
		h[1] = spl0601_read(  0x04);
		h[2] = spl0601_read(  0x05);

    spl06.i32rawTemperature = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
    spl06.i32rawTemperature= (spl06.i32rawTemperature&0x800000) ? (0xFF000000|spl06.i32rawTemperature) : spl06.i32rawTemperature;
}

/***********************************************************************
 * ��ȡԭʼ��ѹֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void spl0601_get_raw_pressure(void)
{
    uint8 h[3];
    
		h[0] = spl0601_read(  0x00);
		h[1] = spl0601_read(  0x01);
		h[2] = spl0601_read(  0x02);
    
    spl06.i32rawPressure = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
    spl06.i32rawPressure= (spl06.i32rawPressure&0x800000) ? (0xFF000000|spl06.i32rawPressure) : spl06.i32rawPressure;
}


/***********************************************************************
 * �¶Ƚ���ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = spl06.i32rawTemperature / (float)spl06.i32kT;
    fTCompensate =  spl06_calib_param.c0 * 0.5 + spl06_calib_param.c1 * fTsc;
    return fTCompensate;
}

/***********************************************************************
 * ��ѹ���㲢�����¶Ȳ���
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = spl06.i32rawTemperature / (float)spl06.i32kT;
    fPsc = spl06.i32rawPressure / (float)spl06.i32kP;
    qua2 = spl06_calib_param.c10 + fPsc * (spl06_calib_param.c20 + fPsc* spl06_calib_param.c30);
    qua3 = fTsc * fPsc * (spl06_calib_param.c11 + fPsc * spl06_calib_param.c21);
		//qua3 = 0.9f *fTsc * fPsc * (spl06_calib_param.c11 + fPsc * spl06_calib_param.c21);
	
    fPCompensate = spl06_calib_param.c00 + fPsc * qua2 + fTsc * spl06_calib_param.c01 + qua3;
		//fPCompensate = spl06_calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06_calib_param.c01 + qua3;
    return fPCompensate;
}



/***********************************************************************
 * ��ȡ�¶�ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float user_spl0601_get_temperature()
{
		spl0601_get_raw_temp();//��ȡ�¶�ԭʼֵ
		return spl0601_get_temperature();//�¶Ƚ�����ֵ
}
/***********************************************************************
 * ��ȡ��ѹ���¶Ȳ���ֵ
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
float user_spl0601_get_presure()
{
		spl0601_get_raw_pressure();//��ȡ��ѹֵԭʼֵ
		return spl0601_get_pressure();	//��ѹ���㲢�����¶Ȳ��������ѹֵ
}



float baro_Offset, alt_3, height;
unsigned char baro_start;
float temperature, alt_high;
float baro_pressure;

void Drv_Spl0601_Read ( void )
{

    imuSensorData.Spl06Data.temperature = user_spl0601_get_temperature();
	imuSensorData.Spl06Data.presure = user_spl0601_get_presure();			

    alt_3 = ( 101400 - imuSensorData.Spl06Data.presure ) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101400 - imuSensorData.Spl06Data.presure ) * 100.0f ;


    imuSensorData.Spl06Data.baro_height = ( height - baro_Offset ) ; //cm +


}


