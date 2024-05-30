//#include "application.h"
//#include "supervisor.h"
//#include "lcdfont.h"
//#include "pic.h"


//lcdStruct_t lcdData;



///******************************************************************************
//      ����˵������ָ�����������ɫ
//      ������ݣ�xsta,ysta   ��ʼ����
//                xend,yend   ��ֹ����
//								color       Ҫ������ɫ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
//{          
//	u16 i,j; 
//	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//������ʾ��Χ
//	for(i=ysta;i<yend;i++)
//	{													   	 	
//		for(j=xsta;j<xend;j++)
//		{
//			LCD_WR_DATA(color);
//		}
//	} 					  	    
//}


///******************************************************************************
//      ����˵������ָ��λ�û���
//      ������ݣ�x,y ��������
//                color �����ɫ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_DrawPoint(u16 x,u16 y,u16 color)
//{
//	LCD_Address_Set(x,y,x,y);//���ù��λ�� 
//	LCD_WR_DATA(color);
//} 


///******************************************************************************
//      ����˵��������
//      ������ݣ�x1,y1   ��ʼ����
//                x2,y2   ��ֹ����
//                color   �ߵ���ɫ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
//{
//	u16 t; 
//	int xerr=0,yerr=0,delta_x,delta_y,distance;
//	int incx,incy,uRow,uCol;
//	delta_x=x2-x1; //������������ 
//	delta_y=y2-y1;
//	uRow=x1;//�����������
//	uCol=y1;
//	if(delta_x>0)incx=1; //���õ������� 
//	else if (delta_x==0)incx=0;//��ֱ�� 
//	else {incx=-1;delta_x=-delta_x;}
//	if(delta_y>0)incy=1;
//	else if (delta_y==0)incy=0;//ˮƽ�� 
//	else {incy=-1;delta_y=-delta_y;}
//	if(delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
//	else distance=delta_y;
//	for(t=0;t<distance+1;t++)
//	{
//		LCD_DrawPoint(uRow,uCol,color);//����
//		xerr+=delta_x;
//		yerr+=delta_y;
//		if(xerr>distance)
//		{
//			xerr-=distance;
//			uRow+=incx;
//		}
//		if(yerr>distance)
//		{
//			yerr-=distance;
//			uCol+=incy;
//		}
//	}
//}


///******************************************************************************
//      ����˵����������
//      ������ݣ�x1,y1   ��ʼ����
//                x2,y2   ��ֹ����
//                color   ���ε���ɫ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
//{
//	LCD_DrawLine(x1,y1,x2,y1,color);
//	LCD_DrawLine(x1,y1,x1,y2,color);
//	LCD_DrawLine(x1,y2,x2,y2,color);
//	LCD_DrawLine(x2,y1,x2,y2,color);
//}


///******************************************************************************
//      ����˵������Բ
//      ������ݣ�x0,y0   Բ������
//                r       �뾶
//                color   Բ����ɫ
//      ����ֵ��  ��
//******************************************************************************/
//void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
//{
//	int a,b;
//	a=0;b=r;	  
//	while(a<=b)
//	{
//		LCD_DrawPoint(x0-b,y0-a,color);             //3           
//		LCD_DrawPoint(x0+b,y0-a,color);             //0           
//		LCD_DrawPoint(x0-a,y0+b,color);             //1                
//		LCD_DrawPoint(x0-a,y0-b,color);             //2             
//		LCD_DrawPoint(x0+b,y0+a,color);             //4               
//		LCD_DrawPoint(x0+a,y0-b,color);             //5
//		LCD_DrawPoint(x0+a,y0+b,color);             //6 
//		LCD_DrawPoint(x0-b,y0+a,color);             //7
//		a++;
//		if((a*a+b*b)>(r*r))//�ж�Ҫ���ĵ��Ƿ��Զ
//		{
//			b--;
//		}
//	}
//}

///******************************************************************************
//      ����˵������ʾ���ִ�
//      ������ݣ�x,y��ʾ����
//                *s Ҫ��ʾ�ĺ��ִ�
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ� ��ѡ 16 24 32
//                mode:  0�ǵ���ģʽ  1����ģʽ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
//{
//	while(*s!=0)
//	{
//		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
//		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
//		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
//		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
//		else return;
//		s+=2;
//		x+=sizey;
//	}
//}

///******************************************************************************
//      ����˵������ʾ����12x12����
//      ������ݣ�x,y��ʾ����
//                *s Ҫ��ʾ�ĺ���
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//                mode:  0�ǵ���ģʽ  1����ģʽ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
//{
//	u8 i,j,m=0;
//	u16 k;
//	u16 HZnum;//������Ŀ
//	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
//	u16 x0=x;
//	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
//	                         
//	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//ͳ�ƺ�����Ŀ
//	for(k=0;k<HZnum;k++) 
//	{
//		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
//		{ 	
//			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
//			for(i=0;i<TypefaceNum;i++)
//			{
//				for(j=0;j<8;j++)
//				{	
//					if(!mode)//�ǵ��ӷ�ʽ
//					{
//						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
//						else LCD_WR_DATA(bc);
//						m++;
//						if(m%sizey==0)
//						{
//							m=0;
//							break;
//						}
//					}
//					else//���ӷ�ʽ
//					{
//						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
//						x++;
//						if((x-x0)==sizey)
//						{
//							x=x0;
//							y++;
//							break;
//						}
//					}
//				}
//			}
//		}				  	
//		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
//	}
//} 

///******************************************************************************
//      ����˵������ʾ����16x16����
//      ������ݣ�x,y��ʾ����
//                *s Ҫ��ʾ�ĺ���
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//                mode:  0�ǵ���ģʽ  1����ģʽ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
//{
//	u8 i,j,m=0;
//	u16 k;
//	u16 HZnum;//������Ŀ
//	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
//	u16 x0=x;
//  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
//	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//ͳ�ƺ�����Ŀ
//	for(k=0;k<HZnum;k++) 
//	{
//		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
//		{ 	
//			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
//			for(i=0;i<TypefaceNum;i++)
//			{
//				for(j=0;j<8;j++)
//				{	
//					if(!mode)//�ǵ��ӷ�ʽ
//					{
//						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
//						else LCD_WR_DATA(bc);
//						m++;
//						if(m%sizey==0)
//						{
//							m=0;
//							break;
//						}
//					}
//					else//���ӷ�ʽ
//					{
//						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
//						x++;
//						if((x-x0)==sizey)
//						{
//							x=x0;
//							y++;
//							break;
//						}
//					}
//				}
//			}
//		}				  	
//		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
//	}
//} 


///******************************************************************************
//      ����˵������ʾ����24x24����
//      ������ݣ�x,y��ʾ����
//                *s Ҫ��ʾ�ĺ���
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//                mode:  0�ǵ���ģʽ  1����ģʽ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
//{
//	u8 i,j,m=0;
//	u16 k;
//	u16 HZnum;//������Ŀ
//	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
//	u16 x0=x;
//	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
//	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//ͳ�ƺ�����Ŀ
//	for(k=0;k<HZnum;k++) 
//	{
//		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
//		{ 	
//			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
//			for(i=0;i<TypefaceNum;i++)
//			{
//				for(j=0;j<8;j++)
//				{	
//					if(!mode)//�ǵ��ӷ�ʽ
//					{
//						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
//						else LCD_WR_DATA(bc);
//						m++;
//						if(m%sizey==0)
//						{
//							m=0;
//							break;
//						}
//					}
//					else//���ӷ�ʽ
//					{
//						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
//						x++;
//						if((x-x0)==sizey)
//						{
//							x=x0;
//							y++;
//							break;
//						}
//					}
//				}
//			}
//		}				  	
//		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
//	}
//} 

///******************************************************************************
//      ����˵������ʾ����32x32����
//      ������ݣ�x,y��ʾ����
//                *s Ҫ��ʾ�ĺ���
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//                mode:  0�ǵ���ģʽ  1����ģʽ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
//{
//	u8 i,j,m=0;
//	u16 k;
//	u16 HZnum;//������Ŀ
//	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
//	u16 x0=x;
//	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
//	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//ͳ�ƺ�����Ŀ
//	for(k=0;k<HZnum;k++) 
//	{
//		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
//		{ 	
//			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
//			for(i=0;i<TypefaceNum;i++)
//			{
//				for(j=0;j<8;j++)
//				{	
//					if(!mode)//�ǵ��ӷ�ʽ
//					{
//						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
//						else LCD_WR_DATA(bc);
//						m++;
//						if(m%sizey==0)
//						{
//							m=0;
//							break;
//						}
//					}
//					else//���ӷ�ʽ
//					{
//						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
//						x++;
//						if((x-x0)==sizey)
//						{
//							x=x0;
//							y++;
//							break;
//						}
//					}
//				}
//			}
//		}				  	
//		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
//	}
//}


///******************************************************************************
//      ����˵������ʾ�����ַ�
//      ������ݣ�x,y��ʾ����
//                num Ҫ��ʾ���ַ�
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//                mode:  0�ǵ���ģʽ  1����ģʽ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
//{
//	u8 temp,sizex,t,m=0;
//	u16 i,TypefaceNum;//һ���ַ���ռ�ֽڴ�С
//	u16 x0=x;
//	sizex=sizey/2;
//	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
//	num=num-' ';    //�õ�ƫ�ƺ��ֵ
//	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //���ù��λ�� 
//	for(i=0;i<TypefaceNum;i++)
//	{ 
//		if(sizey==12)temp=ascii_1206[num][i];		       //����6x12����
//		else if(sizey==16)temp=ascii_1608[num][i];		 //����8x16����
//		else if(sizey==24)temp=ascii_2412[num][i];		 //����12x24����
//		else if(sizey==32)temp=ascii_3216[num][i];		 //����16x32����
//		else return;
//		for(t=0;t<8;t++)
//		{
//			if(!mode)//�ǵ���ģʽ
//			{
//				if(temp&(0x01<<t))LCD_WR_DATA(fc);
//				else LCD_WR_DATA(bc);
//				m++;
//				if(m%sizex==0)
//				{
//					m=0;
//					break;
//				}
//			}
//			else//����ģʽ
//			{
//				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//��һ����
//				x++;
//				if((x-x0)==sizex)
//				{
//					x=x0;
//					y++;
//					break;
//				}
//			}
//		}
//	}   	 	  
//}


///******************************************************************************
//      ����˵������ʾ�ַ���
//      ������ݣ�x,y��ʾ����
//                *p Ҫ��ʾ���ַ���
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//                mode:  0�ǵ���ģʽ  1����ģʽ
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
//{         
//	while(*p!='\0')
//	{       
//		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
//		x+=sizey/2;
//		p++;
//	}  
//}


///******************************************************************************
//      ����˵������ʾ����
//      ������ݣ�m������nָ��
//      ����ֵ��  ��
//******************************************************************************/
//u32 mypow(u8 m,u8 n)
//{
//	u32 result=1;	 
//	while(n--)result*=m;
//	return result;
//}


///******************************************************************************
//      ����˵������ʾ��������
//      ������ݣ�x,y��ʾ����
//                num Ҫ��ʾ��������
//                len Ҫ��ʾ��λ��
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
//{         	
//	u8 t,temp;
//	u8 enshow=0;
//	u8 sizex=sizey/2;
//	for(t=0;t<len;t++)
//	{
//		temp=(num/mypow(10,len-t-1))%10;
//		if(enshow==0&&t<(len-1))
//		{
//			if(temp==0)
//			{
//				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
//				continue;
//			}else enshow=1; 
//		 	 
//		}
//	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
//	}
//} 


///******************************************************************************
//      ����˵������ʾ��λС������
//      ������ݣ�x,y��ʾ����
//                num Ҫ��ʾС������
//                len Ҫ��ʾ��λ��
//                fc �ֵ���ɫ
//                bc �ֵı���ɫ
//                sizey �ֺ�
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
//{         	
//	u8 t,temp,sizex;
//	u16 num1;
//	sizex=sizey/2;
//	num1=num*100;
//	for(t=0;t<len;t++)
//	{
//		temp=(num1/mypow(10,len-t-1))%10;
//		if(t==(len-2))
//		{
//			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
//			t++;
//			len+=1;
//		}
//	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
//	}
//}


///******************************************************************************
//      ����˵������ʾͼƬ
//      ������ݣ�x,y�������
//                length ͼƬ����
//                width  ͼƬ���
//                pic[]  ͼƬ����    
//      ����ֵ��  ��
//******************************************************************************/
//void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
//{
//	u16 i,j;
//	u32 k=0;
//	LCD_Address_Set(x,y,x+length-1,y+width-1);
//	for(i=0;i<length;i++)
//	{
//		for(j=0;j<width;j++)
//		{
//			LCD_WR_DATA8(pic[k*2]);
//			LCD_WR_DATA8(pic[k*2+1]);
//			k++;
//		}
//	}			
//}





//void ShowString(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode) {
//    while (*p != '\0') {
//        LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
//        x += sizey / 2;
//        p++;
//    }
//}

///********************************************************************************
//    @FunctionName:    ShowText
//    @Description:     ��ʾ�ַ���
//    @FunctionAuthor:  ì�ܾۺ���
//    @parameters:
//                    NAME      TYPE       DESCRIBE
//                    @x:       uint16_t   ��ʾ����
//                    @y:       uint16_t   ��ʾ����
//                    @fc:      uint16_t   �ֵ���ɫ
//                    @bc:      uint16_t   �ֵı���ɫ
//                    @sizey:   uint8_t    �ֺ�
//                    @mode:    uint8_t    0�ǵ���ģʽ  1����ģʽ
//                    @str:     uint8_t*   Ҫ��ʾ���ַ���
//    @Return:          void
//    @Other:
//********************************************************************************/
//void ShowText(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode, const char *str, ...) {
//    char strFormatted[256];
//    va_list args;         //��ſɱ���������ݽṹ
//    va_start(args, str);  //��ʼ���ɱ����,��Ҫ��һ��va_list���ͱ���,�Ϳɱ����֮ǰ�Ĳ���,������str
//    vsprintf(strFormatted, str, args);
//    ShowString(x, y, (const uint8_t *)strFormatted, fc, bc, sizey, mode);  //�˺�����ͷ�ļ� stdio��
//    va_end(args);
//}






//void DrawTab(uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, uint16_t row, uint16_t col, uint16_t color) {
//    int i, j;
//    for (i = 0; i <= row; i++) {
//        LCD_DrawLine(
//            x0,
//            y0 + height * i / row,
//            x0 + width,
//            y0 + height * i / row,
//            color);
//    }
//    for (j = 0; j <= col; j++) {
//        LCD_DrawLine(
//            x0 + width * j / col,
//            y0,
//            x0 + width * j / col,
//            y0 + height,
//            color);
//    }
//}


//const char tab_name[7][2][32] =
//    {

//        {"By:Mr.Pan", "Distance"},      // odrv0.axis0.fet_thermistor.temperature		
//        {"Dcbus", "Height"},        // odrv0.axis0.controller.vel_setpoint, odrv0.axis0.encoder.vel_estimate
//        {"Pitch", "Roll"},  // odrv0.axis0.controller.torque_setpoint
//        {"Yaw", "Temp"},        // odrv0.axis0.encoder.error  odrv0.axis1.encoder.shadow_count
//        {"  Robot_Mode ", "  "},      // odrv0.axis0.motor.gate_driver.drv_fault
//        {"  Task", " "},        // odrv0.axis0.motor.current_control.Ibus
//        {"  X_point", "  Y_point"},  // odrv0.axis0.motor_thermistor.temperature
//    };

//const char tab_val[8][2][32] =
//    {
//        {"", ""},
//        {"", ""},
//        {"", ""},
//        {"", ""},
//        {"", ""},
//        {"", ""},
//        {"", ""},
//        {"", ""},
//    };

//int i=0;
//void lcdGlobalInit(void){

//	LCD_Init();//LCD��ʼ�� 
//	LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

//	ShowText(LCD_W * 2 / 4 -1, (LCD_H - 1) * 0 / NUM_ROW + 2, WHITE, BLACK, FontSize_12, 0, "Vbus:");	
//    ShowText(LCD_W * 2 / 4 +70, (LCD_H - 1) * 0 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "V");

//	ShowText(LCD_W * 2 / 4 -1, (LCD_H - 1) * 0 / NUM_ROW + 15, WHITE, BLACK, FontSize_12, 0, "Temp:");		
//    ShowText(LCD_W * 2 / 4 +70, (LCD_H - 1) *0 / NUM_ROW + 15, WHITE, BLACK, FontSize_12, 0,"C");
//	
//	
//	
//    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *0 / NUM_ROW + 2, WHITE, BLACK, FontSize_12, 0,"Con:");
//    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *0 / NUM_ROW + 15, WHITE, BLACK, FontSize_12, 0,"Mode:");


//	for(i=0;i<4;i++){ShowText(LCD_W * 2 / 4 + 6, (LCD_H - 1) *(i+2.5) / NUM_ROW + 2, YELLOW, BLACK, FontSize_12, 0, "->");}
//	

//		
//    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *2.5 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "Cur");
//	

//    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *3.5 / NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "Pos");
//		
//		
//    ShowText(LCD_W * 0 / 4 + 2, (LCD_H - 1) *4.5/ NUM_ROW + 2, WHITE, BLACK, FontSize_16, 0, "Vel");

//    //ShowText(LCD_W * 0 / 4 + 55, (LCD_H - 1) *5.5/ NUM_ROW + 2, GREEN, BLACK, FontSize_12, 0, "No_Error!");


//}




//void MenuUpdate(void)
//{
//	
//    //��ʾ��ǰ��ѹֵ
//	LCD_ShowFloatNum1(LCD_W * 2 / 4 +30, (LCD_H - 1) * 0 / NUM_ROW + 2,OdReceivedData.vbus_voltage.float_temp,4,YELLOW, BLACK,FontSize_16);

//    //��ʾ��ǰ�¶�
//	LCD_ShowFloatNum1(LCD_W * 2 / 4 +30, (LCD_H - 1) * 0 / NUM_ROW + 15,OdReceivedData.temperature[0].float_temp,4,BROWN, BLACK,FontSize_16);	

////    //��ʾ��ǰʹ�ñ���������
////    ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 2, MAGENTA, BLACK, FontSize_12, 0,"OUT_SPI");

//	//��ʾ��ǰʹ�õĿ���ģʽ
//	switch(OdriveData.ControlMode[0]){
//		
//		case CONTROL_MODE_POSITION_TRAP:
// 
//			ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 15, GREEN, BLACK, FontSize_12, 0, "Trap_Pos");
//            break;  
//		
//		case CONTROL_MODE_VELOCITY_RAMP:
// 
//			ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 15, GREEN, BLACK, FontSize_12, 0, "Ramp_Vel");
//            break;  
//		
//		case CONTROL_MODE_CURRENT_RAMP:
// 
//			ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 15, GREEN, BLACK, FontSize_12, 0, "Ramp_Cur");
//            break;
// 		
////		case CONTROL_MODE_CURRENT:
//// 
////			ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 15, GREEN, BLACK, FontSize_12, 0, "Current");
////            break; 

////		case CONTROL_MODE_POSITION:
//// 
////			ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 15, GREEN, BLACK, FontSize_12, 0, "Position");
////            break; 		

////		case CONTROL_MODE_VELOCITY:
//// 
////			ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 15, GREEN, BLACK, FontSize_12, 0, "Velocity");
////            break; 		
//		default:break;
//		} 	

//		
//    //��ʾ���õ���ֵ
//	//LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 27,OdriveData.SetCur[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//    //��ʾ����λ��ֵ
//	LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 40,OdriveData.SetPos[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//    //��ʾ�����ٶ�ֵ
//	LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 53,OdriveData.SetVel[0].float_temp,5,BROWN, BLACK,FontSize_12);	

//    //��ʾ���õ���ֵ
//	if(OdriveData.SetCur[0].float_temp>0.0f)	{
//		
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 27,OdriveData.SetCur[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 2 / 4 + 22, (LCD_H - 1) *(0+2.5) / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "+");		
//		
//	}
//	else{
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 27,-OdriveData.SetCur[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 2 / 4 + 22, (LCD_H - 1) *(0+2.5) / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "-");		
//		
//	}		
//		
//    //��ʾ����λ��ֵ
//	if(OdriveData.SetPos[0].float_temp>0.0f)	{
//		
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 40,OdriveData.SetPos[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 2 / 4 + 22, (LCD_H - 1) *(1+2.5) / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "+");		
//		
//	}
//	else{
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 40,-OdriveData.SetPos[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 2 / 4 + 22, (LCD_H - 1) *(1+2.5) / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "-");		
//		
//	}	

//    //��ʾ�����ٶ�ֵ
//	if(OdriveData.SetVel[0].float_temp>0.0f)	{
//		
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 53,OdriveData.SetVel[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 2 / 4 + 22, (LCD_H - 1) *(2+2.5) / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "+");		
//		
//	}
//	else{
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +35, (LCD_H - 1) * 0 / NUM_ROW + 53,-OdriveData.SetVel[0].float_temp,5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 2 / 4 + 22, (LCD_H - 1) *(2+2.5) / NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "-");		
//		
//	}

//	
//		
//    //��ʾ��ǰ����ֵ
//	if(OdReceivedData.heartbeat_Cur[0]>0.0f)	{
//		
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +-38, (LCD_H - 1) * 0 / NUM_ROW + 27,OdReceivedData.heartbeat_Cur[0],5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 0 / 4 + 27, (LCD_H - 1) *2.5/ NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "+");		
//		
//	}
//	else{
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +-38, (LCD_H - 1) * 0 / NUM_ROW + 27,-OdReceivedData.heartbeat_Cur[0],5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 0 / 4 + 27, (LCD_H - 1) *2.5/ NUM_ROW + 2,BROWN, BLACK, FontSize_16, 0, "-");		
//		
//	}

//		

//    //��ʾ��ǰλ��ֵ
//	if(OdReceivedData.heartbeat_Pos[0]>0.0f)	{
//		
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +-38, (LCD_H - 1) * 0 / NUM_ROW + 40,OdReceivedData.heartbeat_Pos[0],5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 0 / 4 + 27, (LCD_H - 1) *3.5/ NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "+");		
//		
//	}
//	else{
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +-38, (LCD_H - 1) * 0 / NUM_ROW + 40,-OdReceivedData.heartbeat_Pos[0],5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 0 / 4 + 27, (LCD_H - 1) *3.5/ NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "-");		
//		
//	}


//		
//    //��ʾ��ǰ�ٶ�ֵ
//	if(OdReceivedData.heartbeat_Vel[0]>0.0f)	{
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +-38, (LCD_H - 1) * 0 / NUM_ROW + 53,OdReceivedData.heartbeat_Vel[0],5,BROWN, BLACK,FontSize_12);	
//		ShowText(LCD_W * 0 / 4 + 27, (LCD_H - 1) *4.5/ NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "+");		
//		
//	}
//	else{
//		LCD_ShowFloatNum1(LCD_W * 2 / 4 +-38, (LCD_H - 1) * 0 / NUM_ROW + 53,-OdReceivedData.heartbeat_Vel[0],5,BROWN, BLACK,FontSize_12);		
//		ShowText(LCD_W * 0 / 4 + 27, (LCD_H - 1) *4.5/ NUM_ROW + 2, BROWN, BLACK, FontSize_16, 0, "-");				
//	}
//		


//	//��ʾ��ǰ�Ĵ�����Ϣ
////	switch(OdReceivedData.heartbeat_AxisError[0]){
////		case 0:
////			
////				LCD_ShowString(LCD_W * 0 / 4 + 2, (LCD_H - 1) *5.5/ NUM_ROW + 2,"    No_Error!             ",RED,BLACK,FontSize_12,0);
//// 
////            break;  
////		
////		case 10:
////			
////				LCD_ShowString(LCD_W * 0 / 4 + 2, (LCD_H - 1) *5.5/ NUM_ROW + 2,"ERR_OVER_TEMP!",RED,BLACK,FontSize_12,0);
//// 
////            break;  
////		
////		case 2:
////			
////				LCD_ShowString(LCD_W * 0 / 4 + 2, (LCD_H - 1) *5.5/ NUM_ROW + 2,"ERR_UNDER_VOLTAGE!",RED,BLACK,FontSize_12,0);
////		
////            break;  
////		
////		case 1:
////			
////				LCD_ShowString(LCD_W * 0 / 4 + 2, (LCD_H - 1) *5.5/ NUM_ROW + 2,"ERR_OVER_VOLTAGE!",RED,BLACK,FontSize_12,0);		
//// 
////            break;
////		case 4:
////			
////				LCD_ShowString(LCD_W * 0 / 4 + 2, (LCD_H - 1) *5.5/ NUM_ROW + 2,"ERR_OVER_SPEED!",RED,BLACK,FontSize_12,0);
//// 
////            break; 	
////			
////		
////		default:break;
////		} 	
//	
//	//��ʾ��ǰ�Ĵ�����Ϣ
//	switch(OdriveData.AxisState[axis0]){
//		case CMD_MENU:
//			
//				ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 2, MAGENTA, BLACK, FontSize_12, 0,"Relax");
// 
//            break;  
//		case CMD_MOTOR:
//			
//				ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 2, MAGENTA, BLACK, FontSize_12, 0,"Enable");
// 
//            break;  		
//		case CMD_CALIBRATION:
//			
//				ShowText(LCD_W * 0 / 4 + 30, (LCD_H - 1) *0 / NUM_ROW + 2, MAGENTA, BLACK, FontSize_12, 0,"Cali");
// 
//            break;	
//		default:break;
//		} 	
//	
//}



//void lcdUpdateTask(void *Parameters){
//	TickType_t xLastWakeTime = xTaskGetTickCount();
//	digitalLo(&lcdData.dataInitFlag);
//	while(true){
//		vTaskDelayUntil(&xLastWakeTime,LCD_NORMAL_PERIOD);
//        //��ֹ�ظ���ʼ��
//		if(!lcdData.dataInitFlag){	
//            //���п���ȫ����ʼ��            
//			lcdGlobalInit();																																							
//			digitalHi(&lcdData.dataInitFlag);
//            
//		}  
//		
//		MenuUpdate();
//		
//		digitalHi(&OdriveData.readpairsFlag);         

//		digitalIncreasing(&lcdData.loops);        
//		
//		
//	}
//	
//	
//	
//}

//void LcdTaskInit(void){
//	/* uxPriority
//	�ڵ������񴴽�����xTaskCreate()ʱ��Ϊ����ָ�������ȼ�����������������Ⱥ���֮��
//	����ͨ������xTaskPrioritySet()�������޸���������ȼ���ϵͳ��������û�ж����ȼ���������Ҫ��
//	ֻҪ�ڴ��㹻��Ϳ��Դ�������������ò�ͬ�����ȼ�����ͬ��������Ը�����ͬ�����ȼ���
//	��FreeRTOSConfig.h��configMAX_PRIORITIES �Ĵ�С����������ȼ�������
//	���ȼ��Ǵ�0��ʼ��configMAX_PRIORITIES - 1Ϊֹ�����ȼ�Ϊ0����������ȼ���ߡ�
//     ϵͳ�еĵ����������������ȼ���ߵ����������У�����������ӵ����ͬ�����ȼ���
//	��ô����������ʹ����������ִ��һ��ʱ��Ƭ�������ʱ��Ƭ����1/����ʱ��Ƶ�ʣ�����ʱ�Ӽ��δ�ʱ�ӡ�
//	����ͨ������FreeRTOSConfig.h�е�configTICK_RATE_HZ�����������ж�ʱ�ӣ�
//	��configTICK_RATE_HZ��ֵΪ100HZʱ��ÿ100ģʽ����һ������ʱ���жϣ�ϵͳ��ִ����Ӧ���жϺ�����	
//	*/
//				
//	getsupervisorData()->taskEvent[LCD_TASK] = xTaskCreate(lcdUpdateTask,"LCD",LCD_STACK_SIZE,NULL,LCD_PRIORITY,&lcdData.xHandleTask);
//    
//}


