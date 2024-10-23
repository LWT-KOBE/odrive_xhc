#include "application.h"
#include "usartx.h"
#include "board.h"
#include "balance.h"
#include "bsp_exit.h"
balanceStruct_t balanceData;
Encoding_Wheel EW;
balance_NFC_t NFC;
balance_target_t Angle_Goal;
balance_target_t Motor_SpeedA_Goal;
balance_target_t Motor_SpeedB_Goal;

//uint8_t temp[8]={0};

CascadePID mypid = {0}; //创建串级PID结构体变量

balanceStruct_t* getbalanceData(){
    return &balanceData;
}

//float pbuf[10];
void balanceGlobalInit(void){
//	uart1_init(115200);
//	uart2_init(9600);
//	uart5_init(115200);
	uart3_init(115200);
	cigan_Init();
	BSP_GPIO_EXIT_Init(BSP_GPIOA6,EXTI_Trigger_Rising,5,0);
	Encoder_Init_TIM3();
//	PID_Init(&mypid.inner, 0.125f, 0.0000f, 14.0f, 100, 2.0f); //初始化内环参数
//    //PID_Init(&mypid.outer, 3.0f, 0.0f, 0.0f, 0, 200.0f); //初始化外环参数
//	PID_Init(&mypid.outer, 7.0f, 0.0f, 15.0f, 0, 1000.0f); //初始化外环参数
}

void balanceUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	digitalLo(&getbalanceData()->dataInitFlag);
	while(true){
		vTaskDelayUntil(&xLastWakeTime,CONTROL_NORMAL_PERIOD);
        //防止重复初始化
		if(!balanceData.dataInitFlag){	
            //所有控制全部初始化            
			balanceGlobalInit();
			digitalHi(&getbalanceData()->dataInitFlag);
			PDout(2) = 1;
			PBout(3) = 0;
		}
	 	
		
		
		//获取编码器计数
		EW.Encoder_pr = Read_Encoder(3);
		//累加
		EW.mileage -= EW.Encoder_pr;
		//计算里程
		EW.Current_Mileage = EW.mileage / 4000.0f * 20.0f;
		//计算速度
		EW.speed = EW.Encoder_pr /4000.0f * 20.0f * 2.5f;
		
		//正常发速度指令控制
		if(balanceData.flag == 0){
			Motor_SpeedA_Goal.target = Incremental_PID(OdReceivedData.vel_estimate[1].float_temp,Motor_SpeedB_Goal.target);
			//分段控制速度 因为odrive端只有Kp,和设定的目标值有静态差距，故在发送时增大发送值来达到真正的期望值
			
			if(Motor_SpeedB_Goal.target == 0){
				Motor_SpeedA_Goal.target = 0;
			}
			
			OdriveData.SetVel[0].float_temp = -Motor_SpeedA_Goal.target / (M_PI * 0.07f) * 1.00f;
			OdriveData.SetVel[1].float_temp = Motor_SpeedA_Goal.target / (M_PI * 0.07f) * 1.00f;
			
			
			Angle_Goal.target = OdReceivedData.vel_estimate[0].float_temp;
			//EW.mileage = 0;
			Angle_Goal.finish = 0;
			

		}
		
		
		
		if(balanceData.flag == 1){
			if(Angle_Goal.finish == 0){
				EW.mileage = 0;
				Angle_Goal.finish = 1;
			}
		
			//单级PID控制
			OdriveData.SetVel[0].float_temp =  -Position_PID_N(EW.Current_Mileage,Motor_SpeedB_Goal.target) / (M_PI * 0.07f) *3.0f;
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
			
			OdriveData.SetVel[0].float_temp =  Position_PID_N(OdReceivedData.pos_estimate[0].float_temp,Motor_SpeedB_Goal.target) / (M_PI * 0.07f) *3.0f;
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
			
			//串级PID控制
			PID_CascadeCalc(&mypid, Motor_SpeedB_Goal.target, EW.Current_Mileage, (OdReceivedData.vel_estimate[0].float_temp - OdReceivedData.vel_estimate[1].float_temp)/2); //进行PID计算
			if(fabs(Angle_Goal.target) > 0.5 && fabs(Motor_SpeedA_Goal.target) > 0){
				OdriveData.SetVel[0].float_temp = -mypid.output *3.0f;
			}else if(fabs(Angle_Goal.target) < 0.5){
				OdriveData.SetVel[0].float_temp = -mypid.output *5.0f;
			}
			if(Motor_SpeedB_Goal.finish == 1){
				Angle_Goal.target = 0;
				Motor_SpeedA_Goal.target = 0;
			}
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
			
//			PID_CascadeCalc(&mypid, Motor_SpeedB_Goal.target, OdReceivedData.pos_estimate[1].float_temp, OdReceivedData.vel_estimate[1].float_temp); //进行PID计算
//			OdriveData.SetVel[0].float_temp = mypid.output *5.0f ;
////			OdriveData.SetVel[0].float_temp = mypid.output *1.0f ;
//			OdriveData.SetVel[1].float_temp =  OdriveData.SetVel[0].float_temp;
//			NVIC_SystemReset();
		}
		
		
		//测试模式前进
		if(balanceData.flag == 2){
			
			if(Motor_SpeedB_Goal.finish == 1){
				
				if(Motor_SpeedB_Goal.target >= 700.0){
					Motor_SpeedB_Goal.target = 700;
					balanceData.flag = 3;
				}
				
				else if(Motor_SpeedB_Goal.target < 700.0 && Motor_SpeedB_Goal.target >=0){
					Motor_SpeedB_Goal.target += 70;
				}
				
			}
			
			if(Angle_Goal.finish == 0){
				EW.mileage = 0;
				Angle_Goal.finish = 1;
			}

			
			//串级PID控制
			PID_CascadeCalc(&mypid, Motor_SpeedB_Goal.target, EW.Current_Mileage, (OdReceivedData.vel_estimate[0].float_temp - OdReceivedData.vel_estimate[1].float_temp)/2); //进行PID计算
			
			//位置速度控制
			if(fabs(Angle_Goal.target) > 0.5 && fabs(Motor_SpeedA_Goal.target) > 0){
				OdriveData.SetVel[0].float_temp = -mypid.output *3.0f;
			}else if(fabs(Angle_Goal.target) < 0.5){
				OdriveData.SetVel[0].float_temp = -mypid.output *5.0f;
			}
			//判断是否达到目标位置
			if(Motor_SpeedB_Goal.finish == 1){
				Angle_Goal.target = 0;
				Motor_SpeedA_Goal.target = 0;
			}
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
			
		}
		
		//测试模式后退
		if(balanceData.flag == 3){
			
			if(Motor_SpeedB_Goal.finish == 1){
				
				if(Motor_SpeedB_Goal.target <= 0.0f){
					Motor_SpeedB_Goal.target = 0;
					balanceData.flag = 2;
				}
				
				else if(Motor_SpeedB_Goal.target > 0.0f){
					Motor_SpeedB_Goal.target -= 70;
				}
				
			}
			
			if(Angle_Goal.finish == 0){
				EW.mileage = 0;
				Angle_Goal.finish = 1;
			}

			
			//串级PID控制
			PID_CascadeCalc(&mypid, Motor_SpeedB_Goal.target, EW.Current_Mileage, (OdReceivedData.vel_estimate[0].float_temp - OdReceivedData.vel_estimate[1].float_temp)/2); //进行PID计算
			if(fabs(Angle_Goal.target) > 0.5 && fabs(Motor_SpeedA_Goal.target) > 0){
				OdriveData.SetVel[0].float_temp = -mypid.output *3.0f;
			}else if(fabs(Angle_Goal.target) < 0.5){
				OdriveData.SetVel[0].float_temp = -mypid.output *5.0f;
			}
			if(Motor_SpeedB_Goal.finish == 1){
				Angle_Goal.target = 0;
				Motor_SpeedA_Goal.target = 0;
			}
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
		}
		
		//测试模式前进1
		if(balanceData.flag == 4){
			
			if(Motor_SpeedB_Goal.finish == 1){
				
				if(Motor_SpeedB_Goal.target >= 5000.0){
					Motor_SpeedB_Goal.target = 5000;
					balanceData.flag = 5;
				}
				
				else if(Motor_SpeedB_Goal.target < 5000.0 && Motor_SpeedB_Goal.target >=0){
					Motor_SpeedB_Goal.target += 500;
				}
				
			}
			
			if(Angle_Goal.finish == 0){
				EW.mileage = 0;
				Angle_Goal.finish = 1;
			}

			
			//串级PID控制
			PID_CascadeCalc(&mypid, Motor_SpeedB_Goal.target, EW.Current_Mileage, (OdReceivedData.vel_estimate[0].float_temp - OdReceivedData.vel_estimate[1].float_temp)/2); //进行PID计算
			
			//位置速度控制
			if(fabs(Angle_Goal.target) > 0.5 && fabs(Motor_SpeedA_Goal.target) > 0){
				OdriveData.SetVel[0].float_temp = -mypid.output *1.0f;
			}else if(fabs(Angle_Goal.target) < 0.5){
				OdriveData.SetVel[0].float_temp = -mypid.output *2.5f;
			}
			//判断是否达到目标位置
			if(Motor_SpeedB_Goal.finish == 1){
				Angle_Goal.target = 0;
				Motor_SpeedA_Goal.target = 0;
			}
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
			
		}
		
		//测试模式后退
		if(balanceData.flag == 5){
			
			if(Motor_SpeedB_Goal.finish == 1){
				
				if(Motor_SpeedB_Goal.target <= 0.0f){
					Motor_SpeedB_Goal.target = 0;
					balanceData.flag = 4;
				}
				
				else if(Motor_SpeedB_Goal.target > 0.0f){
					Motor_SpeedB_Goal.target -= 500;
				}
				
			}
			
			if(Angle_Goal.finish == 0){
				EW.mileage = 0;
				Angle_Goal.finish = 1;
			}

			
			//串级PID控制
			PID_CascadeCalc(&mypid, Motor_SpeedB_Goal.target, EW.Current_Mileage, (OdReceivedData.vel_estimate[0].float_temp - OdReceivedData.vel_estimate[1].float_temp)/2); //进行PID计算
			if(fabs(Angle_Goal.target) > 0.5 && fabs(Motor_SpeedA_Goal.target) > 0){
				OdriveData.SetVel[0].float_temp = -mypid.output *1.0f;
			}else if(fabs(Angle_Goal.target) < 0.5){
				OdriveData.SetVel[0].float_temp = -mypid.output *2.5f;
			}
			if(Motor_SpeedB_Goal.finish == 1){
				Angle_Goal.target = 0;
				Motor_SpeedA_Goal.target = 0;
			}
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
		}
		
		//测试模式后退
		if(balanceData.flag == 6){
			
			if(Motor_SpeedB_Goal.finish == 1){
				
				if(Motor_SpeedB_Goal.target <= 0.0f){
					Motor_SpeedB_Goal.target = 0;
					balanceData.flag = 4;
				}
				
				else if(Motor_SpeedB_Goal.target > 0.0f){
					Motor_SpeedB_Goal.target -= 200;
				}
				
			}
			
			if(Angle_Goal.finish == 0){
				EW.mileage = 0;
				Angle_Goal.finish = 1;
			}

			
			//串级PID控制
			PID_CascadeCalc(&mypid, Motor_SpeedB_Goal.target, EW.Current_Mileage, (OdReceivedData.vel_estimate[0].float_temp - OdReceivedData.vel_estimate[1].float_temp)/2); //进行PID计算
			if(fabs(Angle_Goal.target) > 0.5 && fabs(Motor_SpeedA_Goal.target) > 0){
				OdriveData.SetVel[0].float_temp = -mypid.output *1.0f;
			}else if(fabs(Angle_Goal.target) < 0.5){
				OdriveData.SetVel[0].float_temp = -mypid.output *2.5f;
			}
			if(Motor_SpeedB_Goal.finish == 1){
				Angle_Goal.target = 0;
				Motor_SpeedA_Goal.target = 0;
			}
			OdriveData.SetVel[1].float_temp =  -OdriveData.SetVel[0].float_temp;
		}
		
		digitalIncreasing(&getbalanceData()->loops);        
		
	}
}



void balanceInit(void){
	/* uxPriority
	在调用任务创建函数xTaskCreate()时就为任务指定了优先级，在启动了任务调度函数之后，
	可以通过调用xTaskPrioritySet()函数来修改任务的优先级。系统中理论上没有对优先级做出上限要求，
	只要内存足够大就可以创建多个任务，设置不同的优先级，不同的任务可以赋予相同的优先级，
	在FreeRTOSConfig.h中configMAX_PRIORITIES 的大小决定最大优先级个数，
	优先级是从0开始到configMAX_PRIORITIES - 1为止，优先级为0的任务的优先级最高。
     系统中的调度器总是先让优先级最高的任务先运行，如果多个任务拥有相同的优先级，
	那么调度器将会使得任务轮流执行一个时间片。这里的时间片等于1/心跳时钟频率，心跳时钟即滴答时钟。
	可以通过设置FreeRTOSConfig.h中的configTICK_RATE_HZ来设置心跳中断时钟，
	当configTICK_RATE_HZ赋值为100HZ时，每100模式发生一次心跳时钟中断，系统会执行相应的中断函数。	
	*/
	
	getsupervisorData()->taskEvent[CONTROL_TASK] = xTaskCreate(balanceUpdateTask,"BALANCE",BALANCE_STACK_SIZE,NULL,BALANCE_PRIORITY,&balanceData.xHandleTask);
    //usbVCP_Printf("ControlInit Successfully \r\n");
    
}
