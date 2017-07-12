#include "timer.h"
#include "fan.h"
#include "mpu6050.h"
#include "Kalman_filter.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ospid.h"
 int i;

extern float Angle,Gyro_x;         //小车滤波后倾斜角度/角速度	
extern float Angle_x_temp;  //由加速度计算的x倾斜角度
extern float Angle_y_temp;  //由加速度计算的y倾斜角度
extern float Angle_z_temp;
extern float Angle_X_Final; //X最终倾斜角度
extern float Angle_Y_Final; //Y最终倾斜角度
extern float Angle_Z_Final; //Z最终倾斜角度
extern float Gyro_x;		 //X轴陀螺仪数据暂存
extern float Gyro_y;        //Y轴陀螺仪数据暂存
extern float Gyro_z;		 //Z轴陀螺仪数据暂存
extern 	short aacx,aacy,aacz;		//加速度传感器原始数据
extern	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern float pitch,roll,yaw; 		//欧拉角
extern void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);


void TIM6_Int_Init(int psc,int prd)  //arr=500, psc=840  
{
	
	    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = psc;        //84M internal clock =83
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = prd;  //5ms,200Hz   
    TIM_TimeBaseInit(TIM6,&tim);
		
		TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	 TIM_Cmd(TIM6,ENABLE); //使能定时器3

}

void TIM6_DAC_IRQHandler(void)  
{
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
			
//get angle ,calc and filter
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			if(pitch)
			{
			Angle_Calcu();
			}
//					i++;
//					if(i>300){
//					fanmove(2000,0);
//						i=0;
//					}
			usart1_report_imu(Angle_x_temp,Angle_X_Final,Angle_y_temp,Angle_Y_Final,Angle_z_temp,Angle_Z_Final,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));

			
		}
		
  
		
//		controltask();
			
			
    }
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);  //清除中断标志位

}



