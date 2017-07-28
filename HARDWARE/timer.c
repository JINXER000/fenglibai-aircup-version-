#include "timer.h"
#include "fan.h"
#include "mpu6050.h"
#include "Kalman_filter.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ospid.h"
#include "niming.h"
#include "mympu6050.h"
#include "math.h"


 int i,dmpresetCounter;
//#define boximu
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
extern int ctrlx,ctrly; 
extern	float imua[3],imuw[3],imuangle[3];
extern float rollset,pitchset;
extern PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID;
extern MPU6050_AxisTypeDef    Axis;  //MPU6050数据结构体
extern float A_P,A_R,A2_P;


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
			
#ifdef boximu
	uint8_t i = 0;
	float pitch_temp1 = 0.0;
	float roll_temp1 = 0.0;
	float pitch_temp2 = 0.0;
	float roll_temp2 = 0.0;
	static float pitch_sum = 0.0;
	static float roll_sum = 0.0;
		
		for(i=0;i<3;i++)
		{
			Angle_Calculate();		
			
			pitch_temp1 = (atan(Axis.AccX/Axis.AccZ)*57.2958-0.4);   //计算Pitch角度 0.4为静态偏差角
			roll_temp1  = (atan(Axis.AccY/Axis.AccZ)*57.2958-0.3);   //计算Roll角度  0.3为静态偏差角
			
			pitch_sum += pitch_temp1;
			roll_sum  += roll_temp1;
		}
		
		pitch_temp1 = pitch_sum / 3.0;	 //取出平均值
		roll_temp1  = roll_sum  / 3.0;	 //取出平均值

		pitch_sum = 0.0;
		roll_sum = 0.0;
		
#endif		
	
	controltask();
	usart1_report_imu(PitchOPID.CurrentError*10,PitchOPID.PIDout/25,RollOPID.CurrentError*10,RollOPID.PIDout/25,PitchOPID.Pout/25,PitchOPID.Dout/25,RollOPID.Pout/25,RollOPID.Dout/25,(int)(yaw*10));
			
    }
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);  //清除中断标志位

}



