#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "JY901.h"
#include "math.h"
#include "fan.h"
#include "timer.h"
#include "pwm.h"
#include "indkey.h"
#include "Kalman_filter.h"
#include "ospid.h"
#include "UART2.h"
#include "BSP_NVIC.h"
#include "mympu6050.h"
#include "iic_analog.h"

//#define JY61
//#define atkimu
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
extern PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID;
extern float A_P,A_R,A2_P;
extern	float imua[3],imuw[3],imuangle[3];
extern int ctrlx,ctrly; 
extern float rollset,pitchset;
extern  MPU6050_AxisTypeDef    Axis;
//used  pins: A:9,10(USART1)  B:6,7,(TIM4OC)8,9(IIC) 10,11(TIM2OC)  C:6,7,8,9(TIM3OC)   TIM6   D:2345(key)
float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据


int main(void)
 { 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组
	delay_init(168);  //初始化延时函数
	uart_init(500000);		//初始化串口波特率为500000
	uart6_init(500000);		//初始化串口波特率为500000

	TIM3_PWM_Init(83,2500);	
	TIM6_Int_Init(83,10000);
	USART3_Configuration();
	 	 

	Key_IO_Init();
BSP_NVIC_InitConfig();
#ifdef JY61
	 Initial_UART2(115200);
#else
	 
	while(MPU_Init())
{

}
	while(mpu_dmp_init())
	{
	}
#endif

	

 	while(1)
	{
#ifdef JY61	
	pitch=imuangle[1];
	roll =imuangle[0];
	yaw=imuangle[2];
	aacx=imua[0];
	aacy=imua[1];
	aacz=imua[2];
	gyrox=imuw[0];
	gyrox=imuw[1];
	gyrox=imuw[2];
#else
	 		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
//			mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
//			usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			if(pitch)
			{
			Angle_Calcu();
			}
//	usart1_report_imu(pitch,pitchset,ctrlx/25,ctrly/25,PitchOPID.Pout,PitchOPID.Dout,RollOPID.Pout,RollOPID.Dout,(int)(yaw*10));
		}
#endif

	} 	
}
