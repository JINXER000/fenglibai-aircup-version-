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
extern float Angle,Gyro_x;         //С���˲�����б�Ƕ�/���ٶ�	
extern float Angle_x_temp;  //�ɼ��ٶȼ����x��б�Ƕ�
extern float Angle_y_temp;  //�ɼ��ٶȼ����y��б�Ƕ�
extern float Angle_z_temp;
extern float Angle_X_Final; //X������б�Ƕ�
extern float Angle_Y_Final; //Y������б�Ƕ�
extern float Angle_Z_Final; //Z������б�Ƕ�
extern float Gyro_x;		 //X�������������ݴ�
extern float Gyro_y;        //Y�������������ݴ�
extern float Gyro_z;		 //Z�������������ݴ�
extern PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID;
extern float A_P,A_R,A2_P;
extern	float imua[3],imuw[3],imuangle[3];
extern int ctrlx,ctrly; 
extern float rollset,pitchset;
extern  MPU6050_AxisTypeDef    Axis;
//used  pins: A:9,10(USART1)  B:6,7,(TIM4OC)8,9(IIC) 10,11(TIM2OC)  C:6,7,8,9(TIM3OC)   TIM6   D:2345(key)
float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����


int main(void)
 { 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����
	delay_init(168);  //��ʼ����ʱ����
	uart_init(500000);		//��ʼ�����ڲ�����Ϊ500000
	uart6_init(500000);		//��ʼ�����ڲ�����Ϊ500000

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
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
//			mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
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
