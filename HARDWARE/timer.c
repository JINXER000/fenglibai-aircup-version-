#include "timer.h"
#include "fan.h"
#include "mpu6050.h"
#include "Kalman_filter.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "ospid.h"
 int i,dmpresetCounter;

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
extern 	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
extern	short gyrox,gyroy,gyroz;	//������ԭʼ����
extern float pitch,roll,yaw; 		//ŷ����
extern void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
extern int ctrlx,ctrly; 
extern	float imua[3],imuw[3],imuangle[3];
extern float rollset,pitchset;
extern PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID;

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
		
		TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	 TIM_Cmd(TIM6,ENABLE); //ʹ�ܶ�ʱ��3

}

void TIM6_DAC_IRQHandler(void)  
{
    
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
			
//get angle ,calc and filter
//		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
//		{ 
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
//					if(i<300)															//waiting for self test
//					{
//						i++;

//					}else
//					{
//						controltask();
//						Angle_Calcu();
//					}
					
//				 if(dmpresetCounter >= 500)						//restart every 5s
//        {
//            dmpresetCounter = 0;
//            mpu_set_dmp_state(0);
//            mpu_set_dmp_state(1);
//        }
//        else
//        {
//            dmpresetCounter++;
//        }

					
	controltask();
	usart1_report_imu(pitch,pitchset,ctrlx/25,ctrly/25,PitchOPID.Pout,PitchOPID.Dout,RollOPID.Pout,RollOPID.Dout,(int)(yaw*10));
			
//		}
//		
//  else
//	{
//			i--;
//	}
		
		
			
			
    }
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);  //����жϱ�־λ

}



