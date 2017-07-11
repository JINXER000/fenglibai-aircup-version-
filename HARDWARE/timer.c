#include "timer.h"
#include "fan.h"
#include "mpu6050.h"
//change to tim 14 if possible

void TIM6_Int_Init(u16 arr,u16 psc)  //arr=1000, psc=840  
{
	
	    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = psc-1;        //84M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = arr;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
		
		TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	 TIM_Cmd(TIM6,ENABLE); //ʹ�ܶ�ʱ��3

}

void TIM6_DAC_IRQHandler(void)  
{
	
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
////	scan key
//		uint8_t i = 0;
//	float pitch_temp1 = 0.0;
//	float roll_temp1 = 0.0;
//	float pitch_temp2 = 0.0;
//	float roll_temp2 = 0.0;
//	static float pitch_sum = 0.0;
//	static float roll_sum = 0.0;
//		
////	GPIOE->BSRR = GPIO_Pin_3;
//	if(TIM_GetITStatus(TIM5,TIM_IT_Update) == SET)
//	{		

//		
//		for(i=0;i<3;i++)
//		{
//			
//			pitch_temp1 = (atan(aacx/Axis.AccZ)*57.2958-0.4);   //����Pitch�Ƕ� 0.4Ϊ��̬ƫ���
//			roll_temp1  = (atan(Axis.AccY/Axis.AccZ)*57.2958-0.3);   //����Roll�Ƕ�  0.3Ϊ��̬ƫ���
//			
//			pitch_sum += pitch_temp1;
//			roll_sum  += roll_temp1;
//		}
//		
//		pitch_temp1 = pitch_sum / 3.0;	 //ȡ��ƽ��ֵ
//		roll_temp1  = roll_sum  / 3.0;	 //ȡ��ƽ��ֵ

//		pitch_sum = 0.0;
//		roll_sum = 0.0;
//		
//		EulerAngle.Pitch = Kalman_Filter(pitch_temp1,Axis.GyroY);       //�������˲���
//		EulerAngle.Roll  = Kalman_Filter(roll_temp1,-Axis.GyroX);       //�������˲���
//		
//		M1.CurPos = EulerAngle.Pitch; 
//		M2.CurPos = EulerAngle.Roll;						   
//		
//		//�����ٶ�
//		M1.CurSpeed = M1.CurPos - M1.PrevPos;
//		M1.PrevPos = M1.CurPos;				
//		

//				
//		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);		
//	}
//	GPIOE->BRR = GPIO_Pin_3;				





//			
		controltask();
			
			
    }
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);  //����жϱ�־λ

}



