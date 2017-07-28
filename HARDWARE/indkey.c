#include "indkey.h"
#include "sys.h"
#include "delay.h"
#include "stdio.h"
#include "mpu6050.h"

uint8_t Item = 0;

u8 CurMode = 0;

extern float R;
extern float setanglexy;
extern int workstate;

void Key_IO_Init(void)
{
	GPIO_InitTypeDef IO_Init;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);	
	IO_Init.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	IO_Init.GPIO_Mode = GPIO_Mode_IN;
	IO_Init.GPIO_Speed = GPIO_Speed_100MHz;
	IO_Init.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&IO_Init);
}

void KeyScan(void)
{
	static u8 key1_up=1;
	static u8 key2_up=1;
	static u8 key3_up=1;
	static u8 key4_up=1;
  if(key1_up && (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)  == KEY_PRESSED))
	{
		delay_ms(10);
		key1_up=0;
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)  == KEY_PRESSED)
		{
		  switch(workstate)
		  {
			  case 2:R+=0.05f;
			         if(R>=0.35f) 
					  	   R=0.35f;
			         //显示
			         break;
			  case 3:setanglexy+=10.0f;
			         if(setanglexy>=180.0f) 
					  	   setanglexy=180.0f;
					     //显示
					     break;
				case 5:R+=0.05f;
			         if(R>=0.35f) 
					  	   R=0.35f;
			         //显示
			         break;
			  default:break;		 
		  }
		}		
	}
	if(key2_up && (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)  == KEY_PRESSED))
	{
		delay_ms(10);
		key2_up=0;
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)  == KEY_PRESSED)
		{
		  switch(workstate)
		  {
			  case 2:R-=0.05f;
			         if(R<=0.1f)
					  	   R=0.1f;
					     //显示
					     break;
			  case 3:setanglexy-=10.0f;
				  	   if(setanglexy<=0.0f)
					  	   setanglexy=0.0f;
					     //显示
					     break;
				case 5:R-=0.05f;
			         if(R<=0.1f)
					  	   R=0.1f;
					     //显示
					     break;
			  default:break;
		  }
		}
	}
	if(key3_up && (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)  == KEY_PRESSED))
	{
		delay_ms(10);
		key3_up=0;
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)  == KEY_PRESSED)
			{
		    switch(Item)
		    {
			    case 1:workstate = 1;
			           //显示
					       Item=0;
			           break;
			    case 2:workstate = 2;
				         //显示
					       Item=0;
			           break;
			    case 3:workstate = 3;
			           //显示
					       Item=0;
			           break;
			    case 4:workstate = 4;
			           //显示
					       Item=0;
			           break;
					case 5:workstate = 5;
					       //显示
					       Item=0;
					       break;
			    default:break;
				}
		  }
		
	}
	if(key4_up && (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)  == KEY_PRESSED))
	{
		delay_ms(10);
		key4_up=0;
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)  == KEY_PRESSED)
		{
			Item++;
			if(Item>5)
				Item=0;
			//xianshi
		}
	}
	else if((GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2))==1)
	{
		key1_up=1;
		key2_up=1;
		key3_up=1;
		key4_up=1;
	}
}




