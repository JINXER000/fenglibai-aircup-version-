#include "fan.h"
#include "pwm.h"
#include "math.h"
#include "ospid.h"
#include "indkey.h"

extern 	float pitch,roll,yaw;
extern short aacx,aacy,aacz,gyrox,gyroy,gyroz;
extern PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID;
const float priod=1554.7;



int workstate=4,lastworkstate,startflag;
int ctrlx,ctrly;                              //pwm order
float setanglexy,     
	   anglegoal,
     anglenow,
			pitchgoal,
			rollgoal,
				pitchset,
       rollset;
short gyroxgoal,gyroygoal,gyrozgoal;
float wuchajiao,wuchasu;

//  pitch+:motor1,    2,3,4 in order of clockwise

void fantest()
{
		PWM1=500;
	PWM2=1000;
	PWM3=1500;
	PWM4=2300;

	
	PWM5=500;
	PWM6=1000;
	PWM7=1500;
	PWM8=2300;

}
void fanmove(int pwmx,int pwmy)     
{
//	startflag=1;	                                 //swing
//	if(anglenow>0||anglenow<wuchajiao)
//		{
//			if(gyrox>0)   
//			{
//				//m3++,m1=0
//			}
//			else
//			{
//				//m1++,m3=0
//			}
//		if(anglenow>wuchajiao)	
//		{
//			
//		}
//		}
//		startflag=0;
	if(pwmx>0)
	{
		PWM1=pwmx;      //m1 forward
		PWM5=0;
		PWM3=0;					//m3 backward
		PWM7=pwmx;
	}
	else
	{
		PWM1=0;						//m1 backward
		PWM5=pwmx;
		PWM3=pwmx;     //m3 forward
		PWM7=0;

	}
		if(pwmy>0)
	{
		PWM2=pwmy;      //m2 forward
		PWM6=0;
		PWM4=0;					//m4 backward
		PWM8=pwmy;
	}
	else
	{
		PWM2=0;						//m2 backward
		PWM6=pwmy;
		PWM4=pwmy;     //m4 forward
		PWM8=0;

	}

}


void dotask1()        //50 cm in 15s,+-2.5com straight line
{
			static uint32_t MoveTimeCnt = 0;
			float R,A,omega;
			MoveTimeCnt+=10;                       //10ms
		  R=0.3;
			 A=atan(R/0.6)*57.2958;
			 omega = 2.0*3.14159*(float)MoveTimeCnt/ priod;	
	
			rollset=A*sin(omega);
	 
			pitchgoal=0;
			PitchOPID.P=80;
			PitchOPID.I=0;
			PitchOPID.D=1000;
			ctrlx=Control_PitchPID();
			
			rollgoal=rollset;
			RollOPID.P=80;
			RollOPID.I=0;
			RollOPID.D=1000;
			ctrly=Control_RollPID();
	
			fanmove(ctrlx,ctrly);

}

void dotask2()           // len:30----60
{
	anglenow=pitch;
}
void dotask3()					// set angle 
{

}
void dotask4()
{
		if(fabs(pitch)<45&&fabs(roll)<45)
		{
			pitchgoal=0;
			PitchOPID.P=80;
			PitchOPID.I=0;
			PitchOPID.D=1000;
			ctrlx=Control_PitchPID();
			
			rollgoal=0;
			RollOPID.P=80;
			RollOPID.I=0;
			RollOPID.D=1000;
			ctrly=Control_RollPID();
		
			fanmove(ctrlx,ctrly);
		}
}
int getworkstate()                                     //get input from button
{
	KeyScan();
	if(lastworkstate!=workstate)        
	{
		initconfig();
		lastworkstate=workstate;
	}
	
	
	return workstate;
}

void controltask()
{
		switch(getworkstate())
		{
			case 1:{
				dotask1();
			}break;
			case 2:{
				dotask2();
			}break;
			case 3:{
				dotask3();
			}break;
			case 4:{
				dotask4();
			}break;
			default:break;
		}

}
