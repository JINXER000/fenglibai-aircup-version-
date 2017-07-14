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
	PWM2=500;
	PWM3=500;
	PWM4=300;

	
	PWM5=500;
	PWM6=500;
	PWM7=500;
	PWM8=300;

}
void fanmove(int pwmx,int pwmy)     
{
	if(pwmx>200)
	{		
		PWM8=pwmx;
		PWM7=0;

	}
	else if(pwmx<-200)
	{
		PWM8=0;
		PWM7=-pwmx;

	}
	else
	{
		PWM8=0;
		PWM7=0;
	}
		if(pwmy>200)
	{
		PWM6=0;					//m4 backward
		PWM5=pwmy;
	}
	else if(pwmy<-200)
	{
		PWM6=-pwmy;					//m4 backward
		PWM5=0;

	}
	else
	{
		PWM6=0;
		PWM5=0;
	}
//		if(pwmy<0)
//	{
//		PWM6=0;					//m4 backward
//		PWM5=pwmy;
//	}
//	else
//	{
//		PWM6=pwmy;					//m4 backward
//		PWM5=0;

//	}

}


void dotask1()        //50 cm in 15s,+-2.5com straight line
{
			static uint32_t MoveTimeCnt = 0;
			float R,A,omega;
			MoveTimeCnt+=10;                       //10ms
		  R=0.3;
			 A=atan(R/0.805)*57.2958;
			 omega = 2.0*3.14159*(float)MoveTimeCnt/ priod;	
	
			pitchset=A*sin(omega);
	 
			pitchgoal=pitchset;
			PitchOPID.P=60;
			PitchOPID.I=0.3;
			PitchOPID.D=800;
			ctrlx=Control_PitchPID();
			
			rollgoal=0;
			RollOPID.P=80;
			RollOPID.I=0;
			RollOPID.D=3000;
			ctrly=Control_RollPID();
	
//fanmove(ctrlx,ctrly);
fanmove(-2000,0);
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
			PitchOPID.D=5000;
			ctrlx=Control_PitchPID();
			
			rollgoal=0;
			RollOPID.P=80;
			RollOPID.I=0;
			RollOPID.D=5000;
			ctrly=Control_RollPID();
			
//			fanmove(0,-2000);
		
			fanmove(ctrlx,ctrly);
//	fanmove(ctrlx,0);

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
