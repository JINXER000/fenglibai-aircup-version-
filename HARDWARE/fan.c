#include "fan.h"
#include "pwm.h"
#include "math.h"
#include "ospid.h"
#include "indkey.h"

extern 	float pitch,roll,yaw;
extern short aacx,aacy,aacz,gyrox,gyroy,gyroz;
extern PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID,AnglePID;
const float priod=1554.7;
float R=0.35;



int workstate=4;
int lastworkstate,startflag;
int ctrlx,ctrly,ctrlangle;                              //pwm order
float setanglexy=0.0,     
	   anglegoal,
     anglenow,
			pitchgoal,
			rollgoal,
				pitchset,
       rollset;
short gyroxgoal,gyroygoal,gyrozgoal;
float wuchajiao,wuchasu;

//  pitch+:motor1,    2,3,4 in order of clockwise

void setpicthpid(int p,int i,int d)
{
		

}
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
	if(pwmx>100)
	{		
		PWM8=0;
		PWM7=pwmx;

	}
	else if(pwmx<-100)
	{
		PWM8=-pwmx;
		PWM7=0;

	}
	else
	{
		PWM8=2500;
		PWM7=2500;
	}
		if(pwmy>100)
	{
		PWM6=0;					//m4 backward
		PWM5=pwmy;
	}
	else if(pwmy<-100)
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
			float R,A,omega,adjbias=1.108;
			MoveTimeCnt+=10;                       //10ms
		  R=0.35;
			 A=atan(R/0.805)*57.2958;
			 omega = 2.0*3.14159*(float)MoveTimeCnt/ priod*adjbias;	
	
			pitchset=A*sin(omega);
	 
			pitchgoal=pitchset;
//			PitchOPID.P=60;
//			PitchOPID.I=0.1;
//			PitchOPID.D=8000;
				PitchOPID.P=20;
			PitchOPID.I=0.5;
			PitchOPID.D=10000;

			ctrlx=Control_PitchPID();
			
			rollgoal=0;
			RollOPID.P=80;
			RollOPID.I=0.5;
			RollOPID.D=10000;
			ctrly=Control_RollPID();
	
fanmove(ctrlx,ctrly);
//fanmove(-2000,0);
}

void dotask2()           // len:30----60
{
				static uint32_t MoveTimeCnt = 0;
			float A,omega,adjbias=1.12;
			MoveTimeCnt+=10;                       //10ms

			 A=atan(R/0.805)*57.2958;
			 omega = 2.0*3.14159*(float)MoveTimeCnt/ priod*adjbias;	
	
			rollset=A*sin(omega);
	 
			pitchgoal=0;
			PitchOPID.P=60;
			PitchOPID.I=0;
			PitchOPID.D=80000;
			ctrlx=Control_PitchPID();
			
			rollgoal=rollset;
			RollOPID.P=80;//80
			RollOPID.I=0;//0
			RollOPID.D=10000;//3000
			ctrly=Control_RollPID();
	
fanmove(ctrlx,ctrly);

}
void dotask3()					// set angle 
{
	                 //相位补偿 0, 10   20   30   40   50   60   70   80   90   100  110  120  130  140  150  160  170 180
		 	const float amp1[19]= {0,0.0, 0.1,  0.1 ,0.1, 0.1, 0.1,0.15,0.17,  0, -0.05, 0.0,0.1, 0.1,  0.1,0.1,0.1,   0.1, 0};
//	    const float amp2[19]= {0,0.0, 0.1,  0.1 ,0.1, 0.1, 0.1,0.15,0.17,  0, 0.0, 0.0, 0.0, 0.0,0.05,0.05,0.05,0.07,0};				
    	static uint32_t MoveTimeCnt = 0;
			float Ax,Ay,A,omega,adjbias=1.1;
			int pOffset = 0;
				
			MoveTimeCnt+=10;                       //10ms
			
	
	//get angleset
	R=0.35;
	A=atan(R/0.805)*57.2958;
	Ax=A*cos(setanglexy/57.2958);
	Ay=A*sin(setanglexy/57.2958);
	
	
			 omega = 2.0*3.14159*(float)MoveTimeCnt/ priod*adjbias;	
	
	

		pOffset = (uint32_t)(setanglexy/10.0f);			 //相位补偿数组下标
		
	
			rollset=Ay*sin(omega)/*(1-amp2[pOffset])*/;
			pitchset=Ax*sin(omega)*(1-amp1[pOffset]);
	
			pitchgoal=pitchset;
			PitchOPID.P=60;
			PitchOPID.I=0.5;
			PitchOPID.D=20000;//10000
			ctrlx=Control_PitchPID();
			
			rollgoal=rollset;
			RollOPID.P=100;
			RollOPID.I=0.5;
			RollOPID.D=15000;//10000
			ctrly=Control_RollPID();

		
		anglenow=atan2(tan(roll),tan(pitch));
//		ctrlangle=keepangle();
//		ctrly+=ctrlangle;          //xy角度随roll单调增
fanmove(ctrlx,ctrly);

	
}
void dotask4()
{
		if(fabs(pitch)<45&&fabs(roll)<45)
		{
			pitchgoal=0;
			PitchOPID.P=80;
			PitchOPID.I=0;
			PitchOPID.D=20000;
			ctrlx=Control_PitchPID();
			
			rollgoal=0;
			RollOPID.P=80;
			RollOPID.I=0;
			RollOPID.D=20000;
			ctrly=Control_RollPID();
			
//			fanmove(0,2000);
		
			fanmove(ctrlx,ctrly);
//	fanmove(ctrlx,0);
//			fanmove(0,ctrly);

		}
}
void dotask5()
{
					static uint32_t MoveTimeCnt = 0;
			float Ax,Ay,A,omega,adjbias=1.13;
			MoveTimeCnt+=10;                       //10ms
		  
	A=atan(R/0.805)*57.2958;
			 omega = 2.0*3.14159*(float)MoveTimeCnt/ priod*adjbias;	
	
			rollset=1.05*A*sin(omega);
			pitchset=A*cos(omega);
	
			pitchgoal=pitchset;
			PitchOPID.P=70;
			PitchOPID.I=0.5;
			PitchOPID.D=12000;
			ctrlx=Control_PitchPID();
			
			rollgoal=rollset;
			RollOPID.P=100;
			RollOPID.I=0.5;
			RollOPID.D=10000;
			ctrly=Control_RollPID();

		fanmove(ctrlx,ctrly);
	
		
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
			case 5:{
				dotask5();
			}break;

			default:break;
		}

}


