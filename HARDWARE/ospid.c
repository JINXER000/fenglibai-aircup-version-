#include "ospid.h"
#include "math.h"

 PID_Type PitchOPID,PitchIPID,RollIPID,RollOPID,AnglePID  ;

extern float setanglexy, anglegoal,anglenow,pitchgoal,rollgoal;
extern short gyroxgoal,gyroygoal,gyrozgoal;
extern 	float pitch,roll,yaw;
extern short aacx,aacy,aacz,gyrox,gyroy,gyroz;
float pitcherrbias=-0.94,rollerrbias=-0.65;

void initconfig()
{
	  PitchOPID.P = 80;
    PitchOPID.I = 0;
    PitchOPID.D = 0;
    PitchOPID.CurrentError = 0;
    PitchOPID.LastError = 0;
//    PitchOPID.LastTick = 0;
    PitchOPID.IMax = 2300;
    PitchOPID.PIDMax = 2500;
    
//    PitchIPID.P = 80;
//    PitchIPID.I = 0;
//    PitchIPID.D = 20;
//    PitchIPID.CurrentError = 0;
//    PitchIPID.LastError = 0;
////    PitchIPID.LastTick = 0;
//    PitchIPID.IMax = 0;
//    PitchIPID.PIDMax = 5000;
	

	  RollOPID.P = 80;
    RollOPID.I = 0;
    RollOPID.D = 0;
    RollOPID.CurrentError = 0;
    RollOPID.LastError = 0;
//    RollOPID.LastTick = 0;
    RollOPID.IMax = 2300;
    RollOPID.PIDMax = 2500;
    
//    RollIPID.P = 80;
//    RollIPID.I = 0;
//    RollIPID.D = 20;
//    RollIPID.CurrentError = 0;
//    RollIPID.LastError = 0;
////    RollIPID.LastTick = 0;
//    RollIPID.IMax = 0;
//    RollIPID.PIDMax = 5000;
		AnglePID.P=10;
		AnglePID.I=0;
		AnglePID.D=1000;
		AnglePID.CurrentError=0;
		AnglePID.LastError=0;
		AnglePID.IMax=2000;
		AnglePID.PIDMax=2300;

}

int16_t Control_PitchPID(void)
{
	
	PitchOPID.CurrentError=pitchgoal-pitch-pitcherrbias;
	PitchOPID.Pout = PitchOPID.P * PitchOPID.CurrentError;
	
	if (PitchOPID.CurrentError>10)
	{
		PitchOPID.index=0;
	}
	else
	{
		PitchOPID.index=1;
		PitchOPID.Iout += PitchOPID.I * PitchOPID.CurrentError;
	PitchOPID.Iout = PitchOPID.Iout > PitchOPID.IMax ? PitchOPID.IMax : PitchOPID.Iout;
	PitchOPID.Iout = PitchOPID.Iout < -PitchOPID.IMax ? -PitchOPID.IMax : PitchOPID.Iout;
	}
	
		PitchOPID.Dout = PitchOPID.D *(PitchOPID.CurrentError-PitchOPID.LastError);//+-?
	
	PitchOPID.PIDout = PitchOPID.Pout +PitchOPID.index* PitchOPID.Iout + PitchOPID.Dout;
	PitchOPID.PIDout = PitchOPID.PIDout > PitchOPID.PIDMax ? PitchOPID.PIDMax : PitchOPID.PIDout;
	PitchOPID.PIDout = PitchOPID.PIDout < -PitchOPID.PIDMax ? -PitchOPID.PIDMax : PitchOPID.PIDout;
	
		PitchOPID.LastError = PitchOPID.CurrentError;
return (short)PitchOPID.PIDout;
	
}

int16_t Control_RollPID(void)
{
	RollOPID.CurrentError=rollgoal-roll-rollerrbias;
	RollOPID.Pout = RollOPID.P * RollOPID.CurrentError;
	
		if (RollOPID.CurrentError>10)
	{
		RollOPID.index=0;
	}
	else
	{
		RollOPID.index=1;
		RollOPID.Iout += RollOPID.I * RollOPID.CurrentError;
	RollOPID.Iout = RollOPID.Iout > RollOPID.IMax ? RollOPID.IMax : RollOPID.Iout;
	RollOPID.Iout = RollOPID.Iout < -RollOPID.IMax ? -RollOPID.IMax : RollOPID.Iout;
	}


		RollOPID.Dout = RollOPID.D *(RollOPID.CurrentError-RollOPID.LastError);
	
	RollOPID.PIDout = RollOPID.Pout + RollOPID.index*RollOPID.Iout + RollOPID.Dout;
	RollOPID.PIDout = RollOPID.PIDout > RollOPID.PIDMax ? RollOPID.PIDMax : RollOPID.PIDout;
	RollOPID.PIDout = RollOPID.PIDout < -RollOPID.PIDMax ? -RollOPID.PIDMax : RollOPID.PIDout;
	
		RollOPID.LastError = RollOPID.CurrentError;

	return (short)RollOPID.PIDout;
}

int16_t keepangle()
{
	AnglePID.CurrentError=setanglexy-anglenow;		
	AnglePID.Pout=AnglePID.P*AnglePID.CurrentError;
	
		if (AnglePID.CurrentError>5)
	{
		AnglePID.index=0;
	}
	else
	{
		AnglePID.index=1;
		AnglePID.Iout += AnglePID.I * AnglePID.CurrentError;
	AnglePID.Iout = AnglePID.Iout > AnglePID.IMax ? AnglePID.IMax : AnglePID.Iout;
	AnglePID.Iout = AnglePID.Iout < -AnglePID.IMax ? -AnglePID.IMax : AnglePID.Iout;
	}


		AnglePID.Dout = AnglePID.D *(AnglePID.CurrentError-AnglePID.LastError);
	
	AnglePID.PIDout = AnglePID.Pout + AnglePID.index*AnglePID.Iout + AnglePID.Dout;
	AnglePID.PIDout = AnglePID.PIDout > AnglePID.PIDMax ? AnglePID.PIDMax : AnglePID.PIDout;
	AnglePID.PIDout = AnglePID.PIDout < -AnglePID.PIDMax ? -AnglePID.PIDMax : AnglePID.PIDout;
	
		AnglePID.LastError = AnglePID.CurrentError;

	return (short)RollOPID.PIDout;

	
}
//int16_t Increment_PitchPID(int pwmx)
//{
//		RollOPID.CurrentError=rollgoal-roll-rollerrbias;
//		RollOPID.Pout=RollOPID.P*(RollOPID.CurrentError-RollOPID.LastError);
//			RollOPID.Iout= RollOPID.I*RollOPID.CurrentError;
//	RollOPID.Dout=RollOPID.D*(RollOPID.CurrentError-2*RollOPID.LastError+RollOPID.NextError);
//	RollOPID.PIDout = RollOPID.Pout + RollOPID.Iout + RollOPID.Dout;
//	pwmx+=RollOPID.PIDout;
//	
//	RollOPID.LastError=RollOPID.CurrentError;
//	RollOPID.NextError=RollOPID.LastError;
//	return pwmx;

//}
