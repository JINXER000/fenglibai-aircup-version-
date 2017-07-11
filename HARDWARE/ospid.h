#ifndef __OSPID_H__
#define __OSPID_H__

#include "sys.h"

typedef struct
{
	float P;
	float I;
	float D;
	
	float CurrentError;
	float LastError;
	float Pout;
	float Iout;
	float Dout;
	float PIDout;
	
	float IMax;
	float PIDMax;
	
//	portTickType LastTick;
}PID_Type;

void initconfig(void);
int16_t Control_RollPID(void);
int16_t Control_PitchPID(void);
#endif
