#ifndef __FAN_H__
#define __FAN_H__
#include "sys.h"

//typedef struct
//{
//		float pitch,roll,yaw; 
//		short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
//		short gyrox,gyroy,gyroz;	//������ԭʼ����
//}imudata;

//typedef struct
//{
//		float pitch,roll,yaw; 
//		short wx,wy,wz;

//}expect;
uint16_t pwmcalc(float pos);
void fantest(void);
void fanmove(int pwmx,int pwmy);
void dotask1(void);
void dotask2(void);
void dotask3(void);
void dotask4(void);
void dotask5(void);

int getworkstate(void);
void controltask(void);

#endif 
