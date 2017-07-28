#include "JY61.h"
#include "string.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;


	float imua[3],imuw[3],imuangle[3];
		 unsigned char ucRxBuffer[250];
	 unsigned char ucRxCnt = 0;	

void CopeSerial2Data(unsigned char ucData)
{
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;
		imua[0]=(float)stcAcc.a[0]/32768*16;
			imua[1]=(float)stcAcc.a[1]/32768*16;
			imua[2]=(float)stcAcc.a[2]/32768*16;
			imuw[0]=(float)stcGyro.w[0]/32768*2000;
			imuw[1]=(float)stcGyro.w[1]/32768*2000;
			imuw[2]=(float)stcGyro.w[2]/32768*2000;
			imuangle[0]=(float)stcAngle.Angle[0]/32768*180;
			imuangle[1]=(float)stcAngle.Angle[1]/32768*180;
			imuangle[2]=(float)stcAngle.Angle[2]/32768*180;
	}
}
