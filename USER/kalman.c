#include "math.h"
#include "ahrs.h"


/*------------------------------------------
 函数功能: 卡尔曼滤波1
 函数参数: 无
------------------------------------------*/
float Kalman_Filter1(float Accel,float Gyro)		
{
	static float Gyro_y;   //Y轴陀螺仪数据暂存
	static float angle = 0.0;
	static float Q_bias = 0.0;
	static float angle_err = 0.0;
	static float Q_angle = 0.1;  //角度数据置信度
	static float Q_gyro = 0.1;   //角速度数据置信度
	static float R_angle = 0.5;
	static float dt = 0.005;	   //dt为滤波器采样时间(秒)
	static char  C_0 = 1;
	static float PCt_0=0, PCt_1=0, E=0;
	static float K_0=0, K_1=0, t_0=0, t_1=0;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	angle += (Gyro - Q_bias) * dt; //先验估计
	angle_err = Accel - angle;	//zk-先验估计

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]= - PP[1][1];
	Pdot[2]= - PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //最优角度
	Q_bias += K_1 * angle_err;	 
	Gyro_y = Gyro - Q_bias;	 //最优角速度

	return (angle);				 //最终角度			 
}

/*------------------------------------------
 函数功能: 卡尔曼滤波2
 函数参数: 无
------------------------------------------*/
float Kalman_Filter2(float Accel,float Gyro)		
{
	static float Gyro_y;   //Y轴陀螺仪数据暂存
	static float angle = 0.0;
	static float Q_bias = 0.0;
	static float angle_err = 0.0;
	static float Q_angle = 0.1;  //角度数据置信度
	static float Q_gyro = 0.1;   //角速度数据置信度
	static float R_angle = 0.5;
	static float dt = 0.005;	   //dt为滤波器采样时间(秒)
	static char  C_0 = 1;
	static float PCt_0=0, PCt_1=0, E=0;
	static float K_0=0, K_1=0, t_0=0, t_1=0;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	angle += (Gyro - Q_bias) * dt; //先验估计
	angle_err = Accel - angle;	//zk-先验估计

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]= - PP[1][1];
	Pdot[2]= - PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //最优角度
	Q_bias += K_1 * angle_err;	 
	Gyro_y = Gyro - Q_bias;	 //最优角速度

	return (angle);				 //最终角度			 
}


