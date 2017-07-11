#include "math.h"
#include "ahrs.h"


/*------------------------------------------
 ��������: �������˲�1
 ��������: ��
------------------------------------------*/
float Kalman_Filter1(float Accel,float Gyro)		
{
	static float Gyro_y;   //Y�������������ݴ�
	static float angle = 0.0;
	static float Q_bias = 0.0;
	static float angle_err = 0.0;
	static float Q_angle = 0.1;  //�Ƕ��������Ŷ�
	static float Q_gyro = 0.1;   //���ٶ��������Ŷ�
	static float R_angle = 0.5;
	static float dt = 0.005;	   //dtΪ�˲�������ʱ��(��)
	static char  C_0 = 1;
	static float PCt_0=0, PCt_1=0, E=0;
	static float K_0=0, K_1=0, t_0=0, t_1=0;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	angle += (Gyro - Q_bias) * dt; //�������
	angle_err = Accel - angle;	//zk-�������

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
	Pdot[1]= - PP[1][1];
	Pdot[2]= - PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //���ŽǶ�
	Q_bias += K_1 * angle_err;	 
	Gyro_y = Gyro - Q_bias;	 //���Ž��ٶ�

	return (angle);				 //���սǶ�			 
}

/*------------------------------------------
 ��������: �������˲�2
 ��������: ��
------------------------------------------*/
float Kalman_Filter2(float Accel,float Gyro)		
{
	static float Gyro_y;   //Y�������������ݴ�
	static float angle = 0.0;
	static float Q_bias = 0.0;
	static float angle_err = 0.0;
	static float Q_angle = 0.1;  //�Ƕ��������Ŷ�
	static float Q_gyro = 0.1;   //���ٶ��������Ŷ�
	static float R_angle = 0.5;
	static float dt = 0.005;	   //dtΪ�˲�������ʱ��(��)
	static char  C_0 = 1;
	static float PCt_0=0, PCt_1=0, E=0;
	static float K_0=0, K_1=0, t_0=0, t_1=0;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	angle += (Gyro - Q_bias) * dt; //�������
	angle_err = Accel - angle;	//zk-�������

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
	Pdot[1]= - PP[1][1];
	Pdot[2]= - PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //���ŽǶ�
	Q_bias += K_1 * angle_err;	 
	Gyro_y = Gyro - Q_bias;	 //���Ž��ٶ�

	return (angle);				 //���սǶ�			 
}


