#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>

#define pi 3.1415926535f
#define length 11.0f //���������Ķ���֮һ
#define MaxBevel 10.0f //б���׼���������ֵ
#define filterNUM 100 //ƽ���˲�����
#define sensors 4 //����������
#define STD_BIAS 30.0f //����Ǽ���ƫ��
#define MError 2.0f
#define EError 1.5f

//һ�м��㶼������Ϊ׼���ǵý��������������Ϊ��λ����Ϊ����Ϊ��λ

typedef struct angle
{
	float bevelAngle[4];
	float directionAngle[4];
}angle_t;

typedef struct output
{
	float bevelAngle;
	float directionAngle;
    
}output_t;

typedef struct input
{
	float dis[4];

	float roll;
	float pitch;
	float yaw;
}input_t;

typedef struct kalman
{
	float measurement; //����ֵ
	float P; //�������������Э���
	//���Թ�������Э����
	float R; //��������������Э���
	float X; //״̬����ֵ
	float K; //����������
}kalman_t;

/* �ⲿ�������� */
angle_t LaserCalculation(input_t inputData);
uint8_t avg(angle_t angle, output_t *out);
float filter(float* num, float now);
kalman_t kalman(kalman_t angle);
void fusion(float* yaw, float* dir, float yawP, float dirP, float* dirAngle);

