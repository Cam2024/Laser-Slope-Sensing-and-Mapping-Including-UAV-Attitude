#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>

#define pi 3.1415926535f
#define length 11.0f //传感器间距的二分之一
#define MaxBevel 10.0f //斜面标准差的最大接受值
#define filterNUM 100 //平滑滤波个数
#define sensors 4 //传感器数量
#define STD_BIAS 30.0f //方向角计算偏差
#define MError 2.0f
#define EError 1.5f

//一切计算都以厘米为准，记得将传感器输出的米为单位更改为厘米为单位

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
	float measurement; //测量值
	float P; //估计误差（估计误差协方差）
	//忽略过程噪声协方差
	float R; //测量误差（测量噪声协方差）
	float X; //状态估计值
	float K; //卡尔曼增益
}kalman_t;

/* 外部函数声明 */
angle_t LaserCalculation(input_t inputData);
uint8_t avg(angle_t angle, output_t *out);
float filter(float* num, float now);
kalman_t kalman(kalman_t angle);
void fusion(float* yaw, float* dir, float yawP, float dirP, float* dirAngle);

