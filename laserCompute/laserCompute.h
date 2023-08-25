#ifndef __LASER_COMPUTE_H__
#define __LASER_COMPUTE_H__

#include <stdint.h>

/* 标准差 */
#define Dx_BIAS 0.08f    //滤波的条件，标准差大于该值停止接收信号

/*  */
#define N_DIS 1.0f  //当接近水平时转向角删除删除方向角

#define ERROR_E 5.0f  //估计误差

#define B_ANGLE 20.0f  //斜面角度，用来选择测量误差

#define TRANSIENT 5.0f  //瞬态变化量，小于该值停止滤波

#define ERROR_MB 3.0f  //斜面角的测量误差

#define ERROR_MD 5.0f  //方向角的测量误差

#define DIR_K 0.5f  //IMU的权重，0到1之间

#define LASER_LENTH_01 22.0    
#define LASER_LENTH_12 22.0



typedef struct LaserOutput
{
	float avgDistance;
	float bevelAngle;
	float directionAngle;
}LaserOutput_t;

typedef struct FusionInput
{
	float dis0;
	float dis1;
	float dis2;
	float dis3;

	float roll;
	float pitch;
	float yaw;
}FusionInput_t;

//typedef struct kalman
//{
//	float measure_k;
//	float error_mk;
//	float x_k;
//	float k;
//	float error_ek;
//}kalman_t;

/* 函数声明 */
uint8_t calculation(FusionInput_t input, LaserOutput_t *output);


#endif

