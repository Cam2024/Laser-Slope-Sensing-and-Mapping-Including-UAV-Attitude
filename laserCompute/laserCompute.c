#include "laserCompute.h"
#include "math.h"
#include "bsp_print.h"
#include "inv_mpu.h"

#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include<time.h>

#define pi 3.1415926535f

/**
  * @brief  根据四点测距计算斜面参数
  * @param  dis0 距离值0
  * @param  dis1 距离值1
  * @param  dis2 距离值2
  * @param  dis3 距离值3  
  * @param  laserOutput 输出的解算数据
  * @retval 返回0，数据计算正常，返回1，数据偏差过大
  */
uint8_t calculation(FusionInput_t input, LaserOutput_t *output)
{
	float point[4][3] = { 0 };
	float theta[4] = { 0 };
	float dirTheta[4] = { 0 };
	float vectorA[3] = { 0 };
	float vectorB[3] = { 0 };
	float vectorC[3] = { 0 };
	float vectorD[3] = { 0 };
	float Ex = 0;
	float Dx = 0;

	//point 0
	point[0][0] = 11;
	point[0][1] = 11;
	point[0][2] = -input.dis0;
	//point 1
	point[1][0] = 11;
	point[1][1] = -11;
	point[1][2] = -input.dis1;
	//point 2
	point[2][0] = -11;
	point[2][1] = -11;
	point[2][2] = -input.dis2;
	//point 3
	point[3][0] = -11;
	point[3][1] = 11;
	point[3][2] = -input.dis3;

	for (int count = 0, i = 0, j = 1, k = 2; count < 4; count++)
	{
		//get the angle of bevel and the angle of direction
		vectorA[0] = point[j][0] - point[i][0];
		vectorA[1] = point[j][1] - point[i][1];
		vectorA[2] = point[j][2] - point[i][2];
		//vector 02
		vectorB[0] = point[k][0] - point[i][0];
		vectorB[1] = point[k][1] - point[i][1];
		vectorB[2] = point[k][2] - point[i][2];
		//vector 01 x 02
		vectorC[0] = vectorB[1] * vectorA[2] - vectorA[1] * vectorB[2];
		vectorC[1] = vectorB[2] * vectorA[0] - vectorA[2] * vectorB[0];
		vectorC[2] = vectorB[0] * vectorA[1] - vectorA[0] * vectorB[1];

		//旋转矩阵
		//x轴
		vectorD[0] = vectorC[0];
		vectorD[1] = cos(-input.roll*pi/180.0f) * vectorC[1] - sin(-input.roll*pi/180.0f) * vectorC[2];
		vectorD[2] = sin(-input.roll*pi/180.0f) * vectorC[1] + cos(-input.roll*pi/180.0f) * vectorC[2];
		//y轴
		vectorD[0] = cos(-input.pitch*pi/180.0f) * vectorD[0] + sin(-input.pitch*pi/180.0f) * vectorD[2];
		vectorD[1] = vectorD[1];
		vectorD[2] = -sin(-input.pitch*pi/180.0f) * vectorD[0] + cos(-input.pitch*pi/180.0f) * vectorD[2];



		vectorC[0] = vectorD[0];
		vectorC[1] = vectorD[1];
		vectorC[2] = vectorD[2];


		//angle
		theta[count] = (180 * acos((fabs(vectorC[2])) / (sqrt(pow(vectorC[0], 2) + pow(vectorC[1], 2) + pow(vectorC[2], 2))))) / pi;
		if ((fabs(vectorC[0]) < N_DIS) && (fabs(vectorC[1]) < N_DIS)) //注意：此处更改了转向角度的判定为绝对值，后续需要调节参数
		{
			dirTheta[count] = 0;
		}
		else
		{
			dirTheta[count] = (180 * acos((fabs(vectorC[1])) / (sqrt(pow(vectorC[0], 2) + pow(vectorC[1], 2))))) / pi;
			
			// 以下部分是为了修正转向角度从-179到180
			if (vectorC[0] < 0)
			{
				if (vectorC[1] < 0)
				{
					dirTheta[count] = dirTheta[count] - 180;
				}
				else
				{
					dirTheta[count] = -dirTheta[count];
				}
			}
			else
			{
				if (vectorC[1] < 0)
				{
					dirTheta[count] = 180 - dirTheta[count];
				}
			}
		}

		
		if (count < 1)
		{
			k += 1;
		}
		else if (count < 2)
		{
			j += 1;
		}
		else if (count < 3)
		{
			i += 1;
		}
	}

	//判断数据是否偏差过大
	Ex = (theta[0] + theta[1] + theta[2] + theta[3]) / 4;
	for (int i = 0; i < 4; i++)
	{
		Dx = Dx + pow((theta[i] - Ex), 2);
	}

	Dx = sqrt(Dx / 4);   //此处是标准差

    if(Dx > Dx_BIAS)
    return 1;

    
	if (Dx < Dx_BIAS) //在这里调整滤波参数，初始为 2
	{
		output->avgDistance = (input.dis0 + input.dis1 + input.dis2 + input.dis3) / 4;
		output->bevelAngle = Ex;
		output->directionAngle = (dirTheta[0] + dirTheta[1] + dirTheta[2] + dirTheta[3]) / 4;
		//以下部分只是为了解决在180度附近产生的179与-179度无法平均的问题
		if (fabs(output->directionAngle - dirTheta[0]) > 30)
		{
			for (int i = 0; i < 4; i++)
			{
				if (dirTheta[i] < 0)
				{
					dirTheta[i] = -180 - dirTheta[i];
				}
				else
				{
					dirTheta[i] = 180 - dirTheta[i];
				}
			}
			output->directionAngle = (dirTheta[0] + dirTheta[1] + dirTheta[2] + dirTheta[3]) / 4;
			if (output->directionAngle < 0)
			{
				output->directionAngle = -180 - output->directionAngle;
			}
			else
			{
				output->directionAngle = 180 - output->directionAngle;
			}
		}
	}
    return 0;
}

struct kalman bevel_kalman(struct kalman bevel)
{
	//卡尔曼滤波算法
	//假设data.yaw角永远准确，对斜面角进行卡尔曼滤波，恒定值

	if (bevel.x_k == 0)
	{
		bevel.x_k = bevel.measure_k;
		bevel.error_ek = ERROR_E;
		if (bevel.measure_k < B_ANGLE) //当斜面角度较小时，测量误差较大
		{
			bevel.error_mk = ERROR_MB;
		}
		else
		{
			bevel.error_mk = ERROR_MB-1;
		}
	}
	else
	{
		if (fabs(bevel.measure_k - bevel.x_k) > TRANSIENT) //此处为过滤条件，当检测到下方有变动的时候会停止滤波
		{
			bevel.x_k = bevel.measure_k;
			bevel.error_ek = ERROR_E;
			bevel.error_mk = ERROR_MB;
		}
		else
		{
			bevel.k = bevel.error_ek / (bevel.error_ek + bevel.error_mk);
			bevel.x_k = bevel.x_k + bevel.k * (bevel.measure_k - bevel.x_k);
			bevel.error_ek = bevel.error_ek * (1 - bevel.k);
		}
	}
	return bevel;
}



struct kalman direct_kalman(float diff, struct kalman direct)
{
	//卡尔曼滤波算法
	//假设data.yaw角一直准确，将imu增量与激光测得的增量融合
	if (direct.x_k == 0)
	{
		direct.x_k = direct.measure_k;
		direct.error_ek = ERROR_E;
		direct.error_mk = ERROR_MD;
	}
	else
	{
		if (fabs(direct.measure_k - direct.x_k) > TRANSIENT) //此处为过滤条件，当检测到下方有变动的时候会停止滤波
		{
			direct.x_k = direct.measure_k;
			direct.error_ek = ERROR_E;
			direct.error_mk = ERROR_MD;
		}
		else
		{
			direct.k = direct.error_ek / (direct.error_ek + direct.error_mk);
			direct.x_k = direct.x_k + (direct.k * (direct.measure_k - direct.x_k)) * DIR_K + diff * (1-DIR_K); //此处为传感器融合公式，需要调参
			direct.error_ek = direct.error_ek * (1 - direct.k);
		}
	}
	return direct;
}


