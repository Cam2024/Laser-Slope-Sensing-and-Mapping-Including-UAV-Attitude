#include "header.h"

/**
  * @brief  根据四点测距计算斜面参数
  * @param  dis0 距离值0
  * @param  dis1 距离值1
  * @param  dis2 距离值2
  * @param  dis3 距离值3  
  * @param  laserOutput 输出的解算数据
  * @retval 返回0，数据计算正常，返回1，数据偏差过大
  */
angle_t LaserCalculation(input_t inputData)
{
	float point[4][3] = { 0 };
	float vectorA[3] = { 0 };
	float vectorB[3] = { 0 };
	float vectorC[3] = { 0 };
	float vectorD[3] = { 0 };
    angle_t angle;
    
	//point 0
	point[0][0] = length;
	point[0][1] = length;
	point[0][2] = -inputData.dis[0];
	//point 1
	point[1][0] = length;
	point[1][1] = -length;
	point[1][2] = -inputData.dis[1];
	//point 2
	point[2][0] = -length;
	point[2][1] = -length;
	point[2][2] = -inputData.dis[2];
	//point 3
	point[3][0] = -length;
	point[3][1] = length;
	point[3][2] = -inputData.dis[3];

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
		vectorD[1] = cos(-inputData.roll * pi / 180) * vectorC[1] - sin(-inputData.roll * pi / 180) * vectorC[2];
		vectorD[2] = sin(-inputData.roll * pi / 180) * vectorC[1] + cos(-inputData.roll * pi / 180) * vectorC[2];
		//y轴
		vectorD[0] = cos(-inputData.pitch * pi / 180) * vectorD[0] + sin(-inputData.pitch * pi / 180) * vectorD[2];
		vectorD[1] = vectorD[1];
		vectorD[2] = -sin(-inputData.pitch * pi / 180) * vectorD[0] + cos(-inputData.pitch * pi / 180) * vectorD[2];
 
		vectorC[0] = vectorD[0];
		vectorC[1] = vectorD[1];
		vectorC[2] = vectorD[2];

		//angle
		angle.bevelAngle[count] = (180 * acos((fabs(vectorC[2])) / (sqrt(pow(vectorC[0], 2) + pow(vectorC[1], 2) + pow(vectorC[2], 2))))) / pi;
		angle.directionAngle[count] = (180 * acos((fabs(vectorC[1])) / (sqrt(pow(vectorC[0], 2) + pow(vectorC[1], 2))))) / pi;

		// 以下部分是为了修正转向角度从-179到180
		if (vectorC[0] < 0)
		{
			if (vectorC[1] < 0)
			{
				angle.directionAngle[count] = angle.directionAngle[count] - 180;
			}
			else
			{
				angle.directionAngle[count] = -angle.directionAngle[count];
			}
		}
		else
		{
			if (vectorC[1] < 0)
			{
				angle.directionAngle[count] = 180 - angle.directionAngle[count];
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
	return angle;
}

