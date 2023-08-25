#include "header.h"


/**
  * @brief  根据四点测距计算斜面参数
  * @param[in]   输入的滤波角度数据
  * @param[out]  *out 输出的解算数据
  * @retval 返回0，数据计算正常，返回1，数据偏差过大
  */
uint8_t avg(angle_t angle, output_t *out)
{
	float bevelAvg = 0;
	float dirAvg = 0;
	float sum = 0;
	float var = 0;
	float std = 0;

	//斜面角平均
	for (int i = 0; i < 4; i++)
	{
		bevelAvg = bevelAvg + angle.bevelAngle[i];
	}
	bevelAvg = bevelAvg / 4;
    if(bevelAvg<5)
    {
        out->directionAngle=0;
        out->bevelAngle = bevelAvg;
        return 0;
    }
	for (int i = 0; i < 4; i++)
	{
		var = var + pow(angle.bevelAngle[i] - bevelAvg, 2);
	}
	var = var / 4;
	std = sqrt(var);
	if (std < MaxBevel) //斜面角的标准差判断是否偏差过大
	{
		out->bevelAngle = bevelAvg;
		var = 0;
		std = 0;
		//转向角平均
		for (int i = 0; i < 4; i++)
		{
			dirAvg = dirAvg + angle.directionAngle[i];
		}
		dirAvg = dirAvg / 4;
		for (int i = 0; i < 4; i++)
		{
			var = var + pow(angle.directionAngle[i] - dirAvg, 2);
		}
		var = var / 4;
        
		std = sqrt(var);
        
		//-179与179平均
		if (std > STD_BIAS)
		{
			for (int i = 0; i < 4; i++)
			{
				if (angle.directionAngle[i] < 0)
				{
					angle.directionAngle[i] = -180 - angle.directionAngle[i];
				}
				else
				{
					angle.directionAngle[i] = 180 - angle.directionAngle[i];
				}
				sum = sum + angle.directionAngle[i];
			}
			out->directionAngle = sum / 4;
			if (out->directionAngle < 0)
			{
				out->directionAngle = -180 - out->directionAngle;
			}
			else
			{
				out->directionAngle = 180 - out->directionAngle;
			}
		}
		else
		{
			out->directionAngle = dirAvg;
		}
        return 0;
	}
	else
	{
		out->bevelAngle = 0;
		out->directionAngle = 0;
        return 1;
	}
}
