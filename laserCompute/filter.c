#include"header.h"

float filter(float* num, float now)
{
	float sum = 0;
	float avg = 0;
	for (int i = 0; i < filterNUM - 1; i++)
	{
		num[i] = num[i + 1];
	}
	num[filterNUM - 1] = now;
	if (num[0] == 0)
	{
		avg = now;
	}
	else
	{
		for (int i = 0; i < filterNUM; i++)
		{
			sum = sum + num[i];
		}
		avg = sum / filterNUM;
	}
	return avg;
}
