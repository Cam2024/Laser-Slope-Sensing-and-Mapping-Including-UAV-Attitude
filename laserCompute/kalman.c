#include"header.h"

//kalman_t kalman(kalman_t angle)
//{
//	if (angle.X == 0)
//	{
//		angle.X = angle.measurement;
//		angle.P = EError;
//		angle.R = MError;
//	}
//	else
//	{
//		angle.K = angle.P / (angle.P + angle.R);
//		angle.X = angle.X + angle.K * (angle.measurement - angle.X);
//		angle.P = angle.P * (1 - angle.K);
//	}
//	return angle;
//}

kalman_t kalman(kalman_t angle)
{
	if (angle.X == 0)
	{
		angle.X = angle.measurement;
		angle.P = 1.5;
		angle.R = 5;
	}
	else
	{
		if (fabs(angle.X - angle.measurement) > 300)
		{
			if (angle.X < 0)
			{
				angle.measurement = angle.measurement - 360;
			}
			else
			{
				angle.measurement = angle.measurement + 360;
			}
		}
		angle.K = angle.P / (angle.P + angle.R);
		angle.X = angle.X + angle.K * (angle.measurement - angle.X);
		angle.P = angle.P * (1 - angle.K);
		if (angle.X >= 180)
		{
			angle.X = angle.X - 360;
		}
		if (angle.X < -180)
		{
			angle.X = angle.X + 360;
		}
	}
	return angle;
}
