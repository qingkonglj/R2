#include "mycalculate.h"
#include "main.h"

#define ONE_PI   (3.14159265)

float angle_to_radian = 0.01745f;//½Ç¶È×ª»¡¶È

double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad * (1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	signed char flag = 1;
	
	while(rad > 2*ONE_PI)
	{
		rad = rad -  2*ONE_PI;
	}
	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

double my_cos(double rad)
{
	signed char flag = 1;
	rad += ONE_PI/2.0;
   
	while(rad > 2*ONE_PI)
	{
		rad = rad -  2*ONE_PI;
	}
	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}
	return my_sin(rad)*flag;
}
