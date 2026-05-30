#ifndef __ALGORISM_H
#define __ALGORISM_H	

#include <stdint.h>

#define square(x) x*x
#define cube(x) x*x*x
#define fsgn(x) ( (fabs(x)<1e-6) ? 0:(0<x)-(x<0) )


/**
 * \brief 平滑滤波器结构体 
 */
typedef struct
{
    float alpha; 
    float last, current;
}SmoothFilter;
typedef struct
{
	float last[3];
}AverageFilter;

typedef struct
{
	float x[30];
	float y[30];
	uint16_t num;
	float a;
	float b;
	float valid_num;
	uint8_t count;
}leastSquareLinear;

void SmoothFilterInitialize(SmoothFilter* filter, float alpha);
float SmoothFilterUpdate(SmoothFilter* filter, float input);
void AverageFilterInitialize(AverageFilter*filter);
float AverageFilterUpdate(AverageFilter*filter, float input);


float AngleLimit(float angle, float limit_min, float limit_max);
float AbsLimiter(float val, float max);
float DoubleEdgeLimiter(float val, float min, float max);
float InvSqrt(float x);
int Sign(float x);

void leastSquareLinearFit(leastSquareLinear* data);
#endif
