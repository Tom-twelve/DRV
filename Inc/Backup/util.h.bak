#ifndef __UTIL_H
#define __UTIL_H

#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "arm_math.h"
#include "tim.h"
#include "adc.h"

#define	SQRT3			(1.73205080757f)
#define ONE_BY_SQRT3    (0.57735026919f)
#define TWO_BY_SQRT3    (2.0f * 0.57735026919f)
#define SQRT3_BY_2      (0.86602540378f)

#define SQUARE(x)								((x) * (x))
#define MAX(a, b)       						(((a) > (b)) ? (a) : (b))
#define MIN(a, b)       						(((a) < (b)) ? (a) : (b))
#define NORMALIZED(val) 						((val > 1) ? 1 : (val < 0 ?  0 : val))	//归一化处理
#define RAD_TO_DEGREE(val)  					((float)val / 2.f / PI * 360.f)
#define DEGREE_TO_RAD(val)						((float)val / 360.f * 2.f * PI)
#define PULSE_TO_RAD(val)						((float)val / (float)TLE5012_ABS_MODE_RESOLUTION * 2.f * PI)

float utils_map(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t UtilBiSearchInt(const int sortedIntArr[], int find, uint32_t maxN);
float util_norm_float(float val, float minBound, float maxBound, float period);
int util_norm_int(float val, float minBound, float maxBound, float period);
void Saturation_float(float *value, float upLimit, float downLimit);
void Saturation_int(int *value, int upLimit, int downLimit);

#endif
