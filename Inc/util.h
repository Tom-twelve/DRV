#ifndef __UTIL_H
#define __UTIL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "arm_math.h"

#define	SQRT3			(1.73205080757f)
#define ONE_BY_SQRT3    (0.57735026919f)
#define TWO_BY_SQRT3    (2.0f * 0.57735026919f)
#define SQRT3_BY_2      (0.86602540378f)

#define SQUARE(x)								((x) * (x))
#define MAX(a, b)       						(((a) > (b)) ? (a) : (b))
#define MIN(a, b)       						(((a) < (b)) ? (a) : (b))
#define NORMALIZED(val) 						((val > 1) ? 1 : (val < 0 ?  0 : val))
#define RAD_TO_DEGREE(val)  					((float)val / 2.f / PI * 360.f)
#define DEGREE_TO_RAD(val)						((float)val / 360.f * 2.f * PI)
#define DRV_PULSE_TO_RAD(val)					((float)val / (float)TLE5012_ABS_MODE_RESOLUTION * 2.f * PI)
#define RAD_TO_MC_PULSE(val)					((int32_t)((float)val / 2.f / PI * (float)MC_CTRL_RESOLUTION))
#define DRV_PULSE_TO_MC_PULSE(val)				((int32_t)((float)val / (float)TLE5012_ABS_MODE_RESOLUTION * (float)MC_CTRL_RESOLUTION))
#define MC_PULSE_TO_DRV_PULSE(val)				((int32_t)((float)val / ((float)MC_CTRL_RESOLUTION) * (float)TLE5012_ABS_MODE_RESOLUTION))

float utils_map(float x, float in_min, float in_max, float out_min, float out_max);
uint32_t UtilBiSearchInt(const int sortedIntArr[], int find, uint32_t maxN);
float util_norm_float(float val, float minBound, float maxBound, float period);
int util_norm_int(float val, float minBound, float maxBound, float period);
typedef float average_filter_t;
average_filter_t UtilAverageFilter(average_filter_t valNew,average_filter_t arr[], uint32_t *index, average_filter_t *pSum,uint16_t averageNum);
void Saturation_float(float *value, float upLimit, float downLimit);
void Saturation_int(int *value, int upLimit, int downLimit);
void LeastSquare(float *x, float *y,int size, float *a);
#endif
