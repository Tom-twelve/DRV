#ifndef __UTIL_H
#define __UTIL_H

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"

#define M_PI			(3.14159265358979323846)
#define	SQRT3			(1.73205080757f)
#define ONE_BY_SQRT3    (0.57735026919f)
#define TWO_BY_SQRT3    (2.0f * 0.57735026919f)
#define SQRT3_BY_2      (0.86602540378f)

// Squared
#define Square(x)       ((x) * (x))
#define MAX(a, b)                               (((a) > (b)) ? (a) : (b))
#define MIN(a, b)                               (((a) < (b)) ? (a) : (b))
#define SATURATION(val, boundMin, boundMax)     MIN(MAX((val), (boundMin)), (boundMax))
// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN(x)     ((x < 0) ? -1 : 1)
#define UTILS_IS_NAN(x)		((x) != (x))
#define UTILS_NAN_ZERO(x)	(x = (UTILS_IS_NAN(x) ? 0.0 : x))

_Bool EvenParityCheck(uint32_t data);
int utils_truncate_number_abs(float *number, float max);
void utils_norm_angle_rad(float *angle);
float utils_map(float x, float in_min, float in_max, float out_min, float out_max);
#define Avg_NUM     3
typedef float Avg_filter_t;
Avg_filter_t UtilAvgFilter(Avg_filter_t valNew,Avg_filter_t arr[], uint32_t *index, Avg_filter_t *pSum);
float util_norm_float(float val, float minBound, float maxBound, float period);
int util_norm_int(float val, float minBound, float maxBound, float period);
uint32_t UtilBiSearchInt(const int sortedIntArr[], int find, uint32_t maxN);

float UtilPI_Control(arm_pid_instance_f32 *S, float err);
float UtilPID_Control(arm_pid_instance_f32 *S, float err);
float arcsine(float value);
float sqrt_DSP(float inputValue);
#endif
