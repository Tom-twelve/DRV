#include "util.h"
	
/* 等比例放大(映射) */
float utils_map(float x, float in_min, float in_max, float out_min, float out_max) 
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) 
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief 二分查找
 * @note 如果数组中有被查找的数字，则返回对应的元素的序号
 *       如果数组中不存在被查找的数字，则返回与他相邻的偏小的元素的序号
 * @retval int型
 */
uint32_t UtilBiSearchInt(const int sortedIntArr[], int find, uint32_t maxN)
{
	uint32_t low, mid, upper;
	low = 0;
	upper = maxN - 1;
	while (low <= upper)
	{
		mid = (low + upper) >> 1;//相当于除以2
		if (sortedIntArr[mid] < find)
			low = mid + 1;
		else if (sortedIntArr[mid] > find)
			upper = mid - 1;
		else
			return mid;
	}
	// 可以看到退出上循环后 upper 一定小于 low
	// It can be seen that upper must be smaller than low when it quit the loop
	return upper;
}

// Author :dzc
float util_norm_float(float val, float minBound, float maxBound, float period)
{
	while(val < minBound)
	{
		val += period;
	}
	while(val >= maxBound)
	{
		val -= period;
	}
	return val;
}

// Author :oliver
int util_norm_int(float val, float minBound, float maxBound, float period)
{
	while(val < minBound)
	{
		val += period;
	}
	while(val >= maxBound)
	{
		val -= period;
	}
	return val;
}

void Saturation_float(float *value, float upLimit, float downLimit)
{	
	if(*value >= upLimit)
	{
		*value	= upLimit;
	}
	else if(*value <= downLimit)
	{
		*value = downLimit;
	}
}

void Saturation_int(int *value, int upLimit, int downLimit)
{	
	if(*value >= upLimit)
	{
		*value	= upLimit;
	}
	else if(*value <= downLimit)
	{
		*value = downLimit;
	}
}
/**
 * @brief 最小二乘法
 * @note 
 * @retval 
 */
void LeastSquare(float *x, float *y,int size,float *a)
{
	float sum_x2 = 0.0;
	float sum_y  = 0.0;
	float sum_x  = 0.0;
	float sum_xy = 0.0;

	for (int i = 0; i < size;i++) 
	{
		sum_x2 += x[i]*x[i];
		sum_y  += y[i];
		sum_x  += x[i];
		sum_xy += x[i]*y[i];
	}    

	*a = (size*sum_xy - sum_x*sum_y)*2000.0f/(size*sum_x2 - sum_x*sum_x);
//	*b = (sum_x2*sum_y - sum_x*sum_xy)/(size*sum_x2-sum_x*sum_x);
}