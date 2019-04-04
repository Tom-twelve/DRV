#include "util.h"
	
_Bool EvenParityCheck(uint32_t data)
{
	_Bool a = 1;
	while(data)
	{
		a=!a;//改为 count++;可以计算该数二进制中包含1的个数
		data&=(data-1);
	}
	return a;
}


int utils_truncate_number_abs(float *number, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}

/**
 * Make sure that -pi <= angle < pi,
 *
 * TODO: Maybe use fmodf instead?
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_rad(float *angle) {
	// *angle = fmodf(*angle, 2.0*M_PI);
	while (*angle < -M_PI) {
		*angle += 2.0 * M_PI;
	}

	while (*angle >  M_PI) {
		*angle -= 2.0 * M_PI;
	}
}

/* 等比例放大(映射) */
float utils_map(float x, float in_min, float in_max, float out_min, float out_max) 
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) 
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Avg_filter_t UtilAvgFilter(Avg_filter_t valNew,Avg_filter_t arr[], uint32_t *index, Avg_filter_t *pSum)
{
//	if (*pSum > arr[*index])//无符号整形时的处理
		*pSum -= arr[*index];
//	else
//		*pSum = 0u;
	*pSum       += valNew;
	arr[*index] = valNew;
	(*index)++;
	*index = ((*index)) < Avg_NUM ? *index : 0u;

	return (*pSum) / Avg_NUM;
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

/**
 * @brief  float形增量式PI调节器
 * @param  None
 * @retval deltaU 控制量增量
 */
float UtilPI_Control(arm_pid_instance_f32 *S, float err)
{
	// 移动偏差队列
	S->state[2] = S->state[1]; //外部使用D调节时可能会用到
	S->state[1] = S->state[0];
	// 填入新偏差
	S->state[0] = err;
	// 计算控制量增量
  return S->A0 * S->state[0] + S->A1 * S->state[1];
}


/**
 * @brief  float形增量式PID调节器
 * @param  None
 * @retval deltaU 控制量增量
 */
float UtilPID_Control(arm_pid_instance_f32 *S, float err)
{
	// 移动偏差队列
	S->state[2] = S->state[1];
	S->state[1] = S->state[0];
	// 填入新偏差
	S->state[0] = err;
	// 计算控制量增量
  return S->A0 * S->state[0] + S->A1 * S->state[1] + S->A2 * S->state[2];
}

/**
 * @brief  Arcsine
 */
float arcsine(float value)
{
	float angle_rad = 0;
	
	//利用四阶泰勒级数计算反正弦函数值, 保证速度, 精度只能保证小数点后三位的准确性, 并且value应在(-0.5 ~ 0.5)之间, 超出此范围将不能保证精度
	angle_rad = value+value*value*value/6 + 3*value*value*value*value*value/40 + 5*value*value*value*value*value*value*value/112;	
	
	return angle_rad;
}

float sqrt_DSP(float inputValue)
{
	float outputValue = 0;
	
	arm_sqrt_f32(inputValue, &outputValue);
	
	return outputValue;
}

float AverageFilter(float inputData)
{
	const uint8_t FilterOrder = 6;
	static float array[FilterOrder] = {0};
	static uint8_t pos = 0;
	static float data = 0.0f;
	static float sum = 0.0f;
	static float Avg = 0.0f;
	float old = array[pos];
		
	data = inputData;
		
	array[pos] = data;
		
	sum = (sum - old) + data;
		
	Avg = sum / FilterOrder;

	pos = (pos+1) % FilterOrder;

	return Avg;
}
	
/*Keil读取数据*/
int TempDataArray[1000] = {0};
int DataReadyFlag = 0;
	
void GetData(int32_t data)
{
	static uint16_t i = 0;
	static uint16_t j = 0;
	
	j++;
	
	if(j >= 50000)
	{
		j = 50000;
		
		if(i < 1000)
		{
			TempDataArray[i] = data;
	
			i++;
		}	
		
		if(i >= 1000)
		{
			i = 1000;
			
			PWM_IT_CMD(DISABLE, DISABLE);
			
			DataReadyFlag = 1;
			
			ADC_CMD(DISABLE);
		}
	}
}

void SendData(void)
{
	if(DataReadyFlag)
	{
		DataReadyFlag = 0;
		
		for(uint16_t k = 0; k < 1000; k++)
		{
			UART_Transmit_DMA("%d\r\n",(int)(TempDataArray[k]));
			for(uint16_t m = 0; m < 1000; m++)
			{
			
			}
		}
	}
}
