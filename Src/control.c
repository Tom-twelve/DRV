/**
 ******************************************************************************
 * @file		control.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2018.1.17
 * @brief		Algorithm of control
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "control.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */

struct CurrentLoop_t CurrentLoop;
struct SpeedLoop_t SpeedLoop;
struct PositionLoop_t PositionLoop;

/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern struct MotorStaticParameter_t MotorStaticParameter;
extern struct CoordinateTransformation_t CoordinateTransformation;
extern struct Encoder_t Encoder;
/* USER CODE END EV */

/* USER CODE BEGIN */

void MotorEnable(void)
{
	/*使能PWM输出*/
	PWM_IT_CMD(ENABLE,ENABLE);
	
	/*设定控制模式*/
	MotorStaticParameter.ControlMode = CurrentControlMode;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrentLoop.ExpectedCurrentD = 0.f;
	
	switch(MotorStaticParameter.ControlMode)
	{
		case VoltageControlMode :		/*测试用*/
		
										break;
		
		case CurrentControlMode : 		/*设定q轴电流*/
										CurrentLoop.ExpectedCurrentQ = 10.f;

										/*设定电流环PI参数*/
										CurrentLoop.Kp_D = 1.0f;
										
										CurrentLoop.Ki_D = 1.0f;
										
										CurrentLoop.Kp_Q = 3.0f;

										CurrentLoop.Ki_Q = 1.5f;
		
										break;
		
		case SpeedControlMode : 		/*设定角速度(°/s)*/
										SpeedLoop.ExpectedMechanicalAngularSpeed = 100.f * 360.f;	//degree per second
										
										/*设定加速度*/
										SpeedLoop.Acceleration = 25.f * 360.f;	//degree per quadratic seconds
		
										/*设定电流环PI参数*/
										CurrentLoop.Kp_D = 0.2f;
										
										CurrentLoop.Ki_D = 0.1f;
										
										CurrentLoop.Kp_Q = 0.1f;

										CurrentLoop.Ki_Q = 0.05f;
		
										/*设定速度环PI参数*/
										SpeedLoop.Kp = 0.006f;
										
										SpeedLoop.Ki = 0.001f;
		
										break;
		
		case PositionControlMode :		/*设定电流环PI参数*/
										CurrentLoop.Kp_D = 0.1f;
										
										CurrentLoop.Ki_D = 0.035f;
										
										CurrentLoop.Kp_Q = 0.10f;

										CurrentLoop.Ki_Q = 0.05f;
		
										/*设定位置环PI参数*/
										PositionLoop.Kp = 0.0001f;
										
										PositionLoop.Ki = 0.f;
		
										break;
	}
}

   /**
   * @brief		Current control loop
   */
void CurrentControlLoop(float expectedCurrentD, float expectedCurrentQ, float realityCurrentD, float realityCurrentQ, float *controlCurrentD, float *controlCurrentQ)
{
	float errorD = 0;
	float errorQ = 0;
	static float integralErrorD = 0;
	static float integralErrorQ = 0;
	
	errorD = expectedCurrentD - realityCurrentD;
	errorQ = expectedCurrentQ - realityCurrentQ;
	
	*controlCurrentD = CurrentLoop.Kp_D * errorD + CurrentLoop.Ki_D * integralErrorD;
	*controlCurrentQ = CurrentLoop.Kp_Q * errorQ + CurrentLoop.Ki_Q * integralErrorQ;
	
	integralErrorD += errorD * CarrierPeriod_s;
	integralErrorQ += errorQ * CarrierPeriod_s;
	
	if(integralErrorD >= CurrentControlLoopIntegralErrorLimit_D)
	{
		integralErrorD = CurrentControlLoopIntegralErrorLimit_D;
	}
	
	else if(integralErrorD <= -CurrentControlLoopIntegralErrorLimit_D)
	{
		integralErrorD = -CurrentControlLoopIntegralErrorLimit_D;
	}
	
	if(integralErrorQ >= CurrentControlLoopIntegralErrorLimit_Q)
	{
		integralErrorQ = CurrentControlLoopIntegralErrorLimit_Q;
	}
	
	else if(integralErrorQ <= -CurrentControlLoopIntegralErrorLimit_Q)
	{
		integralErrorQ = -CurrentControlLoopIntegralErrorLimit_Q;
	}
}

   /**
   * @brief		Speed control loop
   */
void SpeedControlLoop(float expectedMechanicalAngularSpeed, float realityMechanicalAngularSpeed, float *controlCurrentQ)
{
	float error = 0;
	static float integralError = 0;
	
	error = expectedMechanicalAngularSpeed - realityMechanicalAngularSpeed;
	
	*controlCurrentQ = SpeedLoop.Kp * error + SpeedLoop.Ki * integralError;
	
	integralError += error * CarrierPeriod_s;
	
	if(integralError >= SpeedControlLoopIntegralErrorLimit)
	{
		integralError = SpeedControlLoopIntegralErrorLimit;
	}
	
	else if(integralError <= -SpeedControlLoopIntegralErrorLimit)
	{
		integralError = -SpeedControlLoopIntegralErrorLimit;
	}
}

   /**
   * @brief		Position control loop
   */
void PositionControlLoop(float expectedMechanicalAngle, float realityMechanicalAngle, float *controlCurrentQ)
{
	float error = 0;
	static float integralError = 0;
	static float lastError = 0;
	
	error = expectedMechanicalAngle - realityMechanicalAngle;
	
	*controlCurrentQ = PositionLoop.Kp * error + PositionLoop.Ki * integralError + PositionLoop.Kd * (error - lastError);
	
	integralError += error * CarrierPeriod_s;
	
	lastError = error;
	
	if(integralError >= PositionControlLoopIntegralErrorLimit)
	{
		integralError = PositionControlLoopIntegralErrorLimit;
	}
	
	else if(integralError <= -PositionControlLoopIntegralErrorLimit)
	{
		integralError = -PositionControlLoopIntegralErrorLimit;
	}
}

   /**
   * @brief		Velocity slope  generator
   */
float VelocitySlopeGenerator(float expectedVelocity)
{
	static float velocityProcessVolume = 0.0f;
	static float velocityStepValue = 0;
	
	velocityStepValue = SpeedLoop.Acceleration * CarrierPeriod_s;

	if (velocityProcessVolume < (expectedVelocity - velocityStepValue))
	{
		velocityProcessVolume += velocityStepValue;
	}
	else if (velocityProcessVolume > (expectedVelocity + velocityStepValue))
	{
		velocityProcessVolume -= velocityStepValue;
	}
	else
	{
		velocityProcessVolume = expectedVelocity;
	}
	
	return velocityProcessVolume;
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
