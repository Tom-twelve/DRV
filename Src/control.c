/**
 ******************************************************************************
 * @file		control.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
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
	MotorStaticParameter.ControlMode = VoltageControlMode;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrentLoop.ExpectedCurrentD = 0.f;
	
	switch(MotorStaticParameter.ControlMode)
	{
		case VoltageControlMode :		/*测试用*/
										MotorStaticParameter.PowerAngleCompensation_degree = 16.f;
		
										break;
		
		case CurrentControlMode : 		/*设定q轴电流*/
										CurrentLoop.ExpectedCurrentQ = 5.f;
		
										/*设定电流环PI参数*/
										CurrentLoop.Kp_D = 0.1f;
										
										CurrentLoop.Ki_D = 0.1f;
										
										CurrentLoop.Kp_Q = 0.4f;

										CurrentLoop.Ki_Q = 0.1f;
		
										break;
		
		case SpeedControlMode : 		/*设定角速度(°/s)*/
										SpeedLoop.ExpectedMecAngularSpeed = 100.f * 2 * PI;	//degree per second
										
										/*设定加速度*/
										SpeedLoop.Acceleration = 25.f * 2 * PI;	//degree per quadratic seconds
		
										/*设定电流环PI参数*/
										CurrentLoop.Kp_D = CURRENT_CONTROL_KP_D;
										
										CurrentLoop.Ki_D = CURRENT_CONTROL_KI_D;
										
										CurrentLoop.Kp_Q = CURRENT_CONTROL_KP_Q;

										CurrentLoop.Ki_Q = CURRENT_CONTROL_KI_Q;
		
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
										
										PositionLoop.Kd = 0.f;
		
										break;
	}
}

   /**
   * @brief		Current control loop
   */
void CurrentLoopController(float expectedCurrentD, float expectedCurrentQ, float realityCurrentD, float realityCurrentQ, float *controlVoltageD, float *controlVoltageQ)
{
	float errorD = 0;
	float errorQ = 0;
	float controlCurrentD = 0;
	float controlCurrentQ = 0;
	static float integralErrorD = 0;
	static float integralErrorQ = 0;
	
	errorD = expectedCurrentD - realityCurrentD;
	errorQ = expectedCurrentQ - realityCurrentQ;
	
	/*PI控制器*/
	controlCurrentD = CurrentLoop.Kp_D * errorD + CurrentLoop.Ki_D * integralErrorD;
	controlCurrentQ = CurrentLoop.Kp_Q * errorQ + CurrentLoop.Ki_Q * integralErrorQ;
	
	integralErrorD += errorD * CarrierPeriod_s;
	integralErrorQ += errorQ * CarrierPeriod_s;
	
	/*积分限幅*/
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
	
	/*反电动势补偿*/
	
	*controlVoltageD = 0;
	*controlVoltageQ = controlCurrentQ + Encoder.AvgEleAngularSpeed_rad * RotatorFluxLinkage;
}

   /**
   * @brief		Speed control loop
   */
void SpeedLoopController(float expectedMecAngularSpeed, float realityMecAngularSpeed, float *controlCurrentQ)
{
	float error = 0;
	static float integralError = 0;
	
	error = expectedMecAngularSpeed - realityMecAngularSpeed;
	
	*controlCurrentQ = SpeedLoop.Kp * error + SpeedLoop.Ki * integralError;
	
	integralError += error * CarrierPeriod_s;
	
	/*积分限幅*/
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
void PositionLoopController(float expectedMecAngle, float realityMecAngle, float *controlCurrentQ)
{
	float error = 0;
	static float lastError = 0;
	
	error = expectedMecAngle - realityMecAngle;
	
	*controlCurrentQ = PositionLoop.Kp * error + PositionLoop.Kd * (error - lastError);
	
	lastError = error;
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
