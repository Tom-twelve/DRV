/**
 ******************************************************************************
 * @file		control.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		The header file of control.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "MotorConfig.h"
#include "math.h"
#include "foc.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */
struct CurrLoop_t
{
	float ExptCurrD;
	float ExptCurrQ;
	float CtrlVolD;
	float CtrlVolQ;
	float Kp_D;
	float Ki_D;
	float Kp_Q;
	float Ki_Q;
};

struct SpdLoop_t
{
	float ExptMecAngularSpeed;	//期望机械角速度(rad/s)
	float Acceleration;		//加速度(rad/s2)
	float Kp;
	float Ki;
};

struct PosLoop_t
{
	float ExptMecAngle;	//degree
	float Kp;
	float Kd;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURR_INTEGRAL_ERR_LIM_D 		2.5f	//(A)

#define CURR_INTEGRAL_ERR_LIM_Q 		2.5f	//(A)

#define SPD_INTEGRAL_ERR_LIM			(10.0f * 2 * PI)		//(rad/s)

#define CURRENT_CONTROL_KP_D			(INDUCTANCE_D * 500.f)		//d轴电感 * 电流环截止频率
#define CURRENT_CONTROL_KI_D			(PHASE_RES * 500.f)	//相电阻 * 电流环截止频率

#define CURRENT_CONTROL_KP_Q			(INDUCTANCE_Q * 500.f)		//q轴电感 * 电流环截止频率
#define CURRENT_CONTROL_KI_Q			(PHASE_RES * 500.f)	//相电阻 * 电流环截止频率

#define SPEED_CONTROL_KP	
#define SPEED_CONTROL_KI

#define POSITION_CONTROL_KP	
#define POSITION_CONTROL_KD

#define	VoltageControlMode 			0
#define CurrentControlMode 			1
#define SpeedControlMode 			2
#define PositionControlMode 		3

#define WorkMode					1
#define TestMode 					2

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void MotorEnable(void);
void CurrentLoop(float exptCurrD, float exptCurrQ, float realityCurrD, float realityCurrQ, float *ctrlVolD, float *ctrlVolQ);
void SpeedLoop(float expectedMecAngularSpeed, float realityMecAngularSpeed, float *controlCurrentQ);
void PositionLoop(float exptMecAngle, float realityMecAngle, float *controlAngularSpeed);
float VelocitySlopeGenerator(float exptVelocity);
void CurrentController(void);
void SpeedController(void);
void PositionController(void);
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
