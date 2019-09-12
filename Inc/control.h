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

struct VolCtrl_t
{
	float BEMF;
	float CtrlVolD;
	float CtrlVolQ;
	float VolLimit;
	float CompRatio;
	float PowerAngleComp_degree;	
	float PowerAngleComp_rad;
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

struct Regulator_t
{
	float Kp;
	float Ki;
	float Kd;
	float ActualPeriod_s;
	int16_t TargetFSYNC;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURR_EXPT_LIM_Q					15.0f	//(A), 期望Iq限幅

#define CURR_INTEGRAL_ERR_LIM_D 		5.0f	//(A), Id控制器积分限幅
#define CURR_INTEGRAL_ERR_LIM_Q 		5.0f	//(A), Iq控制器积分限幅

#define SPD_INTEGRAL_ERR_LIM			(5.0f * 2 * PI)		//(rad/s)

#define CURRENT_CONTROL_KP_D			(INDUCTANCE_D * 1500.f)	//q轴电感 * 电流环带宽
#define CURRENT_CONTROL_KI_D			(PHASE_RES * 1500.f)	//相电阻 * 电流环带宽

#define CURRENT_CONTROL_KP_Q			(INDUCTANCE_Q * 1500.f)	//q轴电感 * 电流环带宽
#define CURRENT_CONTROL_KI_Q			(PHASE_RES * 1500.f)	//相电阻 * 电流环带宽

#define SPEED_CONTROL_KP	
#define SPEED_CONTROL_KI

#define POSITION_CONTROL_KP	
#define POSITION_CONTROL_KD

#define PERIOD_REGULATOR_KP 			(0.6 * 0.00596f)
#define PERIOD_REGULATOR_KI 			(5 * 0.005952380952381 * DEFAULT_CARRIER_PERIOD_s)
#define PERIOD_REGULATOR_KD 			(5 * 0.005952380952381) // 参数待调（这个初值是通过简单的计算一下，假设偏差的差分出现了1的变动，改变多少周期粗略的估计的，并非使用KD = Kd / dt)
#define PERIOD_REGULATOR_LIM			6	//载波周期调节器限幅值	

#define SPD_CURR_CTRL_MODE 				1
#define POS_SPD_CURR_CTRL_MODE 			2
#define	SPD_VOL_CTRL_MODE 				3
#define	POS_SPD_VOL_CTRL_MODE 			4

#define WORK_MODE						1
#define MEASURE_ANGLE_TABLE_MODE		2
#define MEASURE_PARAM_MODE				3

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void DriverInit(void);
void CurrentLoopInit(void);
void VoltageControllerInit(void);
void VoltageController(void);
void SpeedLoopInit(void);
void PositionLoopInit(void);
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ);
void SpeedLoop(float expectedMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ);
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed);
float VelocitySlopeGenerator(float exptVelocity);
void CurrentController(void);
void SpeedController(void);
void PositionController(void);
void PeriodRegulator(void);
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
