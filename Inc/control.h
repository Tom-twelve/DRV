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
	float LimitCurrQ;
	float ExptCurrD;
	float ExptCurrQ;
	float CtrlVolD;
	float CtrlVolQ;
	float ErrD;
	float ErrQ;
	float IntegralErrD;
	float IntegralErrQ;
	float Kp_D;
	float Ki_D;
	float Kp_Q;
	float Ki_Q;
};

struct SpdLoop_t
{
	float ExptMecAngularSpeed_rad;		//期望机械角速度(rad/s), 弧度制
	float MaxExptMecAngularSpeed_rad;	//最大期望机械角速度(rad/s), 弧度制
	float Acceleration;		//加速度(rad/s2), 弧度制
	float Deceleration;		//减速度(rad/s2), 弧度制
	float Err;
	float IntegralErr;
	float Kp;
	float Ki;
};

struct PosLoop_t
{
	float ExptMecAngle_rad;	//目标角度, 弧度制
	float MecAngleUpperLimit_rad;		//位置环位置上限(rad), 弧度制
	float MecAngleLowerLimit_rad;		//位置环位置下限(rad), 弧度制
	float RefMecAngle_rad;		//参考机械角度(rad), 上电时置零, 弧度制
	float Err;
	float LastErr;
	float DiffErr;
	float Kp;
	float Kd;
};

struct TorqueCtrl_t
{
	float ExptTorque;
	float EleTorque;
	float MaxMecSpd;
};

struct Regulator_t
{
	float Kp;
	float Ki;
	float Kd;
	float ActualPeriod_s;
	int16_t TargetFSYNC;
};

struct MainCtrl_t
{
	int32_t ExptMecAngularSpeed_pulse;		//目标机械角速度, 脉冲
	volatile uint32_t MaxMecAngularSpeed_pulse;		//速度环最大速度, 脉冲
	uint32_t Acceleration_pulse;	//加速度, 脉冲
	uint32_t Deceleration_pulse;	//减速度, 脉冲
	int32_t ExptMecAngle_pulse;				//目标位置, 脉冲
	int32_t MecAngleUpperLimit_pulse;		//位置环位置上限, 脉冲
	int32_t MecAngleLowerLimit_pulse;		//位置环位置下限, 脉冲
	int32_t RefMecAngle_pulse;				//参考机械角度, 上电时置零, 脉冲
	float MaxTorque_Nm;						//最大扭矩, 牛米
	uint16_t PresentMecAngle_pulse;
	uint16_t LastMecAngle_pulse;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD_MULTIPLE					10	//(速度环, 位置环周期 / 电流环周期)
#define OUTER_LOOP_PERIOD				(Regulator.ActualPeriod_s * PERIOD_MULTIPLE)	//外环控制周期

#define CURR_INTEGRAL_ERR_LIM_D 		(1.0f / CurrLoop.Ki_D)	//Id积分限幅
#define CURR_INTEGRAL_ERR_LIM_Q 		(1.0f / CurrLoop.Ki_Q)	//Iq积分限幅

#define SPD_INTEGRAL_ERR_LIM			(5.0 * 2.f * PI)		//(rad/s)

#define CURRENT_CONTROL_KP_D			(INDUCTANCE_D * 1500.f)	//d轴电感 * 电流环带宽
#define CURRENT_CONTROL_KI_D			(PHASE_RES * 1500.f)	//相电阻 * 电流环带宽

#define CURRENT_CONTROL_KP_Q			(INDUCTANCE_Q * 1500.f)	//q轴电感 * 电流环带宽
#define CURRENT_CONTROL_KI_Q			(PHASE_RES * 1500.f)	//相电阻 * 电流环带宽

#define SPEED_CONTROL_KP				1.0f
#define SPEED_CONTROL_KI				1.0f

#define POSITION_CONTROL_KP				60.0f
#define POSITION_CONTROL_KD				0.1f

#define PERIOD_REGULATOR_KP 			(0.3 * 0.00596f)
#define PERIOD_REGULATOR_KI 			(0.005952380952381 * DEFAULT_CARRIER_PERIOD_s)
#define PERIOD_REGULATOR_KD 			(0.005952380952381) // 参数待调（这个初值是通过简单的计算一下，假设偏差的差分出现了1的变动，改变多少周期粗略的估计的，并非使用KD = Kd / dt)
#define PERIOD_REGULATOR_LIM			5	//载波周期调节器限幅值	

#define MC_CTRL_RESOLUTION				4096

/*控制模式, 需与driver.c同步*/
#define SPD_CURR_CTRL_MODE 				1
#define POS_SPD_CURR_CTRL_MODE 			2
#define POS_CURR_CTRL_MODE 				3
#define TORQUE_CTRL_MODE 				4
#define	SPD_VOL_CTRL_MODE 				5
#define	POS_SPD_VOL_CTRL_MODE 			6

#define WORK_MODE						1
#define MEASURE_ANGLE_TABLE_MODE		2
#define MEASURE_PARAM_MODE				3

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void DriverInit(void);
void CurrentLoopInit(void);
void SpeedLoopInit(void);
void PositionLoopInit(void);
void TorqueCtrlInit(void);
void RefAngleInit(void);
void ZeroPosSet(uint16_t posOffset);
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ);
void SpeedLoop(float expectedMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ);
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed);
void SpdCurrController(void);
void PosSpdCurrController(void);
void PosCurrController(void);
void TorqueController(void);
float VelSlopeGenerator(float exptVelocity);
void DriverCtrlModeInit(void);
void PeriodRegulator(void);
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
