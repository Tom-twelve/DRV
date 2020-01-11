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
#include "observer.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */
struct CurrLoop_t
{
	float LimitCurrQ;
	float LimitVolD;
	float LimitVolQ;
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
	float ExptMecAngularSpeed_rad;		//期望机械角速度(rad/s)
	float MaxExptMecAngularSpeed_rad;	//最大期望角速度(rad/s)，用于限幅
	float Acceleration;					//加速度(rad/s2)
	float Deceleration;					//减速度(rad/s2)
	float Err;
	float IntegralErr;
	float Kp;
	float Ki;
};

struct PosLoop_t
{
	float ExptMecAngle_rad;			//期望机械角度(rad)
	float MecAngleUpperLimit_rad;	//机械角度上限(rad)
	float MecAngleLowerLimit_rad;	//机械角度下限(rad)
	float RefMecAngle_rad;			//参考机械角度(rad), 主控的参考零位置
	float Err;
	float LastErr;
	float DiffErr;
	float Kp;
	float Kd;
};

struct TorqueCtrl_t
{
	float ExptTorque_Nm;	//期望转矩(脉冲)
	float EleTorque_Nm;		//电磁转矩(脉冲)
	float MaxTorque_Nm;		//最大转矩(脉冲)
	float MaxMecSpd_rad;	//最大机械角速度(脉冲), 该转速限幅仅用于转矩控制模式
};

struct MainCtrl_t
{
	int32_t ExptMecAngularSpeed_pulse;			//期望机械角速度(脉冲)
	uint32_t MaxMecAngularSpeed_pulse;			//最大机械角速度(脉冲), 用于限幅
	uint32_t Acceleration_pulse;				//加速度(脉冲)
	uint32_t Deceleration_pulse;				//加速度(脉冲)
	int32_t ExptMecAngle_pulse;					//期望机械角度(脉冲)
	int32_t MecAngleUpperLimit_pulse;			//机械角度上限(脉冲), 用于限幅
	int32_t MecAngleLowerLimit_pulse;			//机械角度下限(脉冲), 用于限幅
	int32_t RefMecAngle_pulse;					//参考机械角度(脉冲), 主控的参考零位置
	uint16_t PresentMecAngle_pulse;
	uint16_t LastMecAngle_pulse;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PERIOD_MULTIPLE					10	//(外环周期 / 电流环周期)
#define OUTER_LOOP_PERIOD				(DEFAULT_CARRIER_PERIOD_s * PERIOD_MULTIPLE)	//外环周期

#define CURR_INTEGRAL_ERR_LIM_D 		(12.0f / CurrLoop.Ki_D)	//Id积分限幅
#define CURR_INTEGRAL_ERR_LIM_Q 		(6.0f / CurrLoop.Ki_Q)	//Iq积分限幅

#define SPD_INTEGRAL_ERR_LIM			(15.0 * 2.f * PI)

#define CURRENT_CONTROL_KP_D			(INDUCTANCE_D * 1500.f)	//d轴电流环采用的传统PI控制器, Kp = d轴电感 * 电流环带宽
#define CURRENT_CONTROL_KI_D			(PHASE_RES * 1500.f)	//d轴电流环采用的传统PI控制器, Ki = 相电阻 * 电流环带宽

#define CURRENT_CONTROL_KP_Q			(INDUCTANCE_Q * 2500.f)	//q轴电流环采用基于模型改进的PI控制器, Kp = q轴电感 * 电流环带宽
#define CURRENT_CONTROL_KI_Q			(PHASE_RES * 0.f)		//q轴电流环采用基于模型改进的PI控制器, Ki待调节

#define SPEED_CONTROL_KP				1.0f
#define SPEED_CONTROL_KI				1.0f

#define POSITION_CONTROL_KP				60.0f
#define POSITION_CONTROL_KD				0.1f

#define MC_CTRL_RESOLUTION				4096	//变量分辨率

/*控制模式宏定义, 需与主控同步*/
#define SPD_CURR_CTRL_MODE 				1
#define POS_SPD_CURR_CTRL_MODE 			2
#define POS_CURR_CTRL_MODE 				3
#define TORQUE_CTRL_MODE 				4
#define	SPD_VOL_CTRL_MODE 				5
#define	POS_SPD_VOL_CTRL_MODE 			6

#define WORK_MODE						1
#define MEASURE_ANGLE_TABLE_MODE		2
#define MEASURE_PARAM_MODE				3
#define MEASURE_INERTIA_MODE			4

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
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
