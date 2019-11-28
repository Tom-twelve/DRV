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

struct CurrLoop_t CurrLoop;
struct SpdLoop_t SpdLoop;
struct PosLoop_t PosLoop;
struct TorqueCtrl_t TorqueCtrl;
struct MainCtrl_t MainCtrl;
/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern struct CoordTrans_t CoordTrans;
extern struct PosSensor_t PosSensor;
extern struct Driver_t Driver;
/* USER CODE END EV */

/* USER CODE BEGIN */

void DriverInit(void)
{
	/*使能PWM输出*/
	PWM_IT_CMD(ENABLE,ENABLE);
	
	#if ROBOT_ID == PASS_ROBOT
		#if CAN_ID_NUM == 1
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 16037;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 100.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 2
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26339;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 3
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 21030;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 4
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 5
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 6
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 7
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 350.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 8
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26173;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 50.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 9
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26173;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 50.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 10
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26173;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 50.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 11
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26173;
			CurrLoop.LimitCurrQ = 50.f;
			SpdLoop.ExptMecAngularSpeed_rad = 50.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#endif
	#endif
	#if ROBOT_ID == TRY_ROBOT
		#if CAN_ID_NUM == 1
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 2
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 26339;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 3
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 21030;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 4
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 5
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#elif CAN_ID_NUM == 6
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12190;
			CurrLoop.LimitCurrQ = 200.f;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;
			SpdLoop.Kp = SPEED_CONTROL_KP * 1.0f;	
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.0f;
		#endif
	#endif
}

 /**
   * @brief  电流环参数初始化
   */
void CurrentLoopInit(void)
{
	/*设定电流环PI参数, q轴电流控制器采用简化电机模型+I调节, 不同于传统PI控制器, 故I参数需人工调整*/		
	CurrLoop.Kp_D = CURRENT_CONTROL_KP_D;
	CurrLoop.Ki_D = CURRENT_CONTROL_KI_D;						
	CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q;
	
	/*设定编码器延迟补偿系数*/
	PosSensor.CompRatio_forward = 3.2f;
	PosSensor.CompRatio_reverse = 3.2f;
	
	/*设定最大Iq*/
	CurrLoop.LimitCurrQ = 20.f;
}

 /**
   * @brief  速度环参数初始化
   */
void SpeedLoopInit(void)
{
	if(Driver.ControlMode == SPD_CURR_CTRL_MODE)
	{
		/*速度-电流双环控制*/
		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	//期望速度, rad per second
		SpdLoop.MaxExptMecAngularSpeed_rad = MAX_SPD;	//转速限幅
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//期望加速度, rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*位置-速度-电流三环控制*/
		SpdLoop.Kp = SPEED_CONTROL_KP;
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = 15.f * 2 * PI;	//转速限幅
		SpdLoop.Acceleration = 3000.f * 2 * PI;	//期望加速度, rad per quadratic seconds
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	if(Driver.ControlMode == TORQUE_CTRL_MODE)
	{
		/*转矩控制模式下, 在转速接近转速限幅时加入速度环控制*/
		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = TorqueCtrl.MaxMecSpd_rad;	//转速限幅
	}
}

 /**
   * @brief  位置环参数初始化
   */
void PositionLoopInit(void)
{					
	if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		/*位置-速度-电流三环控制*/
		PosLoop.Kp = POSITION_CONTROL_KP;
		PosLoop.Kd = POSITION_CONTROL_KD;	
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
	else if(Driver.ControlMode == POS_CURR_CTRL_MODE)
	{
		/*位置-电流双环控制*/
		PosLoop.Kp = 5.0f;
		PosLoop.Kd = 0.001f;
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
	}
}

 /**
   * @brief  转矩控制器初始化
   */
void TorqueCtrlInit(void)
{					
	TorqueCtrl.ExptTorque_Nm = 1.0f;	//期望扭矩
	TorqueCtrl.MaxMecSpd_rad = 60.f * 2.f * PI;	//转速限幅
	CurrLoop.LimitCurrQ = 200.f;		//Iq限幅
}

 /**
   * @brief  位置控制模式下标定初始位置
   */
void RefAngleInit(void)
{
	MainCtrl.PresentMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit;
	
	MainCtrl.LastMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit;
	
	/*记当前位置为零位*/
	MainCtrl.RefMecAngle_pulse = 0;
}

 /**
   * @brief  位置控制模式下标定初始位置
   */
void ZeroPosSet(uint16_t posOffset)
{
	MainCtrl.RefMecAngle_pulse = PosSensor.MecAngle_AbsoluteMode_15bit - posOffset;
}

 /**
   * @brief  电流环
   * @param[in]  exptCurrD     		期望Id
   * @param[in]  exptCurrQ      	期望Iq
   * @param[in]  realCurrD     		实际Id
   * @param[in]  realCurrQ      	实际Iq
   * @param[out] ctrlVolD 			Vd输出
   * @param[out] ctrlVolQ 			Vq输出
   */
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ)
{
	/*Iq输入限幅*/
	Saturation_float(&exptCurrQ, CurrLoop.LimitCurrQ, -CurrLoop.LimitCurrQ);
	
	CurrLoop.ErrD = exptCurrD - realCurrD;
	CurrLoop.ErrQ = exptCurrQ - realCurrQ;
	
	CurrLoop.IntegralErrD += CurrLoop.ErrD * DEFAULT_CARRIER_PERIOD_s;
	CurrLoop.IntegralErrQ += CurrLoop.ErrQ * DEFAULT_CARRIER_PERIOD_s;
	
	/*积分限幅*/
	Saturation_float(&CurrLoop.IntegralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	Saturation_float(&CurrLoop.IntegralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	
	/*基于电机模型的改进型电流控制器, 采用简化电机模型+I调节*/
	*ctrlVolD = CurrLoop.Kp_D * CurrLoop.ErrD + CurrLoop.Ki_D * CurrLoop.IntegralErrD;
	*ctrlVolQ = CurrLoop.Ki_Q * CurrLoop.IntegralErrQ + exptCurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	
	/*电压矢量限幅*/
	CurrLoop.LimitVolD = GENERATRIX_VOL / SQRT3;
	arm_sqrt_f32(SQUARE(GENERATRIX_VOL) / 3.f - SQUARE(*ctrlVolD), &CurrLoop.LimitVolQ);
	
	Saturation_float(ctrlVolD, CurrLoop.LimitVolD, -CurrLoop.LimitVolD);
	Saturation_float(ctrlVolQ, CurrLoop.LimitVolQ, -CurrLoop.LimitVolQ);
}

 /**
   * @brief  速度环
   * @param[in]  exptMecAngularSpeed     	期望机械角速度
   * @param[in]  realMecAngularSpeed      	实际机械角速度
   * @param[out] ctrlCurrQ 					Iq输出
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{
	/*最大期望速度限幅*/
	Saturation_float(&exptMecAngularSpeed, SpdLoop.MaxExptMecAngularSpeed_rad, -SpdLoop.MaxExptMecAngularSpeed_rad);
	
	SpdLoop.Err = exptMecAngularSpeed - realMecAngularSpeed;
	
	SpdLoop.IntegralErr += SpdLoop.Err * OUTER_LOOP_PERIOD;
	
	/*积分限幅*/
	Saturation_float(&SpdLoop.IntegralErr, SPD_INTEGRAL_ERR_LIM, -SPD_INTEGRAL_ERR_LIM);
	
	*ctrlCurrQ = SpdLoop.Kp * SpdLoop.Err + SpdLoop.Ki * SpdLoop.IntegralErr;
}

 /**
   * @brief  位置环
   * @param[in]  exptMecAngle     		期望机械角度
   * @param[in]  realMecAngle      		实际机械角度
   * @param[out] ctrlAngularSpeed 		角速度输出
   */
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed)
{
	/*位置限幅*/
	Saturation_float(&exptMecAngle, PosLoop.MecAngleUpperLimit_rad, PosLoop.MecAngleLowerLimit_rad);
	
	PosLoop.Err = exptMecAngle - realMecAngle;
		
	PosLoop.DiffErr = (PosLoop.Err - PosLoop.LastErr) / OUTER_LOOP_PERIOD;
	
	*ctrlAngularSpeed = PosLoop.Kp * PosLoop.Err + PosLoop.Kd * PosLoop.DiffErr;
		
	PosLoop.LastErr = PosLoop.Err;
}

 /**
   * @brief  速度-电流控制器
   */
void SpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//使程序第一次执行中断时, 运行速度环, 位置环
	
	Count++;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*位置环与速度环的周期是电流环周期的十倍*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*更新机械速度及位置信息*/
		GetMecImformation();
		
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  位置-速度-电流控制器
   */
void PosSpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//使程序第一次执行中断时, 运行速度环, 位置环
	
	Count++;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*位置环与速度环的周期是电流环周期的十倍*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*更新机械速度及位置信息*/
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &SpdLoop.ExptMecAngularSpeed_rad);
				
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  位置-电流控制器
   */
void PosCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	//使程序第一次执行中断时, 运行速度环, 位置环
	
	Count++;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
	
	/*位置环与速度环的周期是电流环周期的十倍*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*更新机械速度及位置信息*/
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*计算补偿角度*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  转矩控制器
   */
void TorqueController(void)
{  
	static uint16_t Count = PERIOD_MULTIPLE - 1;
	
	Count++;
	
	/*采用Id = 0控制, 故设定d轴电流为零*/
	CurrLoop.ExptCurrD = 0.f;
		
	/*速度环的周期是电流环周期的十倍*/
	if(Count == PERIOD_MULTIPLE)
	{
		/*更新机械速度及位置信息*/
		GetMecImformation();
					
		/*转速接近转速限幅时, 期望Iq改为速度环的计算结果*/
		if(PosSensor.MecAngularSpeed_rad < (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{		
			/*将目标转矩转换为Iq*/
			CurrLoop.ExptCurrQ = TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
		}
		else if(PosSensor.MecAngularSpeed_rad >= (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{
			SpeedLoop(TorqueCtrl.MaxMecSpd_rad, PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
			
			Saturation_float(&CurrLoop.ExptCurrQ, TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE), -TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE));
		}
			
		Count = 0;
	}

	/*进行Clark变换, 将abc坐标系转换为Alpha-Beta坐标系*/
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	/*正反转编码器延迟补偿系数不同*/
	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	/*计算补偿角度*/
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_s;
	
	/*进行Park变换, 将Alpha-Beta坐标系转换为dq坐标系*/
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*电流环PI控制器*/
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	/*进行逆Park变换, 将转子坐标系下的dq轴电压转换为定子坐标系下的AlphaBeta轴电压*/
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	/*利用SVPWM算法调制电压矢量*/
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  斜坡生成器
   * @param[in]  expectedVelocity      期望角速度
   */
float VelSlopeGenerator(float exptVel)
{
	static float velProcessVolume = 0.0f;
	static float velStep_Acc = 0;
	static float velStep_Dec = 0;
	
	velStep_Acc = SpdLoop.Acceleration * OUTER_LOOP_PERIOD;
	velStep_Dec = SpdLoop.Deceleration * OUTER_LOOP_PERIOD;
	
	if(exptVel > 0 && velProcessVolume >= 0)
	{
		if(velProcessVolume < (exptVel - velStep_Acc))
		{
			velProcessVolume += velStep_Acc;
		}
		else if(velProcessVolume > (exptVel + velStep_Dec))
		{
			velProcessVolume -= velStep_Dec;
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	else if(exptVel > 0 && velProcessVolume < 0)
	{
		velProcessVolume += velStep_Dec;
	}
	else if(exptVel < 0 && velProcessVolume >=  0)
	{
		velProcessVolume -= velStep_Dec;
	}
	else if(exptVel < 0 && velProcessVolume < 0)
	{
		if(velProcessVolume > (exptVel + velStep_Acc))
		{
			velProcessVolume -= velStep_Acc;
		}
		else if(velProcessVolume < (exptVel - velStep_Dec))
		{
			velProcessVolume += velStep_Dec;	
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	else if(exptVel == 0)
	{
		velProcessVolume = 0;
	}
	
	return velProcessVolume;
}

 /**
   * @brief  驱动器控制模式初始化
   */
void DriverCtrlModeInit(void)
{		
	switch(Driver.ControlMode)
	{
		case SPD_CURR_CTRL_MODE : 	
										CurrentLoopInit();

										SpeedLoopInit();

										break;
		
		case POS_SPD_CURR_CTRL_MODE :	
										CurrentLoopInit();
										
										SpeedLoopInit();
										
										PositionLoopInit();
			
										break;
		case POS_CURR_CTRL_MODE :	
										CurrentLoopInit();
										
										PositionLoopInit();
			
										break;
		case TORQUE_CTRL_MODE :	
										TorqueCtrlInit();	//转矩控制器初始化需在执行速度环和电流环初始化之前执行
		
										CurrentLoopInit();
										
										SpeedLoopInit();		
		
										break;
	}
}

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
