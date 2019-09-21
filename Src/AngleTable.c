/**
 ******************************************************************************
 * @file		AngleTable.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		Angle table of motors
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "AngleTable.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */
#if POSITION_SENSOR_TYPE == HALL_SENSOR_DRV5053
	#if ROBOT_ID == 1U
		#if CAN_ID_NUM == 1
			int GivenEle[CALIBRATE_NUM + 2] =
			{
			-60,
				0,
			60,
				120,
				180,
				240,
				300,
				360,
			};

			int HallEle[CALIBRATE_NUM + 2]=
			{
			-51,
			 14,
			 80,
			139,
			184,
			259,
			325,
			370,
			};
		#elif CAN_ID_NUM == 2
			int GivenEle[CALIBRATE_NUM + 2] =
			{
			-60,
				0,
			60,
				120,
				180,
				240,
				300,
				360,
			};

			int HallEle[CALIBRATE_NUM + 2]=
			{
			-51,
			 14,
			 80,
			139,
			184,
			259,
			325,
			370,
			};
		#endif
	#elif ROBOT_ID == 2U
		#if CAN_ID_NUM == 1
			int GivenEle[CALIBRATE_NUM + 2] =
			{
			-60,
			0,
			60,
			120,
			180,
			240,
			300,
			360,
			};

			int HallEle[CALIBRATE_NUM + 2]=
			{
			-62,
			1,
			63,
			127,
			179,
			245,
			310,
			364,
			};
		#elif CAN_ID_NUM == 2
			int GivenEle[CALIBRATE_NUM + 2] =
			{
			-60,
			0,
			60,
			120,
			180,
			240,
			300,
			360,
			};

			int HallEle[CALIBRATE_NUM + 2]=
			{
			-51,
			 14,
			 80,
			139,
			184,
			259,
			325,
			370,
			};
		#endif
	#endif
#endif
/* CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* CODE BEGIN PM */

/* CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* CODE BEGIN PV */

/* CODE END PV */

/* USER CODE BEGIN */

/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
