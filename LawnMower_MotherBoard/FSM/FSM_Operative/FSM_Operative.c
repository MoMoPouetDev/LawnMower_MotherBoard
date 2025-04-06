/*
 * FSM_Operative.c
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "HAL_GPIO.h"
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"
#include "RUN_ADC.h"
#include "RUN_GPIO.h"
#include "RUN_Sensors.h"
#include "RUN_Mower.h"
#include "RUN_PWM.h"

#include "FSM_Enum.h"
#include "FSM_Operative.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
static uint8_t gu8_runMowerState;
static uint8_t gu8_wireDetectionState;
static uint8_t gu8_bumperDetectionState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Operative_SonarDistance(uint32_t u32_CyclicTask);
void FSM_Operative_SensorRead(uint32_t u32_CyclicTask);
void FSM_Operative_TiltProtection(uint32_t u32_CyclicTask);
void FSM_Operative_RunMower(uint32_t u32_CyclicTask);
void FSM_Operative_WireDetection(uint32_t u32_CyclicTask);
void FSM_Operative_BumperDetection(uint32_t u32_CyclicTask);
void FSM_Operative_DisableAllMotor(void);
void FSM_Operative_GetFlagStartButton(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Operative_Init()
{
	gu8_runMowerState = 0;
	gu8_wireDetectionState = 0;
	gu8_bumperDetectionState = 0;
}

void FSM_Operative(S_MOWER_FSM_STATE e_FSM_Operative_State)
{
	uint8_t u8_timeToMow = 0;
	uint8_t u8_startButtonState = 0;
	uint8_t u8_stopButtonState = 0;
	int8_t s8_charge = 0;
	Etat e_rain = OFF;
	uint32_t u32_CyclicTask;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/
	u32_CyclicTask = RUN_Task_GetCyclicTask();

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

	switch( e_FSM_Operative_State )
   	{
	  	default:
	  	case S_SUP_OPERATIVE_Init:
		 	FSM_Operative_Init();
			RUN_GPIO_UpdateBladeState(ON);
			FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);

			break;

	  	case S_SUP_OPERATIVE_Moving :
			FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_Operative_SensorRead(u32_CyclicTask);
			FSM_Operative_RunMower(u32_CyclicTask);	

			//e_rain = RUN_Sensors_GetRainState();
			//s8_charge = RUN_Sensors_IsEnoughCharged();
			//u8_timeToMow = RUN_Sensors_IsTimeToMow();
			u8_startButtonState = RUN_GPIO_GetStartButton();
			u8_stopButtonState = RUN_GPIO_GetStopButton();
			
			if (gu8_runMowerState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Wire_Detection);
			}
			else if (gu8_runMowerState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Bumper_Detection);
			}

			//RUN_GPIO_SetEtatMowerInTask();
			//RUN_GPIO_SetErrorMowerNtr();

			if (u8_startButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
			else if (u8_stopButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			else if (s8_charge == 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			else if (s8_charge == -1)
			{
				//RUN_GPIO_SetEtatMowerReturnToBase();
				FSM_Enum_SetFsmPhase(S_SUP_ERROR_Init);
			}
			else if (e_rain == ON)
			{
				//RUN_GPIO_SetErrorMowerRain();
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			else if (u8_timeToMow == 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			break;

	  	case S_SUP_OPERATIVE_Wire_Detection :
			FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_Operative_WireDetection(u32_CyclicTask);

			if (gu8_wireDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
			else if (gu8_wireDetectionState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Bumper_Detection);
			}
		 	break;

	  	case S_SUP_OPERATIVE_Bumper_Detection:
			FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_Operative_BumperDetection(u32_CyclicTask);

			if (gu8_bumperDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
		 	break;

		case S_SUP_OPERATIVE_Waiting:
			FSM_Operative_DisableAllMotor();
			FSM_Operative_GetFlagStartButton(u32_CyclicTask);

			u8_startButtonState = RUN_GPIO_GetStartButton();
			u8_stopButtonState = RUN_GPIO_GetStopButton();

			//RUN_GPIO_SetEtatMowerInWait();
			//RUN_GPIO_SetErrorMowerNtr();

			if (u8_startButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
			else if (u8_stopButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
		 	break;

	  	case S_SUP_OPERATIVE_Waiting_For_Return_To_Base :
			FSM_Operative_DisableAllMotor();
			//RUN_GPIO_SetEtatMowerReturnToBase();

			FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Init);
			break;
   	}
}

void FSM_Operative_SonarDistance(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SONAR) != 0) {
		RUN_Mower_SonarDistance(); // from slave - MVE
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SONAR);
	}
}

void FSM_Operative_SensorRead(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SENSOR_READ) != 0) {
		// MVE
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SENSOR_READ);
	}
}

void FSM_Operative_RunMower(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_RUN_MOWER) != 0) {
		gu8_runMowerState = RUN_Mower_RunMower();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_RUN_MOWER);
	}
}

void FSM_Operative_WireDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_WIRE_DETECTION) != 0) {
		gu8_wireDetectionState = RUN_Mower_WireDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_WIRE_DETECTION);
	}
}

void FSM_Operative_BumperDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_BUMPER_DETECTION) != 0) {
		gu8_bumperDetectionState = RUN_Mower_BumperDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_BUMPER_DETECTION);
	}
}

void FSM_Operative_DisableAllMotor()
{
	RUN_GPIO_DisableAllMotor();
	RUN_PWM_Stop();
}

void FSM_Operative_DisableMotor()
{
	RUN_GPIO_DisableMotor();
	RUN_PWM_Stop();
}
