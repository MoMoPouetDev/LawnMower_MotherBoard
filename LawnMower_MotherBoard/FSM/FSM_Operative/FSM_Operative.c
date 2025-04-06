/*
 * FSM_Operative.c
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"
#include "RUN_GPIO.h"
#include "RUN_Sensors.h"
#include "RUN_Mower.h"

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
static void _FSM_Operative_RunMower(uint32_t u32_CyclicTask);
static void _FSM_Operative_WireDetection(uint32_t u32_CyclicTask);
static void _FSM_Operative_BumperDetection(uint32_t u32_CyclicTask);
static void _FSM_Operative_DisableAllMotor(void);
static void _FSM_Operative_DisableMotor(void);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Operative_Init(void)
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
			_FSM_Operative_RunMower(u32_CyclicTask);	

			e_rain = RUN_Sensors_GetRainState();
			s8_charge = RUN_Sensors_IsEnoughCharged();
			u8_timeToMow = RUN_Mower_IsTimeToMow();
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

			RUN_Mower_SetEtatMower(TACHE_EN_COURS);
			RUN_Mower_SetErrorMower(NTR);

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
				RUN_Mower_SetEtatMower(RETOUR_STATION);
				FSM_Enum_SetFsmPhase(S_SUP_ERROR_Init);
			}
			else if (e_rain == ON)
			{
				RUN_Mower_SetErrorMower(DETECTED_RAIN);
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			else if (u8_timeToMow == 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			break;

	  	case S_SUP_OPERATIVE_Wire_Detection :
			_FSM_Operative_WireDetection(u32_CyclicTask);

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
			_FSM_Operative_BumperDetection(u32_CyclicTask);

			if (gu8_bumperDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
		 	break;

		case S_SUP_OPERATIVE_Waiting:
			_FSM_Operative_DisableAllMotor();

			u8_startButtonState = RUN_GPIO_GetStartButton();
			u8_stopButtonState = RUN_GPIO_GetStopButton();

			RUN_Mower_SetEtatMower(PAS_DE_TACHE_EN_COURS);
			RUN_Mower_SetErrorMower(NTR);

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
			_FSM_Operative_DisableAllMotor();
			RUN_Mower_SetEtatMower(RETOUR_STATION);

			FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Init);
			break;
   	}
}

static void _FSM_Operative_RunMower(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_RUN_MOWER) != 0) {
		gu8_runMowerState = RUN_Mower_RunMower();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_RUN_MOWER);
	}
}

static void _FSM_Operative_WireDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_WIRE_DETECTION) != 0) {
		gu8_wireDetectionState = RUN_Mower_WireDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_WIRE_DETECTION);
	}
}

static void _FSM_Operative_BumperDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_BUMPER_DETECTION) != 0) {
		gu8_bumperDetectionState = RUN_Mower_BumperDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_BUMPER_DETECTION);
	}
}

static void _FSM_Operative_DisableAllMotor(void)
{
	RUN_GPIO_DisableAllMotor();
	RUN_PWM_Stop();
}

static void _FSM_Operative_DisableMotor(void)
{
	RUN_GPIO_DisableMotor();
	RUN_PWM_Stop();
}
