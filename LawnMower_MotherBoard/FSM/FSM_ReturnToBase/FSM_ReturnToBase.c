/*
 * FSM_ReturnToBase.c
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_GPIO.h"
#include "RUN_Mower.h"
#include "RUN_Sensors.h"
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"

#include "FSM_Enum.h"
#include "FSM_ReturnToBase.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
static uint8_t gu8_angleToBaseState;
static uint8_t gu8_bumperDetectionState;
static uint8_t gu8_sonarDetectionState;
static uint8_t gu8_runMowerState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static void _FSM_ReturnToBase_GetAngleToBase(uint32_t u32_CyclicTask);
static void _FSM_ReturnToBase_RunMower(uint32_t u32_CyclicTask);
static void _FSM_ReturnToBase_BumperDetection(uint32_t u32_CyclicTask);
static void _FSM_ReturnToBase_SonarDetection(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_ReturnToBase_Init()
{
	gu8_angleToBaseState = 0;
	gu8_runMowerState = 0;
	gu8_bumperDetectionState = 0;
	gu8_sonarDetectionState = 0;
}

void FSM_ReturnToBase(S_MOWER_FSM_STATE e_FSM_ReturnToBase_State)
{
	int8_t s8_charge = 0;
	uint32_t u32_CyclicTask;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/
	u32_CyclicTask = RUN_Task_GetCyclicTask();
	RUN_GPIO_UpdateBladeState(ON);
	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

    switch( e_FSM_ReturnToBase_State )
	{
		default:
	  	case S_SUP_RETURN_TO_BASE_Init:
			FSM_ReturnToBase_Init();
			RUN_GPIO_UpdateBladeState(ON);
			FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Angle_To_Base);

			break;
		case S_SUP_RETURN_TO_BASE_Angle_To_Base:
		 	_FSM_ReturnToBase_GetAngleToBase(u32_CyclicTask);

		 	if (gu8_angleToBaseState == 1)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Moving);	
		 	}
			else if (gu8_angleToBaseState == 2)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Bumper_Detection);	
		 	}

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Moving :
			_FSM_ReturnToBase_RunMower(u32_CyclicTask);

			s8_charge = RUN_Sensors_IsEnoughCharged();

			if (gu8_runMowerState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Bumper_Detection);
			}
			else if (gu8_runMowerState == 3)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Sonar_Detection);
			}

		 	break;

	  	case S_SUP_RETURN_TO_BASE_Bumper_Detection :
			_FSM_ReturnToBase_BumperDetection(u32_CyclicTask);

			if (gu8_bumperDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Moving);
			}

		 	break;

	  	case S_SUP_RETURN_TO_BASE_Sonar_Detection :
			_FSM_ReturnToBase_SonarDetection(u32_CyclicTask);

			if (gu8_sonarDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Moving);
			}
		 	break;

	  	case S_SUP_RETURN_TO_BASE_Waiting_For_Docking :
		  /* Insert init code */

		 	break;
   	}
}

static void _FSM_ReturnToBase_GetAngleToBase(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_RUN_MOWER) != 0) {
		RUN_Mower_DirectionFromBase();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_RUN_MOWER);
	}
}

static void _FSM_ReturnToBase_RunMower(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_RUN_MOWER) != 0) {
		gu8_runMowerState = RUN_Mower_RunMower();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_RUN_MOWER);
	}
}

static void _FSM_ReturnToBase_BumperDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_BUMPER_DETECTION) != 0) {
		gu8_bumperDetectionState = RUN_Mower_BumperDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_BUMPER_DETECTION);
	}
}

static void _FSM_ReturnToBase_SonarDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SONAR_DETECTION) != 0) {
		gu8_sonarDetectionState = RUN_Mower_SonarDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SONAR_DETECTION);
	}
}
