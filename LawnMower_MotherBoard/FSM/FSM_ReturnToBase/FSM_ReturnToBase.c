/*
 * FSM_ReturnToBase.c
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "HAL_GPIO.h"
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
static uint8_t gu8_wireDetectionState;
static uint8_t gu8_bumperDetectionState;
static uint8_t gu8_runMowerState;
static uint8_t gu8_wireGuidingState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_ReturnToBase_SonarDistance(uint32_t u32_CyclicTask);
void FSM_ReturnToBase_GetAngleToBase(uint32_t u32_CyclicTask);
void FSM_ReturnToBase_SensorRead(uint32_t u32_CyclicTask);
void FSM_ReturnToBase_RunMower(uint32_t u32_CyclicTask);
void FSM_ReturnToBase_WireDetection(uint32_t u32_CyclicTask);
void FSM_ReturnToBase_BumperDetection(uint32_t u32_CyclicTask);
void FSM_ReturnToBase_WireGuiding(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_ReturnToBase_Init()
{
	gu8_angleToBaseState = 0;
	gu8_runMowerState = 0;
	gu8_wireDetectionState = 0;
	gu8_bumperDetectionState = 0;
	gu8_wireGuidingState = 0;
}

void FSM_ReturnToBase(S_MOWER_FSM_STATE e_FSM_ReturnToBase_State)
{
	uint8_t u8_timeToMow = 0;
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

    switch( e_FSM_ReturnToBase_State )
	{
		default:
	  	case S_SUP_RETURN_TO_BASE_Init:
			FSM_ReturnToBase_Init();
			RUN_GPIO_UpdateBladeState(ON);
			FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Angle_To_Base);

			break;
		case S_SUP_RETURN_TO_BASE_Angle_To_Base:
			FSM_ReturnToBase_SonarDistance(u32_CyclicTask);
		 	FSM_ReturnToBase_GetAngleToBase(u32_CyclicTask);

		 	if (gu8_angleToBaseState == 1)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Moving);	
		 	}
			else if (gu8_angleToBaseState == 2)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Bumper_Detection);	
		 	}
			else if (gu8_angleToBaseState == 3)
		 	{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Wire_Detection);	
		 	}

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Moving :
			FSM_ReturnToBase_SonarDistance(u32_CyclicTask);
			FSM_ReturnToBase_RunMower(u32_CyclicTask);

			e_rain = RUN_Sensors_GetRainState();
			s8_charge = RUN_Sensors_IsEnoughCharged();
			u8_timeToMow = RUN_Sensors_IsTimeToMow();

			if (gu8_runMowerState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Wire_Detection);
			}
			else if (gu8_runMowerState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Bumper_Detection);
			}

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Wire_Detection :
		  	FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_ReturnToBase_WireDetection(u32_CyclicTask);

			if (gu8_wireDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Wire_Guiding);
			}
			else if (gu8_wireDetectionState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Bumper_Detection);
			}

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Bumper_Detection :
		  	FSM_Operative_SonarDistance(u32_CyclicTask);
			FSM_ReturnToBase_BumperDetection(u32_CyclicTask);

			if (gu8_bumperDetectionState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Wire_Guiding:
		 	FSM_ReturnToBase_WireGuiding(u32_CyclicTask);

			if (gu8_wireGuidingState == 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Angle_To_Base);
			}
			

		 	break;
	  	case S_SUP_RETURN_TO_BASE_Waiting_For_Docking :
		  /* Insert init code */

		 	break;
   	}
}

void FSM_ReturnToBase_SonarDistance(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SONAR) != 0) {
		RUN_Mower_SonarDistance(); // - MVE
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SONAR);
	}
}

void FSM_ReturnToBase_GetAngleToBase(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SONAR) != 0) {
		RUN_Mower_DirectionFromBase();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SONAR);
	}
}

void FSM_ReturnToBase_SensorRead(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SENSOR_READ) != 0) {
		// MVE
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SENSOR_READ);
	}
}

void FSM_ReturnToBase_RunMower(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_RUN_MOWER) != 0) {
		gu8_runMowerState = RUN_Mower_RunMower();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_RUN_MOWER);
	}
}

void FSM_ReturnToBase_WireDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_WIRE_DETECTION) != 0) {
		gu8_wireDetectionState = RUN_Mower_WireDetectionOnReturn();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_WIRE_DETECTION);
	}
}

void FSM_ReturnToBase_BumperDetection(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_BUMPER_DETECTION) != 0) {
		gu8_bumperDetectionState = RUN_Mower_BumperDetection();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_BUMPER_DETECTION);
	}
}

void FSM_ReturnToBase_WireGuiding(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_WIRE_GUIDING) != 0) {
		gu8_wireGuidingState = RUN_Mower_WireGuiding();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_WIRE_GUIDING);
	}
}
