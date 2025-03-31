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
static uint8_t gu8_startButtonState;
static uint8_t gu8_stopButtonState;
static uint8_t gu8_runMowerState;
static uint8_t gu8_wireDetectionState;
static uint8_t gu8_bumperDetectionState;
static uint8_t gu8_timeToMow;
static int8_t gs8_charge;
static Etat ge_rain;

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
	gu8_startButtonState = 0;
	gu8_stopButtonState = 0;
	gu8_runMowerState = 0;
	gu8_wireDetectionState = 0;
	gu8_bumperDetectionState = 0;
	gu8_timeToMow = 0;
	gs8_charge = 0;
	ge_rain = OFF;
}

void FSM_Operative(S_MOWER_FSM_STATE e_FSM_Operative_State)
{
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
			
			if (gu8_runMowerState == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Wire_Detection);
			}
			else if (gu8_runMowerState == 2)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Bumper_Detection);
			}

			RUN_GPIO_SetEtatMowerInTask();
			RUN_GPIO_SetErrorMowerNtr();

			if (gs8_charge == 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			else if (gs8_charge == -1)
			{
				RUN_GPIO_SetEtatMowerReturnToBase();
				FSM_Enum_SetFsmPhase(S_SUP_ERROR_Init);
			}
			else if (ge_rain == ON)
			{
				RUN_GPIO_SetErrorMowerRain();
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			else if (gu8_timeToMow == 0)
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

			RUN_GPIO_SetEtatMowerInWait();
			RUN_GPIO_SetErrorMowerNtr();

			if (gu8_startButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Moving);
			}
			else if (gu8_stopButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Waiting_For_Return_To_Base);
			}
			
		 	break;
	  	case S_SUP_OPERATIVE_Waiting_For_Return_To_Base :
			FSM_Operative_DisableAllMotor();
			RUN_GPIO_SetEtatMowerReturnToBase();

			FSM_Enum_SetFsmPhase(S_SUP_RETURN_TO_BASE_Init);

			break;
   	}
}

void FSM_Operative_SonarDistance(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SONAR) != 0) {
		RUN_Mower_SonarDistance();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_SONAR);
	}
}

void FSM_Operative_SensorRead(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_SENSOR_READ) != 0) {
		ge_rain = RUN_Sensors_GetRainState();
		gs8_charge = RUN_Sensors_IsEnoughCharged();
		gu8_timeToMow = RUN_Sensors_IsTimeToMow();
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

void FSM_Operative_GetFlagStartButton(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_READ_START_BUTTON) != 0) {
		gu8_startButtonState = RUN_GPIO_GetStartButton();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_READ_START_BUTTON);
	}
}

void FSM_Operative_GetFlagStopButton(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_READ_STOP_BUTTON) != 0) {
		gu8_stopButtonState = RUN_GPIO_GetStopButton();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_READ_STOP_BUTTON);
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
