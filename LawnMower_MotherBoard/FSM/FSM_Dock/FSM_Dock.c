/*
 * FSM_Dock.c
 *
 *  Created on: 12 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"
#include "RUN_ADC.h"
#include "RUN_Sensors.h"
#include "RUN_GPIO.h"
#include "RUN_PWM.h"
#include "RUN_Mower.h"

#include "FSM_Enum.h"
#include "FSM_Dock.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
uint8_t gu8_isCharging;
uint8_t gu8_leavingDockState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static void _FSM_Dock_LeavingDockCharger(uint32_t u32_CyclicTask);
static void _FSM_Dock_DisableAllMotor(void);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Dock_Init()
{
	gu8_isCharging = 0;
	gu8_leavingDockState = 0;
}

void FSM_Dock(S_MOWER_FSM_STATE e_FSM_Dock_State)
{
	uint32_t u32_CyclicTask = 0;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/
	u32_CyclicTask = RUN_Task_GetCyclicTask();

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

    switch( e_FSM_Dock_State )
	{
		default:
		case S_SUP_DOCK_Init:
			FSM_Dock_Init();
			_FSM_Dock_DisableAllMotor();
			RUN_GPIO_UpdateBladeState(OFF);
			
			if (RUN_Sensors_IsCharging() == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_In_Charge);
			}
			else if (RUN_Sensors_GetDockState() == OFF)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Init);
			}
			else
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Mow);
			}

			break;
		case S_SUP_DOCK_In_Charge :
			if ( (RUN_Sensors_GetRainState() == OFF) && (RUN_Sensors_IsEnoughCharged() == 1) && (RUN_GPIO_GetStartButton() == 1) )
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Leaving_Dock);
			}
			else if (RUN_Sensors_IsCharging() == 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Mow);
			}
			else
			{
				RUN_Mower_SetEtatMower(EN_CHARGE);
			}
			
			break;
		case S_SUP_DOCK_Waiting_For_Mow :
			if ( (RUN_Sensors_IsTimeToMow() == 1) && (RUN_Sensors_GetRainState() == OFF) )
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Waiting_For_Leaving_Dock);
			}

			RUN_Mower_SetEtatMower(PAS_DE_TACHE_EN_COURS);

			if (RUN_Sensors_IsCharging() == 1)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_In_Charge);
				RUN_Mower_SetEtatMower(NTR);
				RUN_Mower_SetEtatMower(TACHE_EN_COURS);
			}

			break;
		case S_SUP_DOCK_Waiting_For_Leaving_Dock :			
			FSM_Dock_LeavingDockCharger(u32_CyclicTask);
			if (gu8_leavingDockState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_OPERATIVE_Init);
			}

			break;
	}
}

static void _FSM_Dock_LeavingDockCharger(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_LEAVE_DOCK) != 0) {
		gu8_leavingDockState = RUN_Mower_LeaveDockCharger();

		RUN_Task_EraseCyclicTask(CYCLIC_TASK_ADC_READ_VALUE);
	}
}

static void _FSM_Dock_DisableAllMotor()
{
	RUN_GPIO_DisableAllMotor();
	RUN_PWM_Stop();
}
