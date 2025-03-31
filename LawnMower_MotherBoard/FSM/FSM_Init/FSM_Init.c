/*
 * FSM_Init.c
 *
 *  Created on: 14 OCT 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Task.h"
#include "RUN_Task_Interface.h"
#include "RUN_GPIO.h"

#include "FSM_Enum.h"
#include "FSM_Init.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/
uint8_t gu8_startButtonState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void FSM_Init_Init(void);
void FSM_Init_GetFlagStartButton(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Init_Init()
{
	gu8_startButtonState = 0;
}

void FSM_Init(S_MOWER_FSM_STATE e_FSM_Init_State)
{
	uint32_t u32_CyclicTask;
	/***************************************************************************************************************/
	/*                                      MANAGE RUN TASK CYCLE                                                  */
	/***************************************************************************************************************/
	u32_CyclicTask = RUN_Task_GetCyclicTask();

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/

    switch( e_FSM_Init_State )
   {
	  	default:
	  	case S_SUP_INIT_Init:
			FSM_Init_Init();
			FSM_Init_GetFlagStartButton(u32_CyclicTask);
			RUN_GPIO_UpdateBladeState(OFF);
			
			if (gu8_startButtonState)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Init);
			}
			break;
   }
}

void FSM_Init_GetFlagStartButton(uint32_t u32_CyclicTask)
{
	if ( (u32_CyclicTask & CYCLIC_TASK_READ_START_BUTTON) != 0) {
		gu8_startButtonState = RUN_GPIO_GetStartButton();
		RUN_Task_EraseCyclicTask(CYCLIC_TASK_READ_START_BUTTON);
	}
}

