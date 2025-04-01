/*
 * FSM_Init.c
 *
 *  Created on: 14 OCT 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_GPIO.h"

#include "FSM_Enum.h"
#include "FSM_Init.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static void _FSM_Init_Init(void);
static void _FSM_Init_GetFlagStartButton(uint32_t u32_CyclicTask);
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
static void _FSM_Init_Init()
{

}

void FSM_Init(S_MOWER_FSM_STATE e_FSM_Init_State)
{
	uint8_t u8_startButtonState = 0;
	uint32_t u32_CyclicTask;

	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/
    switch( e_FSM_Init_State )
	{
		default:
		case S_SUP_INIT_Init:
			FSM_Init_Init();
			FSM_Init_GetFlagStartButton(u32_CyclicTask);

			u8_startButtonState = RUN_GPIO_GetStartButton();
			RUN_GPIO_UpdateBladeState(OFF);
			
			if (u8_startButtonState != 0)
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Init);
			}
			break;
	}
}
