/*
 * FSM_Init.c
 *
 *  Created on: 14 OCT 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/* ... INCLUDES ...                                                         */
/*--------------------------------------------------------------------------*/
#include "RUN_Sensors.h"
#include "RUN_GPIO.h"
#include "RUN_Mower.h"
#include "FSM_Enum.h"
#include "FSM_Init.h"

/*--------------------------------------------------------------------------*/
/* ... DATAS TYPE ...                                                       */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* ... FUNCTIONS DEFINITIONS...                                              */
/*---------------------------------------------------------------------------*/
void FSM_Init_Init()
{

}

void FSM_Init(S_MOWER_FSM_STATE e_FSM_Init_State)
{
	uint8_t u8_startButtonState = 0;
	uint8_t u8_slaveState = 0;
	float f_latitude = 0.0;
	float f_longitude = 0.0;
	/***************************************************************************************************************/
	/*                                  ACU FINITE STATE MACHINE                                                   */
	/***************************************************************************************************************/
    switch( e_FSM_Init_State )
	{
		default:
		case S_SUP_INIT_Init:
			FSM_Init_Init();

			u8_startButtonState = RUN_GPIO_GetStartButton();
			u8_slaveState = RUN_Sensors_GetSlaveState();
			f_latitude = RUN_Sensors_GetLatitude();
			f_longitude = RUN_Sensors_GetLongitude();
			RUN_GPIO_UpdateBladeState(OFF);
			
			if ((f_latitude!= 0.0) && (f_longitude!= 0.0))
			{
				RUN_Mower_SetEtatMower(GPS_READY);
			}
			
			if ((u8_startButtonState != 0) && (u8_slaveState != 0) && (f_latitude!= 0.0) && (f_longitude!= 0.0))
			{
				FSM_Enum_SetFsmPhase(S_SUP_DOCK_Init);
			}
			break;
	}
}
