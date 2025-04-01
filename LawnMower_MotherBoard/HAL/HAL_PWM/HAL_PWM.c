/*
 * HAL_PWM.c
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_PWM.h"
#include "LLD_PWM.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_PWM_Init()
{
	LLD_PWM_Init();
}

void HAL_PWM_Stop()
{
	LLD_PWM_Stop();
}

void HAL_PWM_Forward(uint8_t u8_speedLeft, uint8_t u8_speedRight)
{
	LLD_PWM_Forward(u8_speedLeft, u8_speedRight);
}

void HAL_PWM_Turn(uint8_t u8_turn)
{
	switch (u8_turn)
	{
		case 0:
			LLD_PWM_Left();
			break;

		case 1:
			LLD_PWM_Right();
			break;
		
		default:
			break;
	}
}
