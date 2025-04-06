/*
 * RUN_GPIO.c
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_GPIO.h"
#include "RUN_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_GPIO_Init()
{
	HAL_GPIO_Init();
}

void RUN_GPIO_DisableAllMotor()
{
	HAL_GPIO_UpdateBladeState(OFF);
	HAL_GPIO_UpdateWheelState(STOP);
}

void RUN_GPIO_DisableMotor()
{
	HAL_GPIO_UpdateWheelState(STOP);
}

void RUN_GPIO_UpdateBladeState(Etat e_bladeState)
{
	HAL_GPIO_BladeState(e_bladeState);
}

uint8_t RUN_GPIO_GetStartButton()
{
	uint8_t u8_flagButton = 0;

	u8_flagButton = HAL_GPIO_GetFlagButton(E_START_BUTTON);

	return u8_flagButton;
}

uint8_t RUN_GPIO_GetStopButton()
{
	uint8_t u8_flagButton = 0;

	u8_flagButton = HAL_GPIO_GetFlagButton(E_STOP_BUTTON);

	return u8_flagButton;
}

