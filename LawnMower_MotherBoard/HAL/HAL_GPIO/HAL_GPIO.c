/*
 * HAL_GPIO.c
 *
 *  Created on: 19 août 2022
 *      Author: morgan.venandy
 */


/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>

#include "HAL_Timer.h"
#include "HAL_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
Etat ge_bladeState;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_Init()
{
	LLD_GPIO_Init();
	ge_bladeState = OFF;
}

void HAL_GPIO_UpdateBladeState(Etat e_bladeState)
{
	ge_bladeState = e_bladeState;
}

void HAL_GPIO_BladeState(Etat e_bladeState)
{
	switch(e_bladeState) 
	{
		case ON:
			if(ge_bladeState == ON)
			{
				LLD_GPIO_WritePin(E_MOTOR_BLADE_ENABLE);
			}
			else
			{
				LLD_GPIO_ClearPin(E_MOTOR_BLADE_ENABLE);
			}
			break;
		default:
		case OFF:
			LLD_GPIO_ClearPin(E_MOTOR_BLADE_ENABLE);
			break;
	}
}

void HAL_GPIO_UpdateWheelState(MotorState e_wheelState)
{
	switch(e_wheelState) {
		default:
		case STOP:
			LLD_GPIO_ClearPin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case FORWARD:
			LLD_GPIO_WritePin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case BACKWARD:
			LLD_GPIO_ClearPin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case LEFT:
			LLD_GPIO_ClearPin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
		case RIGHT:
			LLD_GPIO_WritePin(E_MOTOR_ONE_FORWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_ONE_BACKWARD_ENABLE);
			LLD_GPIO_ClearPin(E_MOTOR_TWO_FORWARD_ENABLE);
			LLD_GPIO_WritePin(E_MOTOR_TWO_BACKWARD_ENABLE);
			break;
	}
}

uint8_t HAL_GPIO_GetFlagButton(GPIO e_flagButton)
{
	static uint8_t _u8_flagStartButton = 0;
	static uint8_t _u8_flagStopButton = 0;
	uint8_t u8_flagButton = 0;

	switch (e_flagButton)
	{
		case E_STOP_BUTTON :
			_u8_flagStartButton = LLD_GPIO_ReadPin(E_STOP_BUTTON);
			if (_u8_flagStartButton == 0)
			{
				u8_flagButton = 1;
			}
			break;

		case E_START_BUTTON :
			_u8_flagStopButton = LLD_GPIO_ReadPin(E_START_BUTTON);
			if (_u8_flagStopButton == 0)
			{
				u8_flagButton = 1;
			}
			break;
		
		default:
			break;
	}

	return u8_flagButton;
}

uint8_t HAL_GPIO_GetFlagBumper(GPIO e_flagBumper)
{
	static uint8_t _u8_flagLeftBumper = 0;
	static uint8_t _u8_flagCenterBumper = 0;
	static uint8_t _u8_flagRightBumper = 0;
	uint8_t u8_flagBumper = 0;

	switch (e_flagBumper)
	{
		case E_LEFT_BUMPER :
			_u8_flagLeftBumper = LLD_GPIO_ReadPin(E_LEFT_BUMPER);
			if (_u8_flagLeftBumper == 0)
			{
				u8_flagBumper = 1;
			}
			break;

		case E_CENTER_BUMPER :
			_u8_flagCenterBumper = LLD_GPIO_ReadPin(E_CENTER_BUMPER);
			if (_u8_flagCenterBumper == 0)
			{
				u8_flagBumper = 1;
			}
			break;
		
		case E_RIGHT_BUMPER :
			_u8_flagRightBumper = LLD_GPIO_ReadPin(E_RIGHT_BUMPER);
			if (_u8_flagRightBumper == 0)
			{
				u8_flagBumper = 1;
			}
			break;

		default:
			break;
	}
	return u8_flagBumper;
}
