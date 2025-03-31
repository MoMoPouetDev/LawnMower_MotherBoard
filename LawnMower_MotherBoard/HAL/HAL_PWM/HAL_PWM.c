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
void HAL_PWM_Enable(void);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_PWM_Init()
{
	LLD_PWM_Init(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 1000);
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 0);
	LLD_PWM_Enable(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, true);

	LLD_PWM_Init(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 1000);
	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 0);
	LLD_PWM_Enable(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, true);
}

void HAL_PWM_Stop()
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 0); 

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 0);

	HAL_PWM_Enable();
}

void HAL_PWM_Forward(uint8_t u8_speedLeft, uint8_t u8_speedRight)
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, u8_speedLeft); 

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, u8_speedRight);

	HAL_PWM_Enable();
}

void HAL_PWM_Turn()
{
	LLD_PWM_SetDutyCycle(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, 100);

	LLD_PWM_SetDutyCycle(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, 100);

	HAL_PWM_Enable();
}

void HAL_PWM_Enable()
{
	LLD_PWM_Enable(MOTOR_LEFT_FORWARD, MOTOR_LEFT_CHANNEL, true);
	LLD_PWM_Enable(MOTOR_RIGHT_FORWARD, MOTOR_RIGHT_CHANNEL, true);
}
