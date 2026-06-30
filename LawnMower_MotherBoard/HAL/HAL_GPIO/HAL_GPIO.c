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
typedef enum {
    E_BLADE_OFF = 0,
    E_BLADE_STARTING,
    E_BLADE_ON,
    E_BLADE_BRAKING
} BladeState_t;

static BladeState_t ge_bladeState;
static uint8_t      gu8_bladeBrakeRequest;

/* Temps de démarrage lame avant de considérer qu'elle tourne (en ticks) */
#define BLADE_START_DELAY_TICKS  50   /* à ajuster selon ton timer */
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_Init()
{
	LLD_GPIO_Init();
	gu8_bladeBrakeRequest = 0;
	ge_bladeState = E_BLADE_OFF;
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

/*--------------------------------------------------------------------------*/
/* Demande d'allumage normal (depuis FSM_Operative Moving)                 */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_RequestBladeOn(void)
{
    if (ge_bladeState == E_BLADE_OFF)
    {
        ge_bladeState = E_BLADE_STARTING;
        gu8_bladeBrakeRequest = 0;
        /* Frein désactivé, ENABLE actif */
        LLD_GPIO_ClearPin(E_MOTOR_BLADE_BRAKE);
        LLD_GPIO_WritePin(E_MOTOR_BLADE_ENABLE);
    }
}

/*--------------------------------------------------------------------------*/
/* Arrêt normal — roue libre, pas de frein (depuis FSM hors soulèvement)  */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_RequestBladeOff(void)
{
    ge_bladeState = E_BLADE_OFF;
    gu8_bladeBrakeRequest = 0;
    LLD_GPIO_ClearPin(E_MOTOR_BLADE_BRAKE);
    LLD_GPIO_ClearPin(E_MOTOR_BLADE_ENABLE);
}

/*--------------------------------------------------------------------------*/
/* Arrêt d'urgence avec frein — soulèvement uniquement                    */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_RequestBladeBrake(void)
{
    if (ge_bladeState != E_BLADE_OFF)
    {
        ge_bladeState = E_BLADE_BRAKING;
        gu8_bladeBrakeRequest = 1;
        /* Coupe ENABLE d'abord, puis active le frein */
        LLD_GPIO_ClearPin(E_MOTOR_BLADE_ENABLE);
        LLD_GPIO_WritePin(E_MOTOR_BLADE_BRAKE);
    }
}

/*--------------------------------------------------------------------------*/
/* Libération du frein après fin de soulèvement → repasse en BLADE_OFF    */
/* FSM_Operative devra appeler RequestBladeOn pour rallumer                */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_RequestBladeRelease(void)
{
    if (ge_bladeState == E_BLADE_BRAKING)
    {
        ge_bladeState = E_BLADE_OFF;
        LLD_GPIO_ClearPin(E_MOTOR_BLADE_BRAKE);
        LLD_GPIO_ClearPin(E_MOTOR_BLADE_ENABLE);
    }
}
