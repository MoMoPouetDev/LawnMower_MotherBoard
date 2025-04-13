/*
 * HAL_GPIO.h
 *
 *  Created on: 19 août 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_GPIO_HAL_GPIO_H_
#define HAL_HAL_GPIO_HAL_GPIO_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "LLD_GPIO.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
typedef enum
{
    OFF,
    ON
}Etat;

typedef enum
{
    STOP,
	FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
}MotorState;

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_GPIO_Init(void);
void HAL_GPIO_UpdateBladeState(Etat e_bladeState);
void HAL_GPIO_BladeState(Etat e_bladeState);
void HAL_GPIO_UpdateWheelState(MotorState e_wheelState);
uint8_t HAL_GPIO_GetFlagButton(GPIO e_flagButton);
uint8_t HAL_GPIO_GetFlagBumper(GPIO e_flagBumper);

#endif /* HAL_HAL_GPIO_HAL_GPIO_H_ */
