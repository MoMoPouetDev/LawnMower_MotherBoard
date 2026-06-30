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
void HAL_GPIO_UpdateWheelState(MotorState e_wheelState);
uint8_t HAL_GPIO_GetFlagButton(GPIO e_flagButton);
uint8_t HAL_GPIO_GetFlagBumper(GPIO e_flagBumper);
void HAL_GPIO_RequestBladeOn(void);
void HAL_GPIO_RequestBladeOff(void);
void HAL_GPIO_RequestBladeBrake(void);
void HAL_GPIO_RequestBladeRelease(void);

#endif /* HAL_HAL_GPIO_HAL_GPIO_H_ */
