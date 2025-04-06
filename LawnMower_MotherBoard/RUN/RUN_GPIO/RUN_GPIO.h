/*
 * RUN_GPIO.h
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_GPIO_RUN_GPIO_H_
#define RUN_RUN_GPIO_RUN_GPIO_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "HAL_GPIO.h"
/*--------------------------------------------------------------------------*/
/*! ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_GPIO_Init(void);
void RUN_GPIO_DisableAllMotor(void);
void RUN_GPIO_DisableMotor(void);
void RUN_GPIO_UpdateBladeState(Etat e_bladeState);
uint8_t RUN_GPIO_GetStartButton(void);
uint8_t RUN_GPIO_GetStopButton(void);
void RUN_GPIO_UpdateLed(void);

#endif /* RUN_RUN_GPIO_RUN_GPIO_H_ */
