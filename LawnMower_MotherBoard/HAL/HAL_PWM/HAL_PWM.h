/*
 * HAL_PWM.h
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_PWM_HAL_PWM_H_
#define HAL_HAL_PWM_HAL_PWM_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "LLD_PWM.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define MOTOR_LEFT_CHANNEL  LLD_PWM_CHANNEL_A
#define MOTOR_LEFT_FORWARD LLD_PWM_PWM1_MODULE1
#define MOTOR_RIGHT_CHANNEL  LLD_PWM_CHANNEL_B
#define MOTOR_RIGHT_FORWARD LLD_PWM_PWM1_MODULE0

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_PWM_Init(void);
void HAL_PWM_Stop(void);
void HAL_PWM_Forward(uint8_t u8_speedLeft, uint8_t u8_speedRight);
void HAL_PWM_Turn(uint8_t u8_turn);

#endif /* HAL_HAL_PWM_HAL_PWM_H_ */
