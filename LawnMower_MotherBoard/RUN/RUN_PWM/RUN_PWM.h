/*
 * RUN_PWM.h
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_PWM_RUN_PWM_H_
#define RUN_RUN_PWM_RUN_PWM_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_PWM_Init(void);
void RUN_PWM_Stop(void);
void RUN_PWM_Backward(uint8_t u8_speed);
void RUN_PWM_Right(void);
void RUN_PWM_Forward(uint8_t u8_speedLeft, uint8_t u8_speedRight);
void RUN_PWM_Left(void);

#endif /* RUN_RUN_PWM_RUN_PWM_H_ */
