/*
 * HAL_Sonar.h
 *
 *  Created on: 26 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_SONAR_HAL_SONAR_H_
#define HAL_HAL_SONAR_HAL_SONAR_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define SONAR_WARN 30
#define SONAR_LIMITE 20
#define SONAR_ERR 10
#define SONAR_DIST_ERR 999
#define TIMER1_OVERFLOW 65535
/*** Calcul of value timer 343 m/s -> 34300 cm/s
 dist = (speedSound*TIMER)/2 = (34300*TIMER)/2 = 17150*TIMER = 17150 * (TIMER_VALUE * 0.125 * 10^-6)
 dist = (speedSound*TIMER)/2 = (34300*TIMER)/2 = 17150*TIMER = 17150 * (TIMER_VALUE * 0.017 * 10^-6)
 ***/
//#define TIMER_DISTANCE 466.47 8MHz
#define TIMER_DISTANCE 29.15 //60MHz
#define THRESHOLD_8_BITS 0xFE
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_Sonar_Init(void);
void HAL_Sonar_Distance(void);
void HAL_Sonar_GetDistance(uint8_t* u8_distFC, uint8_t* u8_distFL, uint8_t* u8_distFR);

#endif /* HAL_HAL_SONAR_HAL_SONAR_H_ */
