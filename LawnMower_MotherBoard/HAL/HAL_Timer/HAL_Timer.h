/*
 * HAL_Timer.h
 *
 *  Created on: 17 août 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_TIMER_H_
#define HAL_HAL_TIMER_H_
/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define TICK_TIME 1000
#define ADC_CONVERSION_TIME 10000
#define PULSE_TIME 12

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_Timer_Init(void);
void HAL_TIMER_StartSonarPulse(void);
uint32_t HAL_TIMER_ReadGptValue(void);

#endif /* HAL_HAL_TIMER_H_ */
