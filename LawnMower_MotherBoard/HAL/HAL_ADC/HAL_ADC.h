/*
 * HAL_ADC.h
 *
 *  Created on: 17 août 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_ADC_HAL_ADC_H_
#define HAL_HAL_ADC_HAL_ADC_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_ADC_Init(void);
void HAL_ADC_ReadValueGuideWire(void);
uint16_t HAL_ADC_GetLeftWireValue(void);
uint16_t HAL_ADC_GetRightWireValue(void);

#endif /* HAL_HAL_ADC_HAL_ADC_H_ */
