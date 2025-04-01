/*
 * RUN_ADC.c
 *
 *  Created on: 23 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_ADC.h"
#include "RUN_ADC.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_ADC_Init(void)
{
	HAL_ADC_Init();
}

void RUN_ADC_ReadValue(void)
{
	HAL_ADC_ReadValueGuideWire();
}
