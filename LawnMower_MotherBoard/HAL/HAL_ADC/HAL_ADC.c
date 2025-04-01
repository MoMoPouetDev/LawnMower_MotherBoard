/*
 * HAL_ADC.c
 *
 *  Created on: 17 août 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>

#include "HAL_ADC.h"
#include "LLD_ADC.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
/*** Convertisseur Analogique Numerique ***/
#define PIN_ADC0_LEFT_GUIDEWIRE 0
#define PIN_ADC1_RIGHT_GUIDEWIRE 1
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static uint16_t gu16_adcValueLeftWire;
static uint16_t gu16_adcValueRightWire;
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_ADC_Init()
{
	gu16_adcValueLeftWire = 0;
	gu16_adcValueRightWire = 0;
	LLD_ADC_Init();
}

void HAL_ADC_ReadValueGuideWire(void)
{
	uint8_t u8_adcReturn = 0;
	uint8_t u8_adcState = 0;
	uint16_t u16_adcValue = 0;

	switch (u8_adcState)
	{
	case 0:
		u8_adcReturn = LLD_ADC_ReadConversionValue(PIN_ADC0_LEFT_GUIDEWIRE, &u16_adcValue);
		if (u8_adcReturn != 0)
		{
			u16_adcValue = gu16_adcValueLeftWire;
			u8_adcState = 1;
		}
		break;
	
	case 1:
		u8_adcReturn = LLD_ADC_ReadConversionValue(PIN_ADC1_RIGHT_GUIDEWIRE, &u16_adcValue);
		if (u8_adcReturn != 0)
		{
			u16_adcValue = gu16_adcValueRightWire;
			u8_adcState = 0;
		}
		break;
	
	default:
		break;
	}
}

uint16_t HAL_ADC_GetLeftWireValue()
{
	return gu16_adcValueLeftWire;
}

uint16_t HAL_ADC_GetRightWireValue()
{
	return gu16_adcValueRightWire;
}
