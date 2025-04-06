/**
 * @file LLD_ADC.c
 * @author MVE
 * @brief Specific ADC driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <avr/io.h>

#include "LLD_ADC.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ADC ...                                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/
void LLD_ADC_Init(void)
{
    ADMUX |= (1<<REFS0);
    ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1); // Enable ADC, Interrup et 64 prescale
}

uint8_t LLD_ADC_ReadConversionValue(uint8_t u8_adcChannel, uint16_t* pu16_adcValue)
{
	uint8_t u8_adcState = 0;
	uint8_t u8_returnValue = 0;

	switch (u8_adcState)
	{
		case 0:
			u8_adcChannel &= 0x03;
    		ADMUX = ( ADMUX & 0xFC ) | u8_adcChannel; // Mask pour selection de l'adc
			ADCSRA |= (1<<ADSC); // Start Conversion
			u8_adcState++;
			break;

		case 1:
			if ((ADCSRA & (1<<ADSC)) == 0) // Wait fin conversion
			{
				(*pu16_adcValue) = ADC;
				u8_adcState = 0;
				u8_returnValue = 1;
			}
			break;
		
		default:
			break;
	}
        
    return u8_returnValue;
}
