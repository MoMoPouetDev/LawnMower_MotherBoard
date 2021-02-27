//
//  adc.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 07/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>

#include "adc.h"

uint16_t ADC_read(uint8_t adcChannel)
{
    adcChannel &= 0x03;
    ADMUX = ( ADMUX & 0xFC ) | adcChannel; // Mask pour selection de l'adc
    
    ADCSRA |= (1<<ADSC); // Start Conversion
    
    while (ADCSRA & (1<<ADSC)); // Wait fin conversion
    
    return ADC;
}