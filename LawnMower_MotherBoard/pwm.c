//
//  pwm.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 30/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>

#include "pwm.h"

void PWM_forward(uint8_t speed)
{
    OCR0B = 0x00;
    OCR2B = 0x00;
    
    OCR0A = (( 0xFF / 100 ) * speed);
    OCR2A = (( 0xFF / 100 ) * speed);
    
    PORTD |= (1<<PORTD4) | (1<<PORTD2);
}

void PWM_reverse(uint8_t speed)
{
    OCR0A = 0x00;
    OCR2A = 0x00;
    
    OCR0B = (( 0xFF / 100 ) * speed);
    OCR2B = (( 0xFF / 100 ) * speed);
    
    PORTD |= (1<<PORTD4) | (1<<PORTD2);
}

void PWM_right()
{
    OCR0B = 0x00;
    OCR2A = 0x00;

    OCR0A = (( 0xFF / 100 ) * 20);
    OCR2B = (( 0xFF / 100 ) * 20);
    
    PORTD |= (1<<PORTD4) | (1<<PORTD2);
}

void PWM_left()
{
    OCR0A = 0x00;
    OCR2B = 0x00;
    
    OCR0B = (( 0xFF / 100 ) * 20);
    OCR2A = (( 0xFF / 100 ) * 20);
    
    PORTD |= (1<<PORTD4) | (1<<PORTD2);
}

void PWM_stop()
{
    PORTD &= ~(1<<PORTD4) & ~(1<<PORTD2);
    
    OCR0A = 0x00;
    OCR2A = 0x00;
    
    OCR0B = 0x00;
    OCR2B = 0x00;
}