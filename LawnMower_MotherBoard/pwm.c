//
//  pwm.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 30/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>

#include "constant.h"
#include "pwm.h"

void PWM_forward(uint8_t speed)
{
    OCR0B = 0x00;
    OCR2B = 0x00;
    
    OCR0A = (( 0xFF / 100 ) * speed);
    OCR2A = (( 0xFF / 100 ) * speed);
    
    PORTD |= (1<<ENABLE_MOTOR_1) | (1<<ENABLE_MOTOR_2);
}

void PWM_forward_turn(uint8_t speed_ML, uint8_t speed_MR)
{
    OCR0B = 0x00;
    OCR2B = 0x00;
    
    OCR0A = (( 0xFF / 100 ) * speed_ML);
    OCR2A = (( 0xFF / 100 ) * speed_MR);
    
    PORTD |= (1<<ENABLE_MOTOR_1) | (1<<ENABLE_MOTOR_2);
}

void PWM_reverse(uint8_t speed)
{
    OCR0A = 0x00;
    OCR2A = 0x00;
    
    OCR0B = (( 0xFF / 100 ) * speed);
    OCR2B = (( 0xFF / 100 ) * speed);
    
    PORTD |= (1<<ENABLE_MOTOR_1) | (1<<ENABLE_MOTOR_2);
}

void PWM_right()
{
    OCR0B = 0x00;
    OCR2A = 0x00;

    OCR0A = (( 0xFF / 100 ) * LOW_SPEED);
    OCR2B = (( 0xFF / 100 ) * LOW_SPEED);
    
    PORTD |= (1<<ENABLE_MOTOR_1) | (1<<ENABLE_MOTOR_2);
}

void PWM_left()
{
    OCR0A = 0x00;
    OCR2B = 0x00;
    
    OCR0B = (( 0xFF / 100 ) * LOW_SPEED);
    OCR2A = (( 0xFF / 100 ) * LOW_SPEED);
    
    PORTD |= (1<<ENABLE_MOTOR_1) | (1<<ENABLE_MOTOR_2);
}

void PWM_stop()
{
    PORTD &= ~(1<<ENABLE_MOTOR_1) & ~(1<<ENABLE_MOTOR_2);
    
    OCR0A = 0x00;
    OCR2A = 0x00;
    
    OCR0B = 0x00;
    OCR2B = 0x00;
}
