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

//PWM Motor 1 PD6
//PWM Motor 2 PB3
void PWM_forward(uint8_t speed)
{    
    OCR0A = (( 0xFF / 100 ) * speed);
    OCR2A = (( 0xFF / 100 ) * speed);
    
	PORTD &= ~(1<<ENABLE_REVERSE_MOTOR_1) & ~(1<<ENABLE_REVERSE_MOTOR_2);
    PORTD |= (1<<ENABLE_FORWARD_MOTOR_1) | (1<<ENABLE_FORWARD_MOTOR_2);
}

void PWM_forward_turn(uint8_t speed_ML, uint8_t speed_MR)
{
    OCR0A = (( 0xFF / 100 ) * speed_ML);
    OCR2A = (( 0xFF / 100 ) * speed_MR);
    
    PORTD &= ~(1<<ENABLE_REVERSE_MOTOR_1) & ~(1<<ENABLE_REVERSE_MOTOR_2);
    PORTD |= (1<<ENABLE_FORWARD_MOTOR_1) | (1<<ENABLE_FORWARD_MOTOR_2);
}

void PWM_reverse(uint8_t speed)
{
    OCR0A = (( 0xFF / 100 ) * speed);
    OCR2A = (( 0xFF / 100 ) * speed);
    
    PORTD &= ~(1<<ENABLE_FORWARD_MOTOR_1) & ~(1<<ENABLE_FORWARD_MOTOR_2);
    PORTD |= (1<<ENABLE_REVERSE_MOTOR_1) | (1<<ENABLE_REVERSE_MOTOR_2);
}

void PWM_right()
{
    OCR0A = (( 0xFF / 100 ) * HIGH_SPEED);
    OCR2A = (( 0xFF / 100 ) * HIGH_SPEED);
    
    PORTD &= ~(1<<ENABLE_REVERSE_MOTOR_1) & ~(1<<ENABLE_FORWARD_MOTOR_2);
    PORTD |= (1<<ENABLE_FORWARD_MOTOR_1) | (1<<ENABLE_REVERSE_MOTOR_2);
}

void PWM_left()
{
    OCR0B = (( 0xFF / 100 ) * HIGH_SPEED);
    OCR2A = (( 0xFF / 100 ) * HIGH_SPEED);
    
    PORTD &= ~(1<<ENABLE_FORWARD_MOTOR_1) & ~(1<<ENABLE_REVERSE_MOTOR_2);
    PORTD |= (1<<ENABLE_REVERSE_MOTOR_1) | (1<<ENABLE_FORWARD_MOTOR_2);
}

void PWM_stop()
{
	OCR0A = 0x00;
    OCR2A = 0x00;
	
	PORTD &= ~(1<<ENABLE_FORWARD_MOTOR_1) & ~(1<<ENABLE_FORWARD_MOTOR_2);
	PORTD &= ~(1<<ENABLE_REVERSE_MOTOR_1) & ~(1<<ENABLE_REVERSE_MOTOR_2);
}
