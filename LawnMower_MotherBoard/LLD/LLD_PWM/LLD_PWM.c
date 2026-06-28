/**
 * @file LLD_PWM.c
 * @author MVE
 * @date 2019-03-20
 * @brief Specific PWM driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <avr/io.h>

#include "LLD_PWM.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES PWM ...                                                */
/*--------------------------------------------------------------------------*/
#define HIGH_SPEED 100
#define MIDDLE_SPEED 90
#define LOW_SPEED 80
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/
/**
* @brief		Pulse Width Modulation initialization
* @return		void
* @details
**/
void LLD_PWM_Init(void)
{
	/***** Moteur 1 - Gauche *****/
    TCCR0A |= (1<<COM0A1) | (1<<WGM01) | (1<<WGM00); // Fast PWM
    TCCR0B |= (1<<CS00); // No Prescale
    
    OCR0A = 0x00;
    
	/***** Moteur 2 - Droit *****/
    TCCR2A |= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); // Fast PWM
    TCCR2B |= (1<<CS20); // No Prescale
    
    OCR2A = 0x00;
}

//PWM Motor 1 PD6
//PWM Motor 2 PB3
void LLD_PWM_Forward(uint8_t speed_ML, uint8_t speed_MR)
{
    OCR0A = (( 0xFF / 100 ) * speed_ML);
    OCR2A = (( 0xFF / 100 ) * speed_MR);
}

void LLD_PWM_Right(void)
{
    OCR0A = (( 0xFF / 100 ) * HIGH_SPEED);
    OCR2A = (( 0xFF / 100 ) * HIGH_SPEED);
}

void LLD_PWM_Left(void)
{
    OCR0B = (( 0xFF / 100 ) * HIGH_SPEED);
    OCR2A = (( 0xFF / 100 ) * HIGH_SPEED);
}

void LLD_PWM_Stop(void)
{
	OCR0A = 0x00;
    OCR2A = 0x00;
}
