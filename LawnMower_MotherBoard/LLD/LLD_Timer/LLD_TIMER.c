/**
 * @file LLD_TIMER.c
 * @author ACR
 * @brief Specific TIMER driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_TIMER_USE_LOCALS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "LLD_TIMER.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/
void LLD_TIMER_Init(void)
{
    /* Init Timer 1 for Run Task tick */
	TCCR1B |= (1<<WGM12) | (1<<CS10); // CTC and No Prescale
	OCR1A = 0x1F40; // 16 bit
	TIMSK1 |= (1<<OCIE1A); // Enable Timer 1 Output Compare A Match Interrupt
}
