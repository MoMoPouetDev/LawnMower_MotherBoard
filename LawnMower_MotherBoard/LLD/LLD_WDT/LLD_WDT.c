/**
 * @file LLD_WDT.c
 * @author SPR
 * @brief Specific watchdog driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "LLD_WDT.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                   		*/
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DEFINITIONS ...                                     */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DEFINITIONS ...                                    */
/*--------------------------------------------------------------------------*/

/**
* @brief		Watchdog time initialization
* @param		u8_TimeoutValue : Timeout of watchdog, 0x00 = 0.5s, 0x01 = 1s, ... 0xFF = 128s.
* @return		void
* @details		WDOG1 used
**/
void LLD_WDT_Init(uint8_t u8_OutputValue)
{
    cli();
	wdt_reset();
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1); //1s
	sei();
}


/**
* @brief		Refresh watchdog time
* @param		void
* @return		void
* @details		WDOG1 used
* 				Disable the global interrupt to protect refresh sequence
**/
void LLD_WDT_Refresh(void)
{

}


