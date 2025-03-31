/**
 * @file LLD_WDT.c
 * @author SPR
 * @brief Specific watchdog driver
 * @details
**/

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/

#define LLD_WDT_USE_LOCALS
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "MIMXRT1062.h"
#include "MIMXRT1062_features.h"
#include "fsl_clock.h"

#include "LLD_WDT.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                   		*/
/*--------------------------------------------------------------------------*/

/*! @name Refresh sequence */
/*@{*/
#define WDOG_REFRESH_KEY (0xAAAA5555U)
/*@}*/

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
    uint16_t u16_Value        = 0U;
    uint32_t u32_primaskValue = 0U;

    /* Enable the Watchdog */
    u16_Value = WDOG_WCR_WDE(true) | WDOG_WCR_WT(u8_OutputValue) | WDOG_WCR_SRS_MASK | WDOG_WCR_WDA_MASK;

    /* Set configuration */
    CLOCK_EnableClock(kCLOCK_Wdog1);

    /* Disable the global interrupt to configure watchdog */
    u32_primaskValue = DisableGlobalIRQ();
    WDOG1->WCR    = u16_Value;
    EnableGlobalIRQ(u32_primaskValue);
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
    uint32_t u32_primaskValue = 0U;

    /* Disable the global interrupt to protect refresh sequence */
    u32_primaskValue = DisableGlobalIRQ();
    WDOG1->WSR    = WDOG_REFRESH_KEY & 0xFFFFU;
    WDOG1->WSR    = (WDOG_REFRESH_KEY >> 16U) & 0xFFFFU;
    EnableGlobalIRQ(u32_primaskValue);
}


