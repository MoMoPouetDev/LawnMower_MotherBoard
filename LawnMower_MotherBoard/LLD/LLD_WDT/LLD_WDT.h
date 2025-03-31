/**
 * @file LLD_WDT.h
 * @author SPR
 * @brief Header file for watchdog time peripheral
 * @details
**/

#ifndef LLD_WDT_H_
#define LLD_WDT_H_

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... GLOBAL FUNCTIONS DECLARATIONS ...                                   */
/*--------------------------------------------------------------------------*/

void LLD_WDT_Init(uint8_t u8_OutputValue);
void LLD_WDT_Refresh(void);

#endif /* LLD_WDT_H_ */
