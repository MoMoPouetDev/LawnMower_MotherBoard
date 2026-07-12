/*
 * RUN_UART.h
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef RUN_RUN_UART_RUN_UART_H_
#define RUN_RUN_UART_RUN_UART_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdint.h>
#include "HAL_UART.h"
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void RUN_UART_Init(void);
void RUN_UART_DebugSendStatus(void);

#endif /* RUN_RUN_UART_RUN_UART_H_ */
