/*
 * HAL_UART.h
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_UART_HAL_UART_H_
#define HAL_HAL_UART_HAL_UART_H_

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "stdint.h"
#include "LLD_UART.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define BLE_BAUDRATE   9600U
#define GPS_BAUDRATE   115200U
#define DEBUG_BAUDRATE   115200U
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_UART_Init(void);
void HAL_UART_BleInit(void);
uint8_t HAL_UART_ReceiveCommand(uint8_t* pu8_RxBuffer, uint8_t u8_size);
uint8_t HAL_UART_SendCommand(uint8_t* pu8_buffer, uint8_t u8_bufferSize);
void HAL_UART_SendString(const char* pc_string);

#endif /* HAL_HAL_UART_HAL_UART_H_ */
