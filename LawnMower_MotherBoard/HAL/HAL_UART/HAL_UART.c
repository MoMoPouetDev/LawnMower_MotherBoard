/*
 * HAL_UART.c
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_UART.h"
#include "HAL_GPS.h"
#include "LLD_UART.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static uint8_t _HAL_UART_SendCommand(char* pc_buffer, uint8_t u8_bufferSize);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_UART_Init()
{
	LLD_UART_Init();
}

void HAL_UART_BleInit()
{
	char commandAT[] = "AT";
	char commandRole[] = "AT+ROLE0";
	char commandUuid[] = "AT+UUID0xFFE0";
	char commandChar[] = "AT+CHAR0xFFE1";
	char commandName[] = "AT+NAMEMower";

	while (!(_HAL_UART_SendCommand(commandAT, strlen(commandAT))));
	while (!(_HAL_UART_SendCommand(commandRole, strlen(commandRole))));
	while (!(_HAL_UART_SendCommand(commandUuid, strlen(commandUuid))));
	while (!(_HAL_UART_SendCommand(commandChar, strlen(commandChar))));
	while (!(_HAL_UART_SendCommand(commandName, strlen(commandName))));
}

void HAL_UART_Reception()
{

}

void HAL_UART_SendStatus(uint8_t* tu8_uart_txBuff, uint8_t u8_size)
{
	static uint8_t u8_uartState = 0;

	switch (u8_uartState)
	{
		case 0:
			LLD_UART_Send(*tu8_uart_txBuff);
			u8_uartState = 1;
			break;
		case 1:
			u8_uartState = 0;
			break;
		default:
			u8_uartState = 0;
			break;
	}
}

uint8_t HAL_UART_ReceptionBLE(uint8_t* tu8_RxBuffer, uint8_t u8_size)
{
	static uint8_t tu8_uart_rxBuff[1] = {0};
	static uint8_t u8_uartState = 0;
	uint8_t u8_returnValue = 0;

	switch (u8_uartState)
	{
		case 0:
			LLD_UART_Receive(*tu8_uart_rxBuff);
			u8_uartState = 1;
			break;
		case 1:
			u8_uartState = 0;
			u8_returnValue = 1;
			memcpy(tu8_RxBuffer, tu8_uart_rxBuff, BUFFER_SIZE);
			break;
		default:
			u8_uartState = 0;
			break;
	}
	return u8_returnValue;
}

static uint8_t _HAL_UART_SendCommand(char* pc_buffer, uint8_t u8_bufferSize)
{
	static uint8_t u8_i = 0;
	uint8_t u8_returnValue = 0;
	
	if (u8_i < u8_bufferSize)
	{
		LLD_UART_Send(pc_buffer + u8_i);
		u8_i++;
	}
	else
	{
		u8_i = 0;
		u8_returnValue = 1;
	}

	return u8_returnValue;
}
