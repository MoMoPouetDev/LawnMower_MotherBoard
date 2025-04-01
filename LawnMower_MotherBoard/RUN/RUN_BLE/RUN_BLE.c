/*
 * RUN_BLE.c
 *
 *  Created on: 15 APR 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_GPS.h"
#include "HAL_GPIO.h"
#include "HAL_UART.h"
#include "RUN_BLE.h"
#include "RUN_Sensors.h"
#include "RUN_Mower.h"
/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_BLE_Init()
{
	HAL_UART_BleInit();
}

void RUN_BLE_SendStatus()
{
	static uint8_t tu8_txBuffer[19] = {0};
	static uint8_t u8_uartState = 0;
	uint8_t u8_uartReturnState = 0;
	Coordinates tLatitude;
	Coordinates tLongitude;
	EtatMower eEtatMower;
	ErrorMower eErrorMower;
	uint8_t uBattery;
	uint8_t uHoursGPS;
	uint8_t uMinutesGPS;
	uint8_t uDaysGPS;
	uint8_t uMonthsGPS;
	uint8_t angleLSB, angleMSB;
	uint16_t angleW;

	eEtatMower = HAL_GPIO_GetEtatMower();
	eErrorMower = HAL_GPIO_GetErrorMower();
	uBattery = RUN_Sensors_GetBatteryPercent();
	HAL_GPS_GetStructCoordinates(&tLatitude, &tLongitude);

	angleW = RUN_Mower_GetCurrentAngle();
	angleLSB = angleW & 0xFF;
	angleMSB = (angleW >> 8) & 0xFF;

	switch (u8_uartState)
	{
		case 0:
			tu8_txBuffer[0] = eEtatMower;
			tu8_txBuffer[1] = eErrorMower;
			tu8_txBuffer[2] = uBattery;

			tu8_txBuffer[3] = tLatitude.degrees;
			tu8_txBuffer[4] = tLatitude.minutes;
			tu8_txBuffer[5] = tLatitude.decimalMSB;
			tu8_txBuffer[6] = tLatitude.decimalB;
			tu8_txBuffer[7] = tLatitude.decimalLSB;

			tu8_txBuffer[8] = tLongitude.degrees;
			tu8_txBuffer[9] = tLongitude.minutes;
			tu8_txBuffer[10] = tLongitude.decimalMSB;
			tu8_txBuffer[11] = tLongitude.decimalB;
			tu8_txBuffer[12] = tLongitude.decimalLSB;

			tu8_txBuffer[13] = uHoursGPS;
			tu8_txBuffer[14] = uMinutesGPS;
			tu8_txBuffer[15] = uDaysGPS;
			tu8_txBuffer[16] = uMonthsGPS;

			tu8_txBuffer[17] = angleMSB;
			tu8_txBuffer[18] = angleLSB;

			u8_uartState++;
			break;

		case 1:
			u8_uartReturnState = HAL_UART_SendCommand(tu8_txBuffer , 19);
			if (u8_uartReturnState != 0)
			{
				u8_uartState = 0;
			}
			break

		default:
			u8_uartState = 0;
			break;
	}
}

void RUN_BLE_ReceiveStatus()
{
	CommandMower e_commandMower;
    uint8_t u8_returnValueUART = 0;
	uint8_t tu8_rxBuffer[1] = { 0 };

    u8_returnValueUART = HAL_UART_ReceptionBLE(tu8_rxBuffer, 1);

    if (u8_returnValueUART)
    {
    	e_commandMower = tu8_rxBuffer[0];
    	switch(e_commandMower)
		{
			case START_COMMAND:
				break;

			case STOP_COMMAND:
				break;

			case FORCE_START_COMMAND:
				break;

			case DOCK_ON_COMMAND:
				break;

			case DOCK_OFF_COMMAND:
				break;

			case RAIN_ON_COMMAND:
				break;

			case RAIN_OFF_COMMAND:
				break;

			default:
				break;
		}
	}

}
