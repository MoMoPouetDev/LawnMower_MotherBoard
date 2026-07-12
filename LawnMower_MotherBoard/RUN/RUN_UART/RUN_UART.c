/*
 * RUN_UART.c
 *
 *  Created on: 19 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <string.h>
#include "RUN_Sensors.h"
#include "RUN_UART.h"

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
static void _RUN_UART_UintToString(uint8_t u8_value, char* pc_buffer);
static void _RUN_UART_Uint32ToStringPadded(uint32_t u32_value, uint8_t u8_digits, char* pc_buffer);

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS ...                                           */
/*--------------------------------------------------------------------------*/
void RUN_UART_Init(void)
{
    HAL_UART_Init();
}

void RUN_UART_DebugSendStatus(void)
{
	static uint16_t _u16_slaveCpt = 0;
#ifdef DEBUG_UART
	if (_u16_slaveCpt >= 100)
	{
		static char tc_txBuffer[96] = {0};
		uint8_t u8_distanceSonarFC = 0;
		uint8_t u8_distanceSonarFL = 0;
		uint8_t u8_distanceSonarFR = 0;
		uint8_t u8_batteryPercent = 0;
		char tc_buffer[8] = {0};
		float f_latitude = 0.0f;
		float f_longitude = 0.0f;
		uint8_t u8_negativeLat = 0;
		uint8_t u8_negativeLong = 0;
		uint16_t u16_intLat = 0;
		uint16_t u16_intLong = 0;
		uint32_t u32_decLat = 0;
		uint32_t u32_decLong = 0;

		u8_distanceSonarFC = RUN_Sensors_GetDistanceSonarFC();
		u8_distanceSonarFL = RUN_Sensors_GetDistanceSonarFL();
		u8_distanceSonarFR = RUN_Sensors_GetDistanceSonarFR();
		u8_batteryPercent = RUN_Sensors_GetBatteryPercent();
		f_latitude = RUN_Sensors_GetLatitude();
		f_longitude = RUN_Sensors_GetLongitude();

		/* Format latitude */
		if (f_latitude < 0.0f)
		{
			u8_negativeLat = 1;
			f_latitude = -f_latitude;
		}
		u16_intLat = (uint16_t)f_latitude;
		u32_decLat = (uint32_t)((f_latitude - (float)u16_intLat) * 1000000.0f);

		/* Format longitude */
		if (f_longitude < 0.0f)
		{
			u8_negativeLong = 1;
			f_longitude = -f_longitude;
		}
		u16_intLong = (uint16_t)f_longitude;
		u32_decLong = (uint32_t)((f_longitude - (float)u16_intLong) * 1000000.0f);

		/* Build string manuellement */
		tc_txBuffer[0] = '\0';

		strcat(tc_txBuffer, "SONAR_FC=");
		_RUN_UART_UintToString(u8_distanceSonarFC, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);

		strcat(tc_txBuffer, ";SONAR_FL=");
		_RUN_UART_UintToString(u8_distanceSonarFL, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);

		strcat(tc_txBuffer, ";SONAR_FR=");
		_RUN_UART_UintToString(u8_distanceSonarFR, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);

		strcat(tc_txBuffer, ";GPS_LAT=");
		if (u8_negativeLat != 0)
		{
			strcat(tc_txBuffer, "-");
		}
		_RUN_UART_UintToString((uint8_t)u16_intLat, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);
		strcat(tc_txBuffer, ".");
		_RUN_UART_Uint32ToStringPadded(u32_decLat, 6, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);

		strcat(tc_txBuffer, ";GPS_LONG=");
		if (u8_negativeLong != 0)
		{
			strcat(tc_txBuffer, "-");
		}
		_RUN_UART_UintToString((uint8_t)u16_intLong, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);
		strcat(tc_txBuffer, ".");
		_RUN_UART_Uint32ToStringPadded(u32_decLong, 6, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);

		strcat(tc_txBuffer, ";BAT=");
		_RUN_UART_UintToString(u8_batteryPercent, tc_buffer);
		strcat(tc_txBuffer, tc_buffer);

		strcat(tc_txBuffer, "\r\n");

		HAL_UART_SendString(tc_txBuffer);

		_u16_slaveCpt = 0;
	}
	else
	{
		_u16_slaveCpt++;
	}
#else
    return;
#endif
}

static void _RUN_UART_UintToString(uint8_t u8_value, char* pc_buffer)
{
    uint8_t u8_temp = u8_value;
    uint8_t u8_digits = 0;
    uint8_t u8_result[4] = {0};
    uint8_t u8_i = 0;

    if (u8_temp == 0)
    {
        pc_buffer[0] = '0';
        pc_buffer[1] = '\0';
        return;
    }

    while (u8_temp > 0)
    {
        u8_result[u8_digits++] = (uint8_t)(u8_temp % 10);
        u8_temp = (uint8_t)(u8_temp / 10);
    }

    for (u8_i = 0; u8_i < u8_digits; u8_i++)
    {
        pc_buffer[u8_i] = (char)(u8_result[u8_digits - u8_i - 1] + '0');
    }
    pc_buffer[u8_digits] = '\0';
}

static void _RUN_UART_Uint32ToStringPadded(uint32_t u32_value, uint8_t u8_digits, char* pc_buffer)
{
    uint32_t u32_temp = u32_value;
    uint8_t u8_i = 0;

    for (u8_i = 0; u8_i < u8_digits; u8_i++)
    {
        pc_buffer[u8_digits - u8_i - 1] = (char)('0' + (u32_temp % 10));
        u32_temp /= 10;
    }
    pc_buffer[u8_digits] = '\0';
}