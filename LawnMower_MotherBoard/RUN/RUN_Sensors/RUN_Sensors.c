/*
 * RUN_Sensors.c
 *
 *  Created on: 25 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_I2C.h"
#include "RUN_Mower.h"
#include "RUN_Sensors.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
static uint8_t gu8_batteryVoltage;
static uint8_t gu8_batteryAmp;
static Etat ge_dock;
static uint8_t gu8_distanceSonarFC;
static uint8_t gu8_distanceSonarFL;
static uint8_t gu8_distanceSonarFR;
static float gf_longitude;
static float gf_latitude;
static Etat ge_rain;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void RUN_Sensors_Init()
{
	ge_rain = OFF;
	ge_dock = OFF;
}

uint8_t RUN_Sensors_IsCharging()
{
	uint8_t u8_returnValue = 0;
	uint32_t u8_chargeValue;

	u8_chargeValue = gu8_batteryAmp;

	if (u8_chargeValue <= CHARGING_THRESHOLD)
	{
		u8_returnValue = 0;
	}
	else
	{
		u8_returnValue = 1;
	}

	return u8_returnValue;
}

int8_t RUN_Sensors_IsEnoughCharged()
{
	uint8_t battery = 0;
	int8_t u8_returnValue = 1;

	battery = RUN_Sensors_GetBatteryPercent();
	
	if (battery <= SENSOR_V_FAIBLE_WARN) 
	{
		if(battery <= SENSOR_V_EMPTY)
		{
			RUN_Mower_SetErrorMower(EMPTY_BATTERY);
			u8_returnValue = -1;
		}
		else if (battery <= SENSOR_V_FAIBLE_ERR)
		{
			RUN_Mower_SetErrorMower(VERY_LOW_BATTERY);
			u8_returnValue = 0;
		}
		else
		{
			RUN_Mower_SetErrorMower(LOW_BATTERY);
			u8_returnValue = 0;
		}
	}
	
	return u8_returnValue;
}

/**********************************************/
//	0%   | 9     | 1,8   | 2235
//	5%   | 9,9   | 1,98  | 2458
//	10%  | 10,8  | 2,16  | 2681
//	15%  | 10,95 |  2,19 | 2718
//	20%  | 11,1  | 2,22  | 2756
//	25%  | 11,18 | 2,236 | 2775
//	30%  | 11,25 | 2,25  | 2793
//	40%  | 11,37 | 2,274 | 2823
//	50%  | 11,49 | 2,298 | 2852
//	60%  | 11,61 | 2,322 | 2882
//	70%  | 11,76 | 2,352 | 2919
//	75%  | 11,84 | 2,368 | 2939
//	80%  | 11,91 | 2,382 | 2957
//	85%  | 12,1  | 2,42  | 3004
//	90%  | 12,3  | 2,46  | 3053
//	95%  | 12,45 | 2,49  | 3091
//	100% | 12,6  | 2,52  | 3128
/*********************************************/
uint8_t RUN_Sensors_GetBatteryPercent(void) 
{
	uint32_t uTension;
	uint8_t uPourcentage = 0;

	uTension = gu8_batteryVoltage;
	
	if(uTension <= 2300) { uPourcentage = 0; }
	else if(uTension <= 2458) { uPourcentage = 5; }
	else if(uTension <= 2681) { uPourcentage = 10; }
	else if(uTension <= 2718) { uPourcentage = 15; }
	else if(uTension <= 2756) { uPourcentage = 20; }
	else if(uTension <= 2775) { uPourcentage = 25; }
	else if(uTension <= 2793) { uPourcentage = 30; }
	else if(uTension <= 2823) { uPourcentage = 40; }
	else if(uTension <= 2852) { uPourcentage = 50; }
	else if(uTension <= 2882) { uPourcentage = 60; }
	else if(uTension <= 2919) { uPourcentage = 70; }
	else if(uTension <= 2939) { uPourcentage = 75; }
	else if(uTension <= 2957) { uPourcentage = 80; }
	else if(uTension <= 3004) { uPourcentage = 85; }
	else if(uTension <= 3053) { uPourcentage = 90; }
	else if(uTension <= 3091) { uPourcentage = 95; }
	else { uPourcentage = 100; }
	
	return uPourcentage;
}

Etat RUN_Sensors_GetRainState()
{
	return ge_rain;
}

void RUN_Sensors_SetRainState(Etat e_rainState)
{
	ge_rain = e_rainState;
}

Etat RUN_Sensors_GetDockState()
{
	return ge_dock;
}

void RUN_Sensors_SetDockState(Etat e_dockState)
{
	ge_dock = e_dockState;
}

void RUN_Sensors_ReadSlaveData(void)
{
	static uint8_t _tu8_rxBuffSlave[E_SLAVE_READ_DATA_NUMBER] = {0};
	static uint8_t _u8_rxBuffSlaveSize = 0;
	uint8_t u8_flagI2c = 0;
	uint32_t tempLatDecimal = 0;
	uint32_t tempLongDecimal = 0;
	char tempLat[9] = {0};
	char tempLong[9] = {0};

	u8_flagI2c = HAL_I2C_ReadSlave(_tu8_rxBuffSlave, &_u8_rxBuffSlaveSize);
	if (u8_flagI2c != 0)
	{
		gu8_batteryVoltage = _tu8_rxBuffSlave[E_SLAVE_READ_DATA_V];
		gu8_batteryAmp = _tu8_rxBuffSlave[E_SLAVE_READ_DATA_A];
		ge_dock = _tu8_rxBuffSlave[E_SLAVE_READ_DATA_DOCK];
		RUN_Mower_SetTimeToMow(_tu8_rxBuffSlave[E_SLAVE_READ_DATA_TIME_TO_MOW]);
		gu8_distanceSonarFC = _tu8_rxBuffSlave[E_SLAVE_READ_DATA_SONAR_FC];
		gu8_distanceSonarFL = _tu8_rxBuffSlave[E_SLAVE_READ_DATA_SONAR_FL];
		gu8_distanceSonarFR = _tu8_rxBuffSlave[E_SLAVE_READ_DATA_SONAR_FR];
	
		tempLongDecimal = ((uint32_t)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LONG_DEC_MSB] << 16) | ((uint32_t)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LONG_DEC_B] << 8) | ((uint32_t)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LONG_DEC_LSB]);
		sprintf(tempLong, "%d.%d",(int)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LONG_MIN], (int)tempLongDecimal);
		gf_longitude = (float)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LONG_DEG] + (atof(tempLong)/60.0);

		tempLatDecimal = ((uint32_t)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LAT_DEC_MSB] << 16) | ((uint32_t)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LAT_DEC_B] << 8) | ((uint32_t)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LAT_DEC_LSB]);
		sprintf(tempLat, "%d.%d",(int)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LAT_MIN], (int)tempLatDecimal);
		gf_latitude = (float)_tu8_rxBuffSlave[E_SLAVE_READ_DATA_GPS_LAT_DEG] + (atof(tempLat)/60.0);
	}
	
}
