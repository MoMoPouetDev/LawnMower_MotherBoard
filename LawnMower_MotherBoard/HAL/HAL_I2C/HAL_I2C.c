/*
 * HAL_I2C.c
 *
 *  Created on: 12 sept. 2022
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include "HAL_I2C.h"
#include "LLD_I2C.h"


/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
#define I2C_BAUDRATE   400000UL
#define MASTER_ADDR 0x10
#define SLAVE_SENSOR_ADDR (0x10<<1)
#define COMPASS_ADDR (0x60<<1)
/*** Compass ***/
#define ADDR_DATA_COMPASS_MSB 0x02
#define ADDR_DATA_COMPASS_LSB 0x03
/** Accelerometer ***/
#define ADDR_DATA_ACCELEROMETER_PITCH_MSB 0x1C
#define ADDR_DATA_ACCELEROMETER_PITCH_LSB 0x1D
#define ADDR_DATA_ACCELEROMETER_ROLL 0x05
/*** Slave ***
#define ADDR_SENSOR_V 0x01
#define ADDR_SENSOR_A 0x02
#define ADDR_SENSOR_DOCK 0x03
#define ADDR_SENSOR_RAIN 0x04
#define ADDR_SONAR_FC 0x05
#define ADDR_SONAR_FL 0x06
#define ADDR_SONAR_FR 0x07
#define ADDR_GPS_TIME_HOURS 0x08
#define ADDR_GPS_TIME_MINUTES 0x09
#define ADDR_GPS_DATE_DAYS 0x0A
#define ADDR_GPS_DATE_MONTHS 0x0B
#define ADDR_GPS_LONG_DEG 0x0C
#define ADDR_GPS_LONG_MIN 0x0D
#define ADDR_GPS_LONG_DEC_MSB 0x0E
#define ADDR_GPS_LONG_DEC_B 0x0F
#define ADDR_GPS_LONG_DEC_LSB 0x10
#define ADDR_GPS_LAT_DEG 0x11
#define ADDR_GPS_LAT_MIN 0x12
#define ADDR_GPS_LAT_DEC_MSB 0x13
#define ADDR_GPS_LAT_DEC_B 0x14
#define ADDR_GPS_LAT_DEC_LSB 0x15
#define ADDR_TIME_TO_MOW 0x16
#define ADDR_LED_STATUS 0x17
#define ADDR_UNKNOWN_DATA 0x00 */

/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/

void HAL_I2C_Init(void)
{
	LLD_I2C_Init();
}

void HAL_I2C_ReadAccel(int16_t* ps16_pitch, int8_t* ps8_roll)
{
	uint8_t u8_pitchMSB = 0;
	uint8_t u8_pitchLSB = 0;

	u8_pitchMSB = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_ACCELEROMETER_PITCH_MSB);
	u8_pitchLSB = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_ACCELEROMETER_PITCH_LSB);
	*ps16_pitch = (int16_t)((uint16_t)(u8_pitchMSB << 8) | (uint16_t)u8_pitchLSB);
	*ps8_roll = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_ACCELEROMETER_ROLL);
}

uint16_t HAL_I2C_ReadCompass(void)
{
	uint8_t u8_angleMSB = 0;
	uint8_t u8_angleLSB = 0;
	uint16_t u16_angleValue = 0;

	u8_angleMSB = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_MSB);
	u8_angleLSB = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_LSB);

	u16_angleValue = (((uint16_t)u8_angleMSB<<8) | (uint16_t)u8_angleLSB)/10;

	return u16_angleValue;
}

uint8_t HAL_I2C_ReadSlave(uint8_t* pu8_RxBuff, uint8_t* pu8_Size)
{
	static E_SLAVE_READ_DATA _e_slaveReadData = 0;
	static uint8_t _u8_slaveState = 0;
	uint8_t u8_ReturnValue = 0;

	switch (_u8_slaveState)
	{
		case 0:
			*(pu8_RxBuff+_e_slaveReadData) = LLD_I2C_Read(SLAVE_SENSOR_ADDR, _e_slaveReadData);
			if(_e_slaveReadData == (E_SLAVE_READ_DATA_NUMBER-1))
			{
				_u8_slaveState++;
			}
			else
			{
				_e_slaveReadData++;
			}
			break;

		case 1:
			*pu8_Size = _e_slaveReadData;
			_e_slaveReadData = 0;
			_u8_slaveState = 0;
			u8_ReturnValue = 1;
			break;
		
		default:
			_u8_slaveState = 0;
			_e_slaveReadData = 0;
			break;
	}
	return u8_ReturnValue;
}

uint8_t HAL_I2C_WriteSlave(uint8_t u8_mowerState)
{
	static uint8_t _u8_slaveState = 0;
	uint8_t u8_ReturnValue = 0;

	switch (_u8_slaveState)
	{
		case 0:
			LLD_I2C_Write(SLAVE_SENSOR_ADDR, E_SLAVE_WRITE_DATA_LED_STATUS, u8_mowerState);
			_u8_slaveState++;
			break;

		case 1:
			_u8_slaveState = 0;
			u8_ReturnValue = 1;
			break;
		
		default:
			_u8_slaveState = 0;
			break;
	}

	return u8_ReturnValue;
}
