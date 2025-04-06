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
#define I2C_BAUDRATE   400000U
#define MASTER_ADDR 0x10
#define SLAVE_SENSOR_ADDR 0x02
#define COMPASS_ADDR (0x1E<<1)
#define ACCELEROMETRE_ADDR (0x53<<1)
/*** Compass ***/
#define ADDR_DATA_COMPASS_X_MSB 0x03
#define ADDR_DATA_COMPASS_X_LSB 0x04
#define ADDR_DATA_COMPASS_Z_MSB 0x05
#define ADDR_DATA_COMPASS_Z_LSB 0x06
#define ADDR_DATA_COMPASS_Y_MSB 0x07
#define ADDR_DATA_COMPASS_Y_LSB 0x08
/** Accelerometer ***/
#define ADDR_DATA_ACCELEROMETER_X_LSB 0x32
#define ADDR_DATA_ACCELEROMETER_X_MSB 0x33
#define ADDR_DATA_ACCELEROMETER_Y_LSB 0x34
#define ADDR_DATA_ACCELEROMETER_Y_MSB 0x35
#define ADDR_DATA_ACCELEROMETER_Z_LSB 0x36
#define ADDR_DATA_ACCELEROMETER_Z_MSB 0x37
/*** Slave ***/
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
#define ADDR_UNKNOWN_DATA 0x00
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_I2C_Init()
{
	LLD_I2C_Init();
}

void HAL_I2C_CompassInit()
{
	while(!(LLD_I2C_Write(COMPASS_ADDR, 0x00, 0x70)));
	while(!(LLD_I2C_Write(COMPASS_ADDR, 0x01, 0xE0)));
	while(!(LLD_I2C_Write(COMPASS_ADDR, 0x02, 0x00)));
}

void HAL_I2C_AccelInit()
{
	while(!(LLD_I2C_Write(ACCELEROMETRE_ADDR, 0x2D, 0x08)));
}

void HAL_I2C_Write()
{

}

uint8_t HAL_I2C_ReadAccel(uint8_t* pu8_RxBuff, uint8_t* pu8_Size)
{
	static uint8_t tu8_RxBuff[6] = {0};
	static uint8_t _u8_accelState = 0;
	uint8_t u8_accelReturnState = 0;
	uint8_t u8_ReturnValue = 0;
	uint8_t u8_Size = 6;

	*pu8_Size = u8_Size;

	switch (_u8_accelState)
	{
		case 0:
			u8_accelReturnState = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_X_LSB, &tu8_RxBuff[0]);
			if (u8_accelReturnState != 0)
			{
				_u8_accelState++;
			}
			break;

		case 1:
			u8_accelReturnState = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_X_MSB, &tu8_RxBuff[1]);
			if (u8_accelReturnState != 0)
			{
				_u8_accelState++;
			}
			break;
		
		case 2:
			u8_accelReturnState = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Y_LSB, &tu8_RxBuff[2]);
			if (u8_accelReturnState != 0)
			{
				_u8_accelState++;
			}
			break;

		case 3:
			u8_accelReturnState = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Y_MSB, &tu8_RxBuff[3]);
			if (u8_accelReturnState != 0)
			{
				_u8_accelState++;
			}
			break;

		case 4:
			u8_accelReturnState = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Z_LSB, &tu8_RxBuff[4]);
			if (u8_accelReturnState != 0)
			{
				_u8_accelState++;
			}
			break;
		
		case 5:
			u8_accelReturnState = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Z_MSB, &tu8_RxBuff[5]);
			if (u8_accelReturnState != 0)
			{
				_u8_accelState++;
			}
			break;

		case 6:
			for(int i = 0; i < u8_Size; i++)
			{
				pu8_RxBuff[i] = tu8_RxBuff[i];
			}
			_u8_accelState = 0;
			u8_ReturnValue = 1;
			break;
	
		default:
			_u8_accelState = 0;
			break;
	}

	return u8_ReturnValue;
}

uint8_t HAL_I2C_ReadCompass(uint8_t* pu8_RxBuff, uint8_t* pu8_Size)
{
	static uint8_t tu8_RxBuff[6] = {0};
	static uint8_t _u8_compassState = 0;
	uint8_t u8_compassReturnState = 0;
	uint8_t u8_ReturnValue;
	uint8_t u8_Size = 6;

	*pu8_Size = u8_Size;

	switch (_u8_compassState)
	{
		case 0:
			u8_compassReturnState = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_X_LSB, &tu8_RxBuff[0]);
			if (u8_compassReturnState != 0)
			{
				_u8_compassState++;
			}
			break;

		case 1:
			u8_compassReturnState = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_X_MSB, &tu8_RxBuff[1]);
			if (u8_compassReturnState != 0)
			{
				_u8_compassState++;
			}
			break;
		
		case 2:
			u8_compassReturnState = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Y_LSB, &tu8_RxBuff[2]);
			if (u8_compassReturnState != 0)
			{
				_u8_compassState++;
			}
			break;

		case 3:
			u8_compassReturnState = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Y_LSB, &tu8_RxBuff[3]);
			if (u8_compassReturnState != 0)
			{
				_u8_compassState++;
			}
			break;

		case 4:
			u8_compassReturnState = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Z_MSB, &tu8_RxBuff[4]);
			if (u8_compassReturnState != 0)
			{
				_u8_compassState++;
			}
			break;
		
		case 5:
			u8_compassReturnState = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Z_LSB, &tu8_RxBuff[5]);
			if (u8_compassReturnState != 0)
			{
				_u8_compassState++;
			}
			break;

		case 6:
			for(int i = 0; i < u8_Size; i++)
			{
				pu8_RxBuff[i] = tu8_RxBuff[i];
			}
			_u8_compassState = 0;
			u8_ReturnValue = 1;
			break;
	
		default:
			_u8_compassState = 0;
			break;
	}

	return u8_ReturnValue;
}
