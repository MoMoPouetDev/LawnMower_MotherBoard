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
void HAL_I2C_UnlockBus(void)
{
	LLD_I2C_UnlockBus();
}

void HAL_I2C_Reset(void)
{
	LLD_I2C_Reset();
}

uint8_t HAL_I2C_GetErrorFlag(void)
{
	return LLD_I2C_GetErrorFlag();
}

void HAL_I2C_Init(void)
{
	LLD_I2C_Init();
}

void HAL_I2C_CompassInit(void)
{
	LLD_I2C_InitCompass(COMPASS_ADDR);
}

void HAL_I2C_AccelInit(void)
{
	LLD_I2C_InitAccel(ACCELEROMETRE_ADDR);
}

uint8_t HAL_I2C_ReadAccel(uint8_t* pu8_rxBuff, uint8_t* pu8_Size)
{
	static uint8_t _u8_accelState = 0;
	uint8_t u8_ReturnValue = 0;

	switch (_u8_accelState)
	{
		case 0:
    		*(pu8_rxBuff+_u8_accelState) = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_X_LSB);
			_u8_accelState++;
			break;

		case 1:
    		*(pu8_rxBuff+_u8_accelState) = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_X_MSB);
			_u8_accelState++;
			break;
    
		case 2:
    		*(pu8_rxBuff+_u8_accelState) = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Y_LSB);
			_u8_accelState++;
			break;

		case 3:
    		*(pu8_rxBuff+_u8_accelState) = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Y_MSB);
			_u8_accelState++;
			break;

		case 4:    
    		*(pu8_rxBuff+_u8_accelState) = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Z_LSB);
			_u8_accelState++;
			break;
		
		case 5:
    		*(pu8_rxBuff+_u8_accelState) = LLD_I2C_Read(ACCELEROMETRE_ADDR, ADDR_DATA_ACCELEROMETER_Z_MSB);			
			_u8_accelState++;	
			break;

		case 6:
			*pu8_Size = _u8_accelState;
			_u8_accelState = 0;
			u8_ReturnValue = 1;
			break;
	
		default:
			_u8_accelState = 0;
			break;
	}

	return u8_ReturnValue;
}

uint8_t HAL_I2C_ReadCompass(uint8_t* pu8_rxBuff, uint8_t* pu8_Size)
{
	static uint8_t _u8_compassState = 0;
	uint8_t u8_ReturnValue = 0;

	switch (_u8_compassState)
	{
		case 0:
    		*(pu8_rxBuff+_u8_compassState) = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_X_LSB);
			_u8_compassState++;
			break;

		case 1:
    		*(pu8_rxBuff+_u8_compassState) = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_X_MSB);
			_u8_compassState++;
			break;
    
		case 2:
    		*(pu8_rxBuff+_u8_compassState) = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Y_LSB);
			_u8_compassState++;
			break;

		case 3:
    		*(pu8_rxBuff+_u8_compassState) = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Y_MSB);
			_u8_compassState++;
			break;

		case 4:    
    		*(pu8_rxBuff+_u8_compassState) = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Z_LSB);
			_u8_compassState++;
			break;
		
		case 5:
    		*(pu8_rxBuff+_u8_compassState) = LLD_I2C_Read(COMPASS_ADDR, ADDR_DATA_COMPASS_Z_MSB);			
			_u8_compassState++;	
			break;

		case 6:
			*pu8_Size = _u8_compassState;
			_u8_compassState = 0;
			u8_ReturnValue = 1;
			break;
	
		default:
			_u8_compassState = 0;
			break;
	}

	return u8_ReturnValue;
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
