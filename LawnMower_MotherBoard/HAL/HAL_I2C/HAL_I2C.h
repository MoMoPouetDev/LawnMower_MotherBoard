/*
 * HAL_I2C.h
 *
 *  Created on: 12 sept. 2022
 *      Author: morgan.venandy
 */

#ifndef HAL_HAL_I2C_HAL_I2C_H_
#define HAL_HAL_I2C_HAL_I2C_H_

#include <stdint.h>

typedef enum 
{
	E_SLAVE_READ_DATA_V,
	E_SLAVE_READ_DATA_A,
	E_SLAVE_READ_DATA_DOCK,
	E_SLAVE_READ_DATA_TIME_TO_MOW,
	E_SLAVE_READ_DATA_SONAR_FC,
	E_SLAVE_READ_DATA_SONAR_FL,
	E_SLAVE_READ_DATA_SONAR_FR,
	E_SLAVE_READ_DATA_GPS_LONG_DEG,
	E_SLAVE_READ_DATA_GPS_LONG_MIN,
	E_SLAVE_READ_DATA_GPS_LONG_DEC_MSB,
	E_SLAVE_READ_DATA_GPS_LONG_DEC_B,
	E_SLAVE_READ_DATA_GPS_LONG_DEC_LSB,
	E_SLAVE_READ_DATA_GPS_LAT_DEG,
	E_SLAVE_READ_DATA_GPS_LAT_MIN,
	E_SLAVE_READ_DATA_GPS_LAT_DEC_MSB,
	E_SLAVE_READ_DATA_GPS_LAT_DEC_B,
	E_SLAVE_READ_DATA_GPS_LAT_DEC_LSB,
	
	E_SLAVE_READ_DATA_NUMBER
}E_SLAVE_READ_DATA;

typedef enum 
{
	E_SLAVE_WRITE_DATA_LED_STATUS = E_SLAVE_READ_DATA_NUMBER,
	
	E_SLAVE_WRITE_DATA_NUMBER
}E_SLAVE_WRITE_DATA;

void HAL_I2C_Init(void);
void HAL_I2C_CompassInit(void);
void HAL_I2C_AccelInit(void);
void HAL_I2C_Write(void);
uint8_t HAL_I2C_Read(uint8_t* , uint8_t* );
uint8_t HAL_I2C_ReadAccel(uint8_t* pu8_RxBuff, uint8_t* pu8_Size);
uint8_t HAL_I2C_ReadCompass(uint8_t* pu8_RxBuff, uint8_t* pu8_Size);
uint8_t HAL_I2C_ReadSlave(uint8_t* pu8_RxBuff, uint8_t* pu8_Size);
uint8_t HAL_I2C_WriteSlave(uint8_t u8_mowerState);

#endif /* HAL_HAL_I2C_HAL_I2C_H_ */
