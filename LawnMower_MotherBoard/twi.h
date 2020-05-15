//
//  twi.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 28/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef twi_h
#define twi_h

typedef enum {
    UNKNOWN_DATA = 0x00,
    SENSOR_V = 0x01,
    SENSOR_A = 0x02,
    SENSOR_DOCK = 0x03,
    SENSOR_RAIN = 0x04,
    SONAR_FC = 0x05,
    SONAR_FL = 0x06,
    SONAR_FR = 0x07,
    SONAR_RC = 0x08,
    GPS_TIME_HOURS = 0x09,
    GPS_TIME_MINUTES = 0x0A,
    GPS_DATE_DAYS = 0x0B,
    GPS_DATE_MONTHS = 0x0C,
    GPS_LONG_DEG = 0x0D,
    GPS_LONG_MIN = 0x0E,
    GPS_LONG_DEC = 0x0F,
    GPS_LAT_DEG = 0x10,
    GPS_LAT_MIN = 0x11,
    GPS_LAT_DEC = 0x12,
    TIME_TO_MOW = 0x13
}DataAddress;
DataAddress _eDataAdress;

typedef enum {
	DATA_COMPASS_X_MSB = 0x03,
	DATA_COMPASS_X_LSB = 0x04,
	DATA_COMPASS_Z_MSB = 0x05,
	DATA_COMPASS_Z_LSB = 0x06,
	DATA_COMPASS_Y_MSB = 0x07,
	DATA_COMPASS_Y_LSB = 0x08,
	DATA_ACCELEROMETER_X_LSB = 0x32,
	DATA_ACCELEROMETER_X_MSB = 0x33,
	DATA_ACCELEROMETER_Y_LSB = 0x34,
	DATA_ACCELEROMETER_Y_MSB = 0x35,
	DATA_ACCELEROMETER_Z_LSB = 0x36,
	DATA_ACCELEROMETER_Z_MSB = 0x37
}DataRegister;

typedef enum {
	SLAVE_SENSOR = 0x02,
	SLAVE_COMPASS = 0x3C,
	SLAVE_ACCELEROMETER = 0x53
}SlaveAddress;

uint8_t TWI_getData(uint8_t, uint8_t);
void TWI_start(void);
void TWI_repeat_start(void);
void TWI_write(uint8_t, uint8_t);
void TWI_write_data(uint8_t);
uint8_t TWI_readACK(void);
uint8_t TWI_readNACK(void);
void TWI_stop(void);

#endif /* twi_h */
