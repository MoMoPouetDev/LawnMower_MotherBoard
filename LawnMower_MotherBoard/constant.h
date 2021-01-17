//
//  constant.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 08/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef constant_h
#define constant_h

#include <stdio.h>
#include <math.h>
#include <avr/io.h>

#define F_CPU 8000000UL
#define SCL_CLOCK  400000UL

/*** Convertisseur Analogique Numerique ***/
#define PIN_ADC0_LS 0
#define PIN_ADC1_RS 1
/*** END ***/

/*** Adresses I2C ***/
#define ADDR_MASTER 0x10
#define ADDR_SLAVE_SENSOR 0x02
#define ADDR_SLAVE_COMPASS (0x0D<<1)
#define ADDR_SLAVE_ACCELEROMETER (0x53<<1)

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

/*** COMPASS ***/
#define ADDR_DATA_COMPASS_X_LSB 0x00
#define ADDR_DATA_COMPASS_X_MSB 0x01
#define ADDR_DATA_COMPASS_Y_LSB 0x02
#define ADDR_DATA_COMPASS_Y_MSB 0x03
#define ADDR_DATA_COMPASS_Z_LSB 0x04
#define ADDR_DATA_COMPASS_Z_MSB 0x05
#define ADDR_DATA_COMPASS_TEMP_LSB 0x07
#define ADDR_DATA_COMPASS_TEMP_MSB 0x08

#define DECLINATION ((0+(45.0))*(M_PI/180))
#define OFFSET (-83*(M_PI/180))

#define CALIBRATION_X_MAX 551
#define CALIBRATION_X_MIN -1211
#define CALIBRATION_Y_MAX 358
#define CALIBRATION_Y_MIN -1417

/*** ACCELEROMETER ***/
#define ADDR_DATA_ACCELEROMETER_X_LSB 0x32
#define ADDR_DATA_ACCELEROMETER_X_MSB 0x33
#define ADDR_DATA_ACCELEROMETER_Y_LSB 0x34
#define ADDR_DATA_ACCELEROMETER_Y_MSB 0x35
#define ADDR_DATA_ACCELEROMETER_Z_LSB 0x36
#define ADDR_DATA_ACCELEROMETER_Z_MSB 0x37

/*** END ***/

/*** Périmeter Wire ***/
#define WIRE_DETECTION_LIMITE 700
#define WIRE_DETECTION_MAX 800
#define WIRE_DETECTION_MIN 500

uint8_t _uWireReached;
/*** END ***/

/*** MOTOR ***/
#define HIGH_SPEED 100
#define MIDDLE_SPEED 90
#define LOW_SPEED 80
#define ENABLE_FORWARD_MOTOR_1 PORTD4
#define ENABLE_REVERSE_MOTOR_1 PORTD5
#define ENABLE_FORWARD_MOTOR_2 PORTD2
#define ENABLE_REVERSE_MOTOR_2 PORTD3

/*** END ***/

#define ERROR_DATA 0xFF

/*** BLE ***/
#define BAUD 9600
#define BAUD_PRESCALE ((F_CPU/ (16UL*BAUD))-1)
/*** END ***/

/*** GPS ***/
#define COORDINATES_BASE_LAT 49.2315928
#define COORDINATES_BASE_LONG 1.2470619

uint8_t _uTime;
uint8_t _uDate;
/*** END ***/

/*** SONAR ***/
#define SONAR_WARN 30
#define SONAR_LIMITE 20
#define SONAR_ERR 10
/*** END ***/

/*** Capteur de tension ***/
#define SENSOR_V_OK 80
#define SENSOR_V_FAIBLE_WARN 20
#define SENSOR_V_FAIBLE_ERR 10
#define SENSOR_V_EMPTY 1

uint8_t _uBattery;
/*** END ***/

volatile uint8_t _uBpStart;
volatile uint8_t _uBpForceStart;
volatile uint8_t _uBpStop;

volatile uint8_t _uFlagWatchdog;

typedef enum
{
    ON, 
	OFF
}Etat;
volatile Etat _eEtatBlade;
volatile Etat _eEtatRain;



#endif /* constant_h */
