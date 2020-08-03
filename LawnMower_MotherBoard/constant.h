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
#define SCL_CLOCK  400000L

/*** Convertisseur Analogique Numerique ***/
#define PIN_ADC0 0
#define PIN_ADC1 1
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
#define WIRE_DETECTION_LIMITE 820
#define WIRE_DETECTION_MAX 1000
#define WIRE_DETECTION_MIN 500

uint8_t _uWireReached;
/*** END ***/

/*** MOTOR ***/
#define HIGH_SPEED 100
#define MIDDLE_SPEED 50
#define LOW_SPEED 20
#define ENABLE_MOTOR_1 PORTD2
#define ENABLE_MOTOR_2 PORTD4
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
#define SONAR_ERR 10
/*** END ***/

/*** Capteur de tension ***/
#define SENSOR_V_OK 80
#define SENSOR_V_FAIBLE_WARN 20
#define SENSOR_V_FAIBLE_ERR 10
#define SENSOR_V_EMPTY 1

uint8_t _uBattery;
/*** END ***/

/*** LED ***/
#define LED_GREEN PORTB4
#define LED_ORANGE PORTB2
#define LED_RED PORTB1
#define LED_YELLOW_1 PORTB5
#define LED_YELLOW_2 PORTC2
#define LED_YELLOW_3 PORTC3
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
