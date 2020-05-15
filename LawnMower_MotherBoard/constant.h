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
#include <avr/io.h>

#define F_CPU 8000000UL

#define PIN_ADC0 0
#define PIN_ADC1 1

#define WIRE_DETECTION_LIMITE 820
#define WIRE_DETECTION_MAX 1000
#define WIRE_DETECTION_MIN 500

#define HIGH_SPEED 100
#define MIDDLE_SPEED 50
#define LOW_SPEED 20
#define ENABLE_MOTOR_1 PORTD2
#define ENABLE_MOTOR_2 PORTD4

#define ADDR_MASTER 0x10
#define ADDR_SLAVE_SENSOR 0x02
#define ADDR_SLAVE_COMPASS 0x3C
#define ADDR_SLAVE_ACC 0x53

#define ERROR_DATA 0xFF

#define BAUD 9600
#define BAUD_PRESCALE ((F_CPU/ (16UL*BAUD))-1)

#define COORDINATES_BASE_LAT 49.2315928
#define COORDINATES_BASE_LONG 1.2470619

#define SONAR_WARN 30
#define SONAR_ERR 10

#define SENSOR_V_OK 80
#define SENSOR_V_FAIBLE_WARN 20
#define SENSOR_V_FAIBLE_ERR 10
#define SENSOR_V_EMPTY 1

#define LED_GREEN PORTB4
#define LED_ORANGE PORTB2
#define LED_RED PORTB1
#define LED_YELLOW_1 PORTB5
#define LED_YELLOW_2 PORTC2
#define LED_YELLOW_3 PORTC3

uint8_t _uTime;
uint8_t _uDate;

uint8_t _uBattery;

uint8_t _uBpStart;
uint8_t _uBpForceStart;
uint8_t _uBpStop;

uint8_t _uWireReached;

uint8_t _uFlagWatchdog;
uint8_t _FlagError;

typedef enum
{
    ON, 
	OFF
}Etat;
Etat _eEtatBlade;
Etat _eEtatRain;



#endif /* constant_h */
