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

#define HIGH_SPEED 100
#define MIDDLE_SPEED 50
#define LOW_SPEED 20

#define ADDR_SLAVE_SENSOR 0x01
#define ADDR_SLAVE_SONAR_GPS 0x02
#define ADDR_SLAVE_COMPASS 0x03
#define I2C_WRITE   0
#define I2C_READ    1

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define SONAR_DOCKING 3
#define SONAR_WARN 30
#define SONAR_ERR 10
uint8_t distanceSonarFC;
uint8_t distanceSonarFL;
uint8_t distanceSonarFR;
uint8_t distanceSonarRC;

#define SENSOR_V_FAIBLE_WARN 20
#define SENSOR_V_FAIBLE_ERR 10
#define SENSOR_V_EMPTY 1
uint8_t batteryLevel;
uint8_t inCharge;
uint8_t underTheRain;

typedef enum Etat Etat;
enum Etat
{
    ON, OFF
};
Etat etatBlade;

#endif /* constant_h */
