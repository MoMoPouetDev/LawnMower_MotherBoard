//
//  status.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 21/02/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "constant.h"
#include "mower.h"
#include "status.h"
#include "uart.h"
#include "twi.h"

void STATUS_updateStatus() {
    uint8_t uStatus = 0x00;
    
    STATUS_updateStatusLed(&uStatus);
    STATUS_updateStatusError(&uStatus);
    
    TWI_setData(ADDR_SLAVE_SENSOR, ADDR_LED_STATUS, uStatus);
}

void STATUS_updateStatusLed(uint8_t* status) {
    /*** LED VERT: PB4, ORANGE: PB2, ROUGE: PB1 ***/
    *status |= _eEtatMower;
}

void STATUS_updateStatusError(uint8_t* status) {
    /*** LED JAUNE1: PB5, JAUNE2: PC2, JAUNE3: PC3 ***/
    *status |= _eErrorMower;
}

void STATUS_sendStatus() {
	Coordinates tLatitude;
	Coordinates tLongitude;
	uint8_t uHoursGPS;
	uint8_t uMinutesGPS;
	uint8_t uDaysGPS;
	uint8_t uMonthsGPS;
	uint8_t angleLSB, angleMSB;
	uint16_t angleW;
	float angle;
	
	tLatitude.degrees = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEG);
	tLatitude.minutes = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_MIN);
	tLatitude.decimalMSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_MSB);
	tLatitude.decimalB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_B);
	tLatitude.decimalLSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_LSB);
	
	tLongitude.degrees = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEG);
	tLongitude.minutes = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_MIN);
	tLongitude.decimalMSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_MSB);
	tLongitude.decimalB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_B);
	tLongitude.decimalLSB = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_LSB);
	
	uHoursGPS = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_TIME_HOURS);
	uMinutesGPS = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_TIME_MINUTES);
	uDaysGPS = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_DATE_DAYS);
	uMonthsGPS = TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_DATE_MONTHS);
	
	angle = MOWER_getAngleFromNorth();
	angleW = (uint16_t) angle;
	angleLSB = angleW & 0xFF;
	angleMSB = (angleW >> 8) & 0xFF;
	
    UART_transmission(_eEtatMower);
    UART_transmission(_eErrorMower);
    UART_transmission(_uBattery);
	
	UART_transmission(tLatitude.degrees);
	UART_transmission(tLatitude.minutes);
	UART_transmission(tLatitude.decimalMSB);
	UART_transmission(tLatitude.decimalB);
	UART_transmission(tLatitude.decimalLSB);
	
	UART_transmission(tLongitude.degrees);
	UART_transmission(tLongitude.minutes);
	UART_transmission(tLongitude.decimalMSB);
	UART_transmission(tLongitude.decimalB);
	UART_transmission(tLongitude.decimalLSB);
	
	UART_transmission(uHoursGPS);
	UART_transmission(uMinutesGPS);
	UART_transmission(uDaysGPS);
	UART_transmission(uMonthsGPS);
	
	UART_transmission(angleMSB);
	UART_transmission(angleLSB);
}

void STATUS_receivedStatus() {
    _eCommandMower = UDR0;
    
    switch(_eCommandMower) {
        case START:
            _uBpStop = 0;
            _uBpStart ^= (1<<1);
            break;
            
        case STOP:
            _uBpStop = 1;
            if((_eEtatRain == ON) && (isDocking()))
                _eEtatRain = OFF;
            break;
            
        case FORCE_START:
            _uBpForceStart = 1;
            break;
            
        default:
            break;
    }
}
