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
    UART_transmission(_eEtatMower);
    UART_transmission(_eErrorMower);
    UART_transmission(_uBattery);
    
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEG));
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_MIN));
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_MSB));
    UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_B));
    UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LAT_DEC_LSB));
	
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEG));
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_MIN));
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_MSB));
    UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_B));
    UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_LONG_DEC_LSB));
	
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_TIME_HOURS));
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_TIME_MINUTES));
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_DATE_DAYS));
	UART_transmission(TWI_getData(ADDR_SLAVE_SENSOR, ADDR_GPS_DATE_MONTHS));
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
