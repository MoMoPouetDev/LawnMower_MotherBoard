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
    STATUS_updateStatusLed();
    STATUS_updateStatusError();
}

void STATUS_updateStatusLed() {
    /*** LED VERT: PB4, ORANGE: PB2, ROUGE: PB1 ***/
    switch(_eEtatMower) {
        case UNKNOWN_ETAT:
            PORTB &= ~(1<<LED_GREEN) & ~(1<<LED_ORANGE) & ~(1<<LED_RED);
            break;
            
        case TACHE_EN_COURS:
            PORTB &= ~(1<<LED_ORANGE) & ~(1<<LED_RED);
            PORTB |= (1<<LED_GREEN);
            break;
            
        case RETOUR_STATION:
            PORTB &= ~(1<<LED_GREEN) & ~(1<<LED_RED);
            PORTB |= (1<<LED_ORANGE);
            break;
            
        case EN_CHARGE:
            PORTB &= ~(1<<LED_GREEN) & ~(1<<LED_ORANGE) & ~(1<<LED_RED);
            _delay_ms(500);
            PORTB |= (1<<PORT4) | (1<<LED_ORANGE) | (1<<LED_RED);
            _delay_ms(500);
            break;
            
        case PAS_DE_TACHE_EN_COURS:
            PORTB &= ~(1<<LED_GREEN) & ~(1<<LED_ORANGE);
            PORTB |= (1<<LED_RED);
            break;
            
        case PAUSE:
            PORTB &= ~(1<<LED_GREEN) & ~(1<<LED_ORANGE) & ~(1<<LED_RED);
            _delay_ms(500);
            PORTB |= (1<<LED_GREEN);
            _delay_ms(500);
            break;
            
        default:
            PORTB &= ~(1<<LED_GREEN) & ~(1<<LED_ORANGE) & ~(1<<LED_RED);
            break;
    }
}

void STATUS_updateStatusError() {
    /*** LED JAUNE1: PB5, JAUNE2: PC2, JAUNE3: PC3 ***/
    switch(_eErrorMower) {
        case NTR:
            PORTB &= ~(1<<LED_YELLOW_1);
            PORTC &= ~(1<<LED_YELLOW_3) & ~(1<<LED_YELLOW_2);
            break;
            
        case BLOCKED_MOWER:
            PORTB &= ~(1<<LED_YELLOW_1);
            PORTC &= ~(1<<LED_YELLOW_2);
            PORTC |= (1<<LED_YELLOW_3);
            break;
            
        case DETECTED_RAIN:
            PORTB &= ~(1<<LED_YELLOW_1);
            PORTC &= ~(1<<LED_YELLOW_3);
            PORTC |= (1<<LED_YELLOW_2);
            break;
            
        case WIRE_NOT_DETECTED:
            PORTB &= ~(1<<LED_YELLOW_1);
            PORTC |= (1<<LED_YELLOW_2) | (1<<LED_YELLOW_3);
            break;
            
        case LOW_BATTERY:
            PORTC &= ~(1<<LED_YELLOW_3) & ~(1<<LED_YELLOW_2);
            PORTB |= (1<<LED_YELLOW_1);
            break;
            
        case VERY_LOW_BATTERY:
            PORTC &= ~(1<<LED_YELLOW_2);
            PORTB |= (1<<LED_YELLOW_1);
            PORTC |= (1<<LED_YELLOW_3);
            break;
            
        case EMPTY_BATTERY:
            PORTC &= ~(1<<LED_YELLOW_3);
            PORTB |= (1<<LED_YELLOW_1);
            PORTC |= (1<<LED_YELLOW_2);
            break;
            
        default:
            PORTB &= ~(1<<LED_YELLOW_1);
            PORTC &= ~(1<<LED_YELLOW_3) & ~(1<<LED_YELLOW_2);
            break;
    }
}

void STATUS_sendStatus() {
    UART_transmission(_eEtatMower);
    UART_transmission(_eErrorMower);
    UART_transmission(_uBattery);
    
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_LAT_DEG));
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_LAT_MIN));
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_LAT_DEC));
	
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_LONG_DEG));
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_LONG_MIN));
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_LONG_DEC));
	
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_TIME_HOURS));
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_TIME_MINUTES));
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_DATE_DAYS));
	UART_transmission(TWI_getData(SLAVE_SENSOR, GPS_DATE_MONTHS));
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
