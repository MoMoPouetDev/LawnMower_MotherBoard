//
//  Interrupt.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 29/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "constant.h"
#include "uart.h"
#include "status.h"
#include "mower.h"

ISR(USART_RX_vect)
{
    STATUS_receivedStatus();
}

ISR(WDT_vect) {
	_uFlagWatchdog = 1;
}

ISR(PCINT2_vect)
{
    //BP Start
	if(!(PIND & (1<<PIND7))) {
		if(!isDocking()) {
			_uBpStop = 0;
			_uBpStart ^= (1<<1);
		}
		else
			_uBpForceStart = 1;
		
	}
}

ISR(PCINT0_vect)
{
    //BP Stop
	if(!(PINB & (1<<PINB0))) {
		_uBpStop = 1;
		if((_eEtatRain == ON) && (isDocking()))
			_eEtatRain = OFF;
	}
}
