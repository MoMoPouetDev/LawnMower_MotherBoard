//
//  uart.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 29/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "uart.h"

void UART_transmission(uint8_t statusToSend)
{
/***** Attente du buffer *****/
    while (!(UCSR0A & (1<<UDRE0)));
/***** Stock la data dans le buffer *****/
    UDR0 = statusToSend;
}

uint8_t UART_reception()
{
/***** Attente de reception *****/
    while (!(UCSR0A & (1<<RXC0)));
/***** Return received data *****/
    return UDR0;
}

void UART_sendCommand(char *command)
{
	int i=0;
	
	while(i <= strlen(command)) {
		UART_transmission((uint8_t)command[i]);
		i++;
	}
	_delay_ms(100);
}
