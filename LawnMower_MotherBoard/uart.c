//
//  uart.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 29/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>

#include "uart.h"

void UART_transmission(unsigned char dataToSend)
{
/***** Attente du buffer *****/
    while (!(UCSR0A & (1<<UDRE0)));
/***** Stock la data dans le buffer *****/
    UDR0 = dataToSend;
}

unsigned char UART_reception()
{
/***** Attente de reception *****/
    while (!(UCSR0A & (1<<RXC0)));
/***** Return received data *****/
    return UDR0;
}