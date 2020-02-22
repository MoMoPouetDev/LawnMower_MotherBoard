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

ISR(PCINT2_vect)
{
    
}

ISR(PCINT0_vect)
{
    
}