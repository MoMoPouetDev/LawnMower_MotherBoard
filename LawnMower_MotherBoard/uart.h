//
//  uart.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 29/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef uart_h
#define uart_h

#include <stdio.h>
#include <avr/io.h>

void UART_transmission(unsigned char);
unsigned char UART_reception(void);

#endif /* uart_h */
