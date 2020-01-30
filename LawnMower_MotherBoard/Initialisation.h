//
//  Initialisation.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 25/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef Initialisation_h
#define Initialisation_h

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

void Initialisation(void);
void InitIO(void);
void InitPWM(void);
void InitInterrupt(void);
void InitI2C(void);
void InitUART(unsigned int);


#endif /* Initialisation_h */
