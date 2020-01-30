//
//  I2c.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 28/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef I2c_h
#define I2c_h

#include <stdio.h>
#include <avr/io.h>

void I2C_start(void);
void I2C_write(unsigned char);
unsigned char I2C_read(unsigned char);
void I2C_repeat_start(void);
void I2C_stop(void);

#endif /* I2c_h */
