//
//  I2c.h
//  LawnMower_MotherBoard
//
//  Created by morgan on 28/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#ifndef I2c_h
#define I2c_h

void I2C_start(void);
void I2C_write(uint8_t, uint8_t);
uint8_t I2C_readACK(void);
uint8_t I2C_readNACK(void);
void I2C_stop(void);

#endif /* I2c_h */
