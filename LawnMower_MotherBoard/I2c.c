//
//  I2c.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 28/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include "I2c.h"

#define ADDR_SLAVE_1 0x01
#define ADDR_SLAVE_2 0x02
#define I2C_WRITE   0
#define I2C_READ    1

void I2C_start()
{
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);
    while (!(TWCR & (1<<TWINT)));
}

void I2C_write(unsigned char addrSlave)
{
    TWDR = addrSlave + I2C_WRITE;
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

void I2C_write_data(unsigned char dataToSend)
{
    TWDR = dataToSend;
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
}

unsigned char I2C_read(unsigned char addrSlave)
{
    TWDR = addrSlave + I2C_READ;
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

void I2C_repeat_start()
{
    I2C_start();
}

void I2C_stop()
{
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

