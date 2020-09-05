//
//  twi.c
//  LawnMower_MotherBoard
//
//  Created by morgan on 28/01/2020.
//  Copyright © 2020 morgan venandy. All rights reserved.
//

#include <stdio.h>
#include <avr/io.h>
#include <util/twi.h>
#include <avr/wdt.h>

#include "constant.h"
#include "twi.h"

uint8_t TWI_getData(uint8_t addrSlave, uint8_t addrData) {
    volatile uint8_t receivedData = 0;
	
	wdt_reset();
    _uFlagWatchdog = 0;
    
    TWI_start();
    TWI_write(addrSlave, TW_WRITE);
    TWI_write_data(addrData);
    
    TWI_repeat_start();
    TWI_write(addrSlave, TW_READ);
    receivedData = TWI_readNACK();
    
    TWI_stop();
    
	if(_uFlagWatchdog)
		receivedData = ERROR_DATA;
    
    return receivedData;
}

void TWI_setData(uint8_t addrSlave, uint8_t addrData, uint8_t data) {
    wdt_reset();
    _uFlagWatchdog = 0;
    
    TWI_start();
    TWI_write(addrSlave, TW_WRITE);
    TWI_write_data(addrData);
    TWI_write_data(data);
    
    TWI_stop();
}

void TWI_start()
{
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);
    while (!(TWCR & (1<<TWINT)) && !(_uFlagWatchdog));
    while(!((TWSR & 0xF8) == TW_START) && !(_uFlagWatchdog));
}

void TWI_repeat_start()
{
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWSTA);
    while (!(TWCR & (1<<TWINT)) && !(_uFlagWatchdog));
    while(!((TWSR & 0xF8) == TW_REP_START) && !(_uFlagWatchdog));
}

void TWI_write(uint8_t addrSlave, uint8_t twi_read_write)
{
    TWDR = addrSlave + twi_read_write;
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)) && !(_uFlagWatchdog));
    if (twi_read_write)
        while (!((TWSR & 0xF8) == TW_MR_SLA_ACK) && !(_uFlagWatchdog));
    else
        while (!((TWSR & 0xF8) == TW_MT_SLA_ACK) && !(_uFlagWatchdog));
}

void TWI_write_data(uint8_t dataToSend)
{
    TWDR = dataToSend;
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
    while (!(TWCR & (1<<TWINT)) && !(_uFlagWatchdog));
    while (!((TWSR & 0xF8) == TW_MT_DATA_ACK) && !(_uFlagWatchdog));
}

uint8_t TWI_readACK()
{
    TWCR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
    while (!(TWCR & (1<<TWINT)) && !(_uFlagWatchdog));
    while (!((TWSR & 0xF8) == TW_MR_DATA_ACK) && !(_uFlagWatchdog));
    
    if (_uFlagWatchdog)
        return 0;
    else
        return TWDR;
}

uint8_t TWI_readNACK()
{
    TWCR = (1<<TWEN) | (1<<TWINT);
    while (!(TWCR & (1<<TWINT)) && !(_uFlagWatchdog));
    while (!((TWSR & 0xF8) == TW_MR_DATA_NACK) && !(_uFlagWatchdog));

    if (_uFlagWatchdog)
        return 0;
    else
        return TWDR;
}

void TWI_stop()
{
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

